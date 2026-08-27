"""Point-cloud segmentation and truck-board geometry pipeline.

Current production flow:
    PCD -> ROI -> voxel downsample -> main-plane removal -> DBSCAN
    -> statistical filter -> PointNet++ full-cloud inference
    -> board-plane RANSAC -> PCA/UV slicing -> 4-edge RANSAC
    -> 3D corners / geometry / attitude -> first-workface loading plan
    -> output/json/<name>.json

Only per-file JSON results are written to disk.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

try:
    import open3d as o3d
except ImportError:
    o3d = None

try:
    from . import PointNet2Segmenter
    from .inference import DEFAULT_NUM_CLASSES, DEFAULT_NUM_PART, pc_normalize
except ImportError as exc:
    PointNet2Segmenter = None
    POINTNET_IMPORT_ERROR = exc
else:
    POINTNET_IMPORT_ERROR = None


# ============================================================================
# 用户可调参数区
# 现场调试时，优先只修改这里。
# ============================================================================
@dataclass(frozen=True)
class PipelineConfig:
    # ------------------------------------------------------------------------
    # 1. ROI 裁剪范围（mm）
    # ------------------------------------------------------------------------
    roi_min: tuple[float, float, float] = (1000.0, -10000.0, -200.0)
    roi_max: tuple[float, float, float] = (8000.0, 13000.0, 3000.0)
    roi_redundancy: float = 0.025

    # ------------------------------------------------------------------------
    # 2. 点云预处理
    # ------------------------------------------------------------------------
    voxel_size: float = 20.0

    # 地面/主平面 RANSAC
    plane_threshold: float = 140.0
    plane_ransac_n: int = 3
    plane_iterations: int = 500

    # DBSCAN
    dbscan_eps: float = 100.0
    dbscan_min_points: int = 50

    # 统计离群点滤波
    stat_neighbors: int = 6
    stat_std_ratio: float = 3.0

    # ------------------------------------------------------------------------
    # 3. PointNet++
    # 当前主流程是“整云一次推理”，不做空间分批、重叠或 KNN 恢复。
    # pointnet_num_point 仍用于 PointNet2Segmenter 初始化/兼容。
    # ------------------------------------------------------------------------
    pointnet_num_point: int = 49152
    pointnet_num_votes: int = 1
    target_labels: tuple[int, int] = (2, 3)
    seed: int = 42

    # ------------------------------------------------------------------------
    # 4. 底板平面 RANSAC
    # ------------------------------------------------------------------------
    edge_plane_distance: float = 100.0
    edge_plane_ransac_n: int = 5
    edge_plane_iterations: int = 800
    edge_min_inliers: int = 3000

    # ------------------------------------------------------------------------
    # 5. PCA + 分段切片边缘提取
    # U = 车长方向；V = 车宽方向
    # ------------------------------------------------------------------------
    slice_width_u: float = 50.0
    slice_width_v: float = 30.0
    edge_u_bins_min: int = 80
    edge_u_bins_max: int = 350
    edge_v_bins_min: int = 50
    edge_v_bins_max: int = 220
    slice_min_points: int = 20

    # ------------------------------------------------------------------------
    # 6. 四边 RANSAC
    # ------------------------------------------------------------------------
    line_ransac_distance: float = 30.0
    line_ransac_iterations: int = 300
    line_min_inliers: int = 25

    # ------------------------------------------------------------------------
    # 7. 车辆方向定义
    # 指向车头的大致世界坐标方向，用于固定 Umin = 车头端。
    # ------------------------------------------------------------------------
    truck_head_dir: tuple[float, float, float] = (-1.0, 1.0, 0.0)

    # ------------------------------------------------------------------------
    # 8. 第一作业面装载规划
    # 仅当 label_2 与 label_3 都成功识别时，对 label_2 计算。
    # ------------------------------------------------------------------------
    pallet_length_mm: float = 1200.0
    pad_trigger_ratio: float = 0.50


DEFAULT_CONFIG = PipelineConfig()


def calculate_roi_bounds(
    roi_min,
    roi_max,
    redundancy_ratio: float = 0.025,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    low = np.asarray(roi_min, dtype=np.float64)
    high = np.asarray(roi_max, dtype=np.float64)
    delta = high - low
    redundancy = np.where(delta > 1e-6, delta * float(redundancy_ratio), 0.0)
    return tuple(low - redundancy), tuple(high + redundancy)


def collect_pcd_files(input_path: str | Path) -> list[Path]:
    path = Path(input_path).expanduser().resolve()
    if path.is_file():
        if path.suffix.lower() != ".pcd":
            raise ValueError(f"输入文件不是 PCD：{path}")
        return [path]
    if path.is_dir():
        files = sorted(
            candidate
            for candidate in path.iterdir()
            if candidate.is_file() and candidate.suffix.lower() == ".pcd"
        )
        if files:
            return files
        raise FileNotFoundError(f"目录中未找到 PCD 文件：{path}")
    raise FileNotFoundError(f"输入路径不存在：{path}")


def require_open3d():
    if o3d is None:
        raise RuntimeError("缺少 open3d，请先在运行环境中安装 open3d。")
    return o3d


def read_point_cloud(path: str | Path):
    backend = require_open3d()
    point_cloud = backend.io.read_point_cloud(str(path))
    if len(point_cloud.points) == 0:
        raise ValueError(f"点云为空或读取失败：{path}")
    return point_cloud


def write_json(path: str | Path, payload) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def preprocess_point_cloud_timed(pcd, config: PipelineConfig):
    backend = require_open3d()
    original_count = len(pcd.points)
    if original_count == 0:
        raise ValueError("输入点云为空")

    timings = {}
    total_start = time.perf_counter()

    step_start = time.perf_counter()
    low, high = calculate_roi_bounds(
        config.roi_min,
        config.roi_max,
        config.roi_redundancy,
    )
    bounds = backend.geometry.AxisAlignedBoundingBox(min_bound=low, max_bound=high)
    roi = pcd.crop(bounds)
    timings["roi_crop_s"] = time.perf_counter() - step_start
    if len(roi.points) == 0:
        raise ValueError("ROI 裁剪后点云为空")

    step_start = time.perf_counter()
    downsampled = roi.voxel_down_sample(config.voxel_size)
    timings["voxel_downsample_s"] = time.perf_counter() - step_start
    if len(downsampled.points) < config.plane_ransac_n:
        raise ValueError("降采样后点数不足以进行平面拟合")

    step_start = time.perf_counter()
    _, plane_indices = downsampled.segment_plane(
        distance_threshold=config.plane_threshold,
        ransac_n=config.plane_ransac_n,
        num_iterations=config.plane_iterations,
    )
    timings["ground_plane_ransac_s"] = time.perf_counter() - step_start

    step_start = time.perf_counter()
    non_plane = downsampled.select_by_index(plane_indices, invert=True)
    timings["ground_plane_remove_s"] = time.perf_counter() - step_start
    if len(non_plane.points) == 0:
        raise ValueError("平面过滤后点云为空")

    step_start = time.perf_counter()
    cluster_labels = np.asarray(
        non_plane.cluster_dbscan(
            eps=config.dbscan_eps,
            min_points=config.dbscan_min_points,
            print_progress=False,
        )
    )
    timings["dbscan_s"] = time.perf_counter() - step_start
    valid_labels = cluster_labels[cluster_labels >= 0]
    if valid_labels.size == 0:
        raise ValueError("DBSCAN 没有找到有效聚类")

    step_start = time.perf_counter()
    unique_labels, cluster_sizes = np.unique(valid_labels, return_counts=True)
    largest_label = unique_labels[np.argmax(cluster_sizes)]
    largest_cluster = non_plane.select_by_index(
        np.flatnonzero(cluster_labels == largest_label)
    )
    timings["largest_cluster_select_s"] = time.perf_counter() - step_start

    step_start = time.perf_counter()
    statistical, _ = largest_cluster.remove_statistical_outlier(
        nb_neighbors=config.stat_neighbors,
        std_ratio=config.stat_std_ratio,
    )
    timings["statistical_filter_s"] = time.perf_counter() - step_start
    if len(statistical.points) == 0:
        raise ValueError("统计过滤后点云为空")

    timings["preprocess_total_s"] = time.perf_counter() - total_start
    counts = {
        "original": original_count,
        "roi": len(roi.points),
        "downsampled": len(downsampled.points),
        "plane_filtered": len(non_plane.points),
        "cluster_filtered": len(largest_cluster.points),
        "stat_filtered": len(statistical.points),
    }
    return statistical, counts, timings


def normalize_vector(vector) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        return vector
    return vector / norm


def fit_plane_once(points, config: PipelineConfig):
    if len(points) < config.edge_plane_ransac_n:
        return None, None
    backend = require_open3d()
    pcd = backend.geometry.PointCloud()
    pcd.points = backend.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=config.edge_plane_distance,
        ransac_n=config.edge_plane_ransac_n,
        num_iterations=config.edge_plane_iterations,
    )
    return plane_model, np.asarray(inliers, dtype=np.int32)


def get_ransac_normal(plane_model) -> np.ndarray:
    normal = normalize_vector(np.asarray(plane_model[:3], dtype=np.float64))
    if normal[2] < 0:
        normal = -normal
    return normal


def compute_pca_info(points, plane_model, truck_head_dir):
    """建立真实朝向的底板局部 UVN 坐标系。

    U：车长方向；Umin 固定为车头端。
    V：车宽方向。
    N：拟合平面法向，整体朝 world +Z。
    """

    points = np.asarray(points, dtype=np.float64)
    raw_center = np.mean(points, axis=0)

    plane = np.asarray(plane_model, dtype=np.float64)
    plane_normal_raw = plane[:3]
    plane_normal_sq = float(np.dot(plane_normal_raw, plane_normal_raw))
    if plane_normal_sq < 1e-12:
        raise ValueError("RANSAC 平面法向量无效")

    signed_scale = (
        float(np.dot(plane_normal_raw, raw_center)) + float(plane[3])
    ) / plane_normal_sq
    center = raw_center - signed_scale * plane_normal_raw

    centered = points - raw_center
    covariance = np.cov(centered, rowvar=False)
    _, eigenvectors = np.linalg.eigh(covariance)
    eigenvectors = eigenvectors[:, ::-1]

    long_dir = normalize_vector(eigenvectors[:, 0])
    normal_dir = get_ransac_normal(plane_model)
    if normal_dir[2] < 0:
        normal_dir = -normal_dir

    long_dir = normalize_vector(
        long_dir - np.dot(long_dir, normal_dir) * normal_dir
    )
    short_dir = normalize_vector(np.cross(normal_dir, long_dir))

    head_dir = np.asarray(truck_head_dir, dtype=np.float64)
    head_dir = head_dir - np.dot(head_dir, normal_dir) * normal_dir
    if np.linalg.norm(head_dir) < 1e-12:
        raise ValueError("truck_head_dir 投影后与拟合平面法向平行")
    head_dir = normalize_vector(head_dir)

    # Umin 位于 -long_dir 一侧，要求它朝向车头。
    if np.dot(-long_dir, head_dir) < 0:
        long_dir = -long_dir
        short_dir = -short_dir

    return {
        "center": center,
        "u_dir": long_dir,
        "v_dir": short_dir,
        "normal_dir": normal_dir,
    }


def project_points_to_uvn(points, pca_info):
    points = np.asarray(points, dtype=np.float64)
    centered = points - pca_info["center"]
    return (
        centered @ pca_info["u_dir"],
        centered @ pca_info["v_dir"],
        centered @ pca_info["normal_dir"],
    )


def uv_to_3d(u, v, pca_info):
    return (
        pca_info["center"]
        + float(u) * pca_info["u_dir"]
        + float(v) * pca_info["v_dir"]
    )


def compute_dynamic_bins(uv, config: PipelineConfig):
    u_range = float(np.ptp(uv[:, 0]))
    v_range = float(np.ptp(uv[:, 1]))
    u_bins = int(
        np.clip(
            np.ceil(u_range / config.slice_width_u),
            config.edge_u_bins_min,
            config.edge_u_bins_max,
        )
    )
    v_bins = int(
        np.clip(
            np.ceil(v_range / config.slice_width_v),
            config.edge_v_bins_min,
            config.edge_v_bins_max,
        )
    )
    return u_bins, v_bins


def line_from_two_points(point_1, point_2):
    point_1 = np.asarray(point_1, dtype=np.float64)
    point_2 = np.asarray(point_2, dtype=np.float64)
    direction = point_2 - point_1
    if np.linalg.norm(direction) < 1e-12:
        return None
    a = direction[1]
    b = -direction[0]
    norm = np.hypot(a, b)
    if norm < 1e-12:
        return None
    a /= norm
    b /= norm
    c = -(a * point_1[0] + b * point_1[1])
    return np.asarray([a, b, c], dtype=np.float64)


def line_distances(points_2d, line):
    a, b, c = line
    return np.abs(a * points_2d[:, 0] + b * points_2d[:, 1] + c)


def refit_line_svd(points_2d):
    points_2d = np.asarray(points_2d, dtype=np.float64)
    if len(points_2d) < 2:
        return None
    center = np.mean(points_2d, axis=0)
    _, _, right_vectors = np.linalg.svd(points_2d - center, full_matrices=False)
    direction = normalize_vector(right_vectors[0])
    normal = normalize_vector(
        np.asarray([direction[1], -direction[0]], dtype=np.float64)
    )
    a, b = normal
    c = -(a * center[0] + b * center[1])
    return np.asarray([a, b, c], dtype=np.float64)


def ransac_fit_line_2d(points_2d, config: PipelineConfig, rng):
    points_2d = np.asarray(points_2d, dtype=np.float64)
    if len(points_2d) < max(2, config.line_min_inliers):
        return None

    best_inliers = None
    best_count = 0
    for _ in range(config.line_ransac_iterations):
        first, second = rng.choice(len(points_2d), size=2, replace=False)
        line = line_from_two_points(points_2d[first], points_2d[second])
        if line is None:
            continue
        inliers = np.flatnonzero(
            line_distances(points_2d, line) <= config.line_ransac_distance
        )
        if len(inliers) > best_count:
            best_count = len(inliers)
            best_inliers = inliers

    if best_inliers is None or len(best_inliers) < config.line_min_inliers:
        return None

    refined_line = refit_line_svd(points_2d[best_inliers])
    if refined_line is None:
        return None

    refined_inliers = np.flatnonzero(
        line_distances(points_2d, refined_line) <= config.line_ransac_distance
    )
    if len(refined_inliers) < config.line_min_inliers:
        return None

    return refit_line_svd(points_2d[refined_inliers])


def line_intersection(line_1, line_2):
    if line_1 is None or line_2 is None:
        return None
    a_1, b_1, c_1 = line_1
    a_2, b_2, c_2 = line_2
    coefficients = np.asarray([[a_1, b_1], [a_2, b_2]], dtype=np.float64)
    values = np.asarray([-c_1, -c_2], dtype=np.float64)
    if abs(np.linalg.det(coefficients)) < 1e-9:
        return None
    return np.linalg.solve(coefficients, values)


def pick_edge_point(points_bin, edge_type):
    points_bin = np.asarray(points_bin, dtype=np.float64)
    if points_bin.ndim != 2 or points_bin.shape[1] != 2 or len(points_bin) == 0:
        raise ValueError(f"边缘候选输入必须为非空 [N,2]，当前为 {points_bin.shape}")

    if edge_type == "v_min":
        return points_bin[np.argmin(points_bin[:, 1])]
    if edge_type == "v_max":
        return points_bin[np.argmax(points_bin[:, 1])]
    if edge_type == "u_min":
        return points_bin[np.argmin(points_bin[:, 0])]
    if edge_type == "u_max":
        return points_bin[np.argmax(points_bin[:, 0])]
    raise ValueError(f"不支持的边缘类型：{edge_type}")


def extract_edge_candidates_2d(uv, config: PipelineConfig):
    uv = np.asarray(uv, dtype=np.float64)
    if uv.ndim != 2 or uv.shape[1] != 2 or len(uv) == 0:
        raise ValueError(f"UV 点必须为非空 [N,2]，当前为 {uv.shape}")

    u = uv[:, 0]
    v = uv[:, 1]
    u_bin_count, v_bin_count = compute_dynamic_bins(uv, config)
    candidates = {"v_min": [], "v_max": [], "u_min": [], "u_max": []}

    # 沿 U（长度方向）切片，找 Vmin / Vmax 两条纵向侧边。
    u_edges = np.linspace(np.min(u), np.max(u), u_bin_count + 1)
    for index in range(u_bin_count):
        upper_test = (
            u <= u_edges[index + 1]
            if index == u_bin_count - 1
            else u < u_edges[index + 1]
        )
        points_bin = uv[(u >= u_edges[index]) & upper_test]
        if len(points_bin) < config.slice_min_points:
            continue
        candidates["v_min"].append(pick_edge_point(points_bin, "v_min"))
        candidates["v_max"].append(pick_edge_point(points_bin, "v_max"))

    # 沿 V（宽度方向）切片，找 Umin / Umax 车头端与车尾端。
    v_edges = np.linspace(np.min(v), np.max(v), v_bin_count + 1)
    for index in range(v_bin_count):
        upper_test = (
            v <= v_edges[index + 1]
            if index == v_bin_count - 1
            else v < v_edges[index + 1]
        )
        points_bin = uv[(v >= v_edges[index]) & upper_test]
        if len(points_bin) < config.slice_min_points:
            continue
        candidates["u_min"].append(pick_edge_point(points_bin, "u_min"))
        candidates["u_max"].append(pick_edge_point(points_bin, "u_max"))

    return {
        name: np.asarray(points, dtype=np.float64)
        for name, points in candidates.items()
    }


def compute_geometry_result(corners_3d, corner_start: int = 1):
    """计算长度、宽度、平均高度、纵向倾斜角和水平偏移角。"""

    corners = np.asarray(corners_3d, dtype=np.float64)
    if corners.shape != (4, 3):
        raise ValueError(f"corners_3d 必须为 [4,3]，当前为 {corners.shape}")

    p1, p2, p3, p4 = corners

    # U = length
    length_mm = (
        np.linalg.norm(p3 - p1) + np.linalg.norm(p4 - p2)
    ) / 2.0

    # V = width
    width_mm = (
        np.linalg.norm(p2 - p1) + np.linalg.norm(p4 - p3)
    ) / 2.0

    # Umin = 车头端；Umax = 车尾端。
    u_min_center = np.mean((p1, p2), axis=0)
    u_max_center = np.mean((p3, p4), axis=0)
    u_vector = u_max_center - u_min_center

    # 纵向倾斜角：车头 -> 车尾，相对于水平面。
    delta_height = float(u_vector[2])
    horizontal_run = float(np.linalg.norm(u_vector[:2]))
    tilt_angle_deg = (
        0.0
        if horizontal_run < 1e-9
        else float(np.degrees(np.arctan2(delta_height, horizontal_run)))
    )

    # 水平偏移角：Umin -> Umax 相对于理论 world -Y。
    u_xy = np.asarray([u_vector[0], u_vector[1]], dtype=np.float64)
    offset_angle_deg = (
        0.0
        if np.linalg.norm(u_xy) < 1e-9
        else float(np.degrees(np.arctan2(u_xy[0], -u_xy[1])))
    )

    corner_ids = [f"P{corner_start + i}" for i in range(4)]
    corners_rounded = [
        [round(float(value), 3) for value in point]
        for point in corners
    ]

    return {
        "corner_ids": corner_ids,
        "corners_xyz_mm": corners_rounded,
        "length_mm": round(float(length_mm), 3),
        "width_mm": round(float(width_mm), 3),
        "height_mean_mm": round(float(np.mean(corners[:, 2])), 3),
        "tilt_angle_deg": round(float(tilt_angle_deg), 6),
        "offset_angle_deg": round(float(offset_angle_deg), 6),
    }


def compute_first_workface_loading_plan(
    board_length_mm: float,
    pallet_length_mm: float,
    pad_trigger_ratio: float,
):
    """计算两块底板场景下 label_2 第一作业面的剩余空间规划。"""

    board_length_mm = float(board_length_mm)
    pallet_length_mm = float(pallet_length_mm)
    pad_trigger_ratio = float(pad_trigger_ratio)

    if board_length_mm < 0:
        raise ValueError("board_length_mm 不能为负数")
    if pallet_length_mm <= 0:
        raise ValueError("pallet_length_mm 必须大于 0")
    if not 0.0 <= pad_trigger_ratio <= 1.0:
        raise ValueError("pad_trigger_ratio 必须位于 [0, 1]")

    normal_pallet_count = int(np.floor(board_length_mm / pallet_length_mm))
    remaining_length_mm = board_length_mm - normal_pallet_count * pallet_length_mm

    if abs(remaining_length_mm) < 1e-6:
        remaining_length_mm = 0.0

    remaining_ratio = remaining_length_mm / pallet_length_mm

    if remaining_length_mm <= 1e-6:
        pad_required = False
        compensation_mm = 0.0
        final_pallet_count = normal_pallet_count
    elif remaining_ratio >= pad_trigger_ratio:
        pad_required = True
        compensation_mm = pallet_length_mm - remaining_length_mm
        final_pallet_count = normal_pallet_count + 1
    else:
        pad_required = False
        compensation_mm = 0.0
        final_pallet_count = normal_pallet_count

    return {
        "pallet_length_mm": round(pallet_length_mm, 3),
        "normal_pallet_count": normal_pallet_count,
        "remaining_length_mm": round(float(remaining_length_mm), 3),
        "remaining_ratio": round(float(remaining_ratio), 6),
        "pad_trigger_ratio": round(float(pad_trigger_ratio), 6),
        "pad_required": pad_required,
        "remaining_workface_compensation_mm": round(float(compensation_mm), 3),
        "final_pallet_count": final_pallet_count,
    }


def process_one_label_timed(
    points,
    label_id,
    config: PipelineConfig,
    corner_start: int = 1,
):
    rows = []
    label_name = f"label_{label_id}"
    total_start = time.perf_counter()

    step_start = time.perf_counter()
    plane_model, inliers = fit_plane_once(points, config)
    rows.append((label_name, "平面RANSAC拟合", time.perf_counter() - step_start))

    if plane_model is None or inliers is None or len(inliers) < config.edge_min_inliers:
        rows.append((label_name, "该标签总耗时", time.perf_counter() - total_start))
        return None, rows

    plane_points = np.asarray(points)[inliers]

    step_start = time.perf_counter()
    pca_info = compute_pca_info(
        plane_points,
        plane_model,
        config.truck_head_dir,
    )
    rows.append((label_name, "PCA局部坐标系", time.perf_counter() - step_start))

    step_start = time.perf_counter()
    u, v, _ = project_points_to_uvn(plane_points, pca_info)
    uv = np.column_stack((u, v))
    rows.append((label_name, "UV投影", time.perf_counter() - step_start))

    step_start = time.perf_counter()
    edge_candidates = extract_edge_candidates_2d(uv, config)
    rows.append((label_name, "分段切片与边缘候选提取", time.perf_counter() - step_start))

    step_start = time.perf_counter()
    lines = {}
    rng = np.random.default_rng(config.seed + int(label_id))
    for edge_name in ("v_min", "v_max", "u_min", "u_max"):
        lines[edge_name] = ransac_fit_line_2d(
            edge_candidates[edge_name],
            config,
            rng,
        )
    rows.append((label_name, "四边RANSAC直线拟合", time.perf_counter() - step_start))

    step_start = time.perf_counter()
    corners_3d = None
    if all(lines[name] is not None for name in ("v_min", "v_max", "u_min", "u_max")):
        intersections = [
            line_intersection(lines["u_min"], lines["v_min"]),
            line_intersection(lines["u_min"], lines["v_max"]),
            line_intersection(lines["u_max"], lines["v_min"]),
            line_intersection(lines["u_max"], lines["v_max"]),
        ]
        if all(point is not None for point in intersections):
            corners_3d = np.asarray(
                [uv_to_3d(point[0], point[1], pca_info) for point in intersections],
                dtype=np.float64,
            )
    rows.append((label_name, "直线求交与3D角点恢复", time.perf_counter() - step_start))

    step_start = time.perf_counter()
    geometry_result = (
        None
        if corners_3d is None
        else compute_geometry_result(corners_3d, corner_start=corner_start)
    )
    rows.append((label_name, "尺寸/高度/倾斜角/偏移角计算", time.perf_counter() - step_start))
    rows.append((label_name, "该标签总耗时", time.perf_counter() - total_start))
    return geometry_result, rows


def process_segmented_points_timed(
    xyz,
    labels,
    config: PipelineConfig,
):
    total_start = time.perf_counter()
    xyz = np.asarray(xyz, dtype=np.float32)
    labels = np.asarray(labels, dtype=np.int32)
    if xyz.shape != (labels.size, 3):
        raise ValueError(f"分割数据形状不一致：xyz={xyz.shape}, labels={labels.shape}")

    result = {}
    rows = []
    detected_board_count = 0

    for label_id in config.target_labels:
        label_name = f"label_{label_id}"
        label_points = xyz[labels == label_id]
        if len(label_points) == 0:
            result[label_name] = None
            continue

        corner_start = detected_board_count * 4 + 1
        label_result, label_rows = process_one_label_timed(
            label_points,
            label_id,
            config,
            corner_start=corner_start,
        )
        result[label_name] = label_result
        rows.extend(label_rows)

        if label_result is not None:
            detected_board_count += 1

    # 只有两块底板都成功时，才对 label_2 计算第一作业面装载规划。
    if result.get("label_2") is not None and result.get("label_3") is not None:
        result["label_2"]["loading_plan"] = compute_first_workface_loading_plan(
            result["label_2"]["length_mm"],
            pallet_length_mm=config.pallet_length_mm,
            pad_trigger_ratio=config.pad_trigger_ratio,
        )

    return result, rows, time.perf_counter() - total_start


class PointCloudPipeline:
    """持有一次加载的 PointNet++ 模型，并处理单个或多个 PCD。"""

    def __init__(
        self,
        config: PipelineConfig | None = None,
        checkpoint_path: str | None = None,
        device: str | None = None,
        segmenter_factory=None,
    ) -> None:
        self.config = config or PipelineConfig()
        factory = segmenter_factory or PointNet2Segmenter
        if factory is None:
            raise RuntimeError(
                f"无法导入 point_cloud_segment_module：{POINTNET_IMPORT_ERROR}"
            )
        self.segmenter = factory(
            checkpoint_path=checkpoint_path,
            device=device,
            num_point=self.config.pointnet_num_point,
            num_votes=self.config.pointnet_num_votes,
            seed=self.config.seed,
        )

    def segment_xyz_full_coverage(self, xyz):
        """全部过滤后点一次送入 PointNet++；无分批、无重叠、无 KNN。"""

        dense_xyz = np.asarray(xyz, dtype=np.float32)
        if dense_xyz.ndim != 2 or dense_xyz.shape[1] != 3 or len(dense_xyz) == 0:
            raise ValueError(f"整云推理输入必须为非空 [N,3]，当前为 {dense_xyz.shape}")

        dense_count = int(len(dense_xyz))

        prepare_start = time.perf_counter()
        normalized_xyz = pc_normalize(dense_xyz.copy())
        points = (
            torch.from_numpy(normalized_xyz)
            .float()
            .unsqueeze(0)
            .to(self.segmenter.device)
            .transpose(2, 1)
        )
        cls_label = torch.zeros((1, 1), dtype=torch.long, device=self.segmenter.device)
        cls_one_hot = torch.eye(
            DEFAULT_NUM_CLASSES,
            device=self.segmenter.device,
        )[cls_label.cpu().numpy()]
        prepare_s = time.perf_counter() - prepare_start

        inference_start = time.perf_counter()
        vote_count = max(int(self.segmenter.num_votes), 1)
        vote_pool = torch.zeros(
            (1, dense_count, DEFAULT_NUM_PART),
            dtype=torch.float32,
            device=self.segmenter.device,
        )

        try:
            with torch.inference_mode():
                for _ in range(vote_count):
                    seg_pred, _ = self.segmenter.model(points, cls_one_hot)
                    if tuple(seg_pred.shape) != tuple(vote_pool.shape):
                        raise ValueError(
                            "PointNet++ 整云输出形状与输入点数不一致："
                            f"expected={tuple(vote_pool.shape)}, actual={tuple(seg_pred.shape)}"
                        )
                    vote_pool += seg_pred
        except torch.cuda.OutOfMemoryError as exc:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            raise RuntimeError(
                f"整云一次性推理显存不足：输入 {dense_count} 点。"
                "当前版本不自动回退到分批模式。"
            ) from exc

        averaged_scores = vote_pool / vote_count
        inference_s = time.perf_counter() - inference_start

        restore_start = time.perf_counter()
        dense_labels = (
            torch.argmax(averaged_scores, dim=2)[0]
            .cpu()
            .numpy()
            .astype(np.int32)
        )
        restore_s = time.perf_counter() - restore_start

        label_counts = {
            label: int(np.count_nonzero(dense_labels == label))
            for label in range(DEFAULT_NUM_PART)
        }

        print(
            f"    PointNet++ 整云推理: {inference_s:.3f} s | "
            f"输入 {dense_count} 点 | 无分批 | 无重叠 | 无KNN"
        )

        return dense_labels, label_counts, {
            "dense_count": dense_count,
            "prepare_s": round(float(prepare_s), 6),
            "inference_s": round(float(inference_s), 6),
            "restore_s": round(float(restore_s), 6),
        }

    def process_file(self, input_path: str | Path, output_root: str | Path):
        input_path = Path(input_path)
        output_root = Path(output_root)
        file_start = time.perf_counter()

        point_cloud = read_point_cloud(input_path)
        filtered, stage_counts, preprocess_timings = preprocess_point_cloud_timed(
            point_cloud,
            self.config,
        )

        dense_xyz = np.asarray(filtered.points, dtype=np.float32)
        dense_labels, dense_label_counts, pointnet_stats = self.segment_xyz_full_coverage(
            dense_xyz
        )

        result, geometry_steps, geometry_s = process_segmented_points_timed(
            dense_xyz,
            dense_labels,
            self.config,
        )

        json_path = output_root / "json" / f"{input_path.stem}.json"
        write_json(json_path, result)

        total_s = time.perf_counter() - file_start
        print(
            f"[完成] {input_path.name}: 总 {total_s:.3f} s | "
            f"PointNet++ {pointnet_stats['inference_s']:.3f} s | "
            f"几何后处理 {geometry_s:.3f} s"
        )

        return {
            "input": str(input_path),
            "status": "success",
            "json": str(json_path),
            "stage_counts": stage_counts,
            "dense_label_counts": dense_label_counts,
            "preprocess_timing": {
                key: round(float(value), 6)
                for key, value in preprocess_timings.items()
            },
            "pointnet_timing": pointnet_stats,
            "geometry_steps": [
                {
                    "label": label,
                    "step": step,
                    "seconds": round(float(seconds), 6),
                }
                for label, step, seconds in geometry_steps
            ],
            "geometry_s": round(float(geometry_s), 6),
            "total_s": round(float(total_s), 6),
            "result": result,
        }

    def process_path(self, input_path: str | Path, output_root: str | Path):
        files = collect_pcd_files(input_path)
        resolved_output = Path(output_root).expanduser().resolve()
        resolved_output.mkdir(parents=True, exist_ok=True)

        records = []
        batch_start = time.perf_counter()

        for index, path in enumerate(files, start=1):
            print("=" * 78)
            print(f"[{index}/{len(files)}] 开始处理：{path.name}")
            one_start = time.perf_counter()
            try:
                records.append(self.process_file(path, resolved_output))
            except Exception as exc:
                elapsed = time.perf_counter() - one_start
                print(
                    f"[失败] {path.name}: {elapsed:.3f} s | "
                    f"{type(exc).__name__}: {exc}"
                )
                records.append(
                    {
                        "input": str(path),
                        "status": "failed",
                        "error": f"{type(exc).__name__}: {exc}",
                        "total_s": round(float(elapsed), 6),
                    }
                )

        batch_total_s = time.perf_counter() - batch_start
        summary = {
            "input": str(Path(input_path).expanduser().resolve()),
            "output": str(resolved_output),
            "total": len(records),
            "succeeded": sum(row["status"] == "success" for row in records),
            "failed": sum(row["status"] == "failed" for row in records),
            "batch_total_s": round(float(batch_total_s), 6),
            "files": records,
        }

        print("=" * 78)
        print(
            f"处理完成：成功 {summary['succeeded']} / {summary['total']}，"
            f"总耗时 {batch_total_s:.3f} s"
        )
        print(f"JSON目录：{resolved_output / 'json'}")
        return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="PCD -> PointNet++ -> 底板角点/姿态/装载规划 -> JSON"
    )
    parser.add_argument("input", help="单个 PCD 文件或包含 PCD 的目录")
    parser.add_argument(
        "-o",
        "--output",
        default="point_cloud_output",
        help="输出根目录；最终只写 output/json/*.json",
    )
    parser.add_argument("--device", choices=("cpu", "cuda"), default=None)
    parser.add_argument("--checkpoint", default=None, help="模型权重路径")

    # 保留常用 CLI 覆盖项；其余参数直接在 PipelineConfig 集中修改。
    parser.add_argument(
        "--num-point",
        type=int,
        default=DEFAULT_CONFIG.pointnet_num_point,
    )
    parser.add_argument(
        "--num-votes",
        type=int,
        default=DEFAULT_CONFIG.pointnet_num_votes,
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=DEFAULT_CONFIG.voxel_size,
    )
    parser.add_argument(
        "--plane-threshold",
        type=float,
        default=DEFAULT_CONFIG.plane_threshold,
    )
    parser.add_argument(
        "--dbscan-eps",
        type=float,
        default=DEFAULT_CONFIG.dbscan_eps,
    )
    parser.add_argument(
        "--dbscan-min-points",
        type=int,
        default=DEFAULT_CONFIG.dbscan_min_points,
    )
    return parser


def main(argv=None) -> int:
    args = build_argument_parser().parse_args(argv)

    config = PipelineConfig(
        pointnet_num_point=args.num_point,
        pointnet_num_votes=args.num_votes,
        voxel_size=args.voxel_size,
        plane_threshold=args.plane_threshold,
        dbscan_eps=args.dbscan_eps,
        dbscan_min_points=args.dbscan_min_points,
    )

    pipeline = PointCloudPipeline(
        config=config,
        checkpoint_path=args.checkpoint,
        device=args.device,
    )
    summary = pipeline.process_path(args.input, args.output)
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
