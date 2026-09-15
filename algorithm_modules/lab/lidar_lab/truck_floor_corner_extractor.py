# -*- coding: utf-8 -*-
"""Truck floor point cloud corner extraction.

Call ``extract_truck_floor_corners(points)`` with an ``N x 3`` point array in
LiDAR coordinates. The function returns the four board corners and geometry
without reading or writing any files.
"""

from dataclasses import dataclass, field

import numpy as np
import open3d as o3d


@dataclass
class TruckFloorCornerConfig:
    roi_x: tuple[float, float] = (297.0, 1497.0)
    roi_y: tuple[float, float] = (-1103.0, 596.0)
    roi_z: tuple[float, float] = (-147.0, 953.0)

    ground_fit_band: float = 35.0
    ground_ransac_dist: float = 35.0
    ground_delete_dist: float = 35.0

    board_ransac_dist: float = 6.0
    board_keep_dist: float = 25.0

    dbscan_eps: float = 35.0
    dbscan_min_points: int = 8

    sor_nb_neighbors: int = 16
    sor_std_ratio: float = 3.0
    sor_max_delete_ratio: float = 0.05

    truck_head_dir: np.ndarray = field(
        default_factory=lambda: np.array([-1.0, 1.0, 0.0], dtype=np.float64)
    )

    slice_width_u: float = 50.0
    slice_width_v: float = 30.0
    edge_u_bins_min: int = 80
    edge_u_bins_max: int = 350
    edge_v_bins_min: int = 50
    edge_v_bins_max: int = 220
    slice_min_points: int = 20
    edge_pick_mode: str = "extreme"
    edge_percentile: float = 2.0

    line_ransac_dist: float = 8.0
    line_ransac_iter: int = 800
    line_min_inliers: int = 20

    ransac_iter: int = 3000
    random_seed: int = 42


DEFAULT_CONFIG = TruckFloorCornerConfig()


def extract_truck_floor_corners(
    points: np.ndarray,
    config: TruckFloorCornerConfig | None = None,
    return_debug: bool = False,
) -> dict:
    """Extract four truck floor corners from raw LiDAR points.

    Corner order:
        P1 = small U, small V
        P2 = small U, large V
        P3 = large U, small V
        P4 = large U, large V
    """
    cfg = config or DEFAULT_CONFIG
    points = _validate_points(points, "points")

    roi_points = crop_roi(points, cfg)
    after_ground, ground_plane = remove_ground(roi_points, cfg)
    board_points, board_plane, board_band = fit_board(after_ground, cfg)
    filtered_board = light_statistical_filter(board_points, cfg)
    corner_result = extract_board_corners(filtered_board, board_plane, cfg)

    corners = corner_result["corners_3d"]
    geometry = compute_corner_geometry(corners)

    result = {
        "coordinate_unit": "mm",
        "corner_order": {
            "P1": "U_small_V_small",
            "P2": "U_small_V_large",
            "P3": "U_large_V_small",
            "P4": "U_large_V_large",
        },
        "corners_xyz_mm": {
            f"P{index + 1}": corners[index].copy()
            for index in range(4)
        },
        "corners_array_xyz_mm": corners.copy(),
        "center_xyz_mm": geometry["center_xyz_mm"].copy(),
        "length_mm": geometry["length_mm"],
        "width_mm": geometry["width_mm"],
        "ground_plane_ax_by_cz_d": ground_plane.copy(),
        "board_plane_ax_by_cz_d": board_plane.copy(),
        "slice_info": corner_result["slice_info"].copy(),
        "line_inlier_counts": corner_result["line_inlier_counts"].copy(),
    }

    if return_debug:
        result["debug"] = {
            "roi_points": roi_points,
            "after_ground_points": after_ground,
            "board_band_points": board_band,
            "board_points_before_filter": board_points,
            "board_points": filtered_board,
            "corners_uv": corner_result["corners_uv"],
            "uv": corner_result["uv"],
            "edge_candidates": corner_result["edge_candidates"],
            "lines": corner_result["lines"],
            "pca_info": corner_result["pca_info"],
        }

    return result


def crop_roi(points: np.ndarray, cfg: TruckFloorCornerConfig = DEFAULT_CONFIG) -> np.ndarray:
    points = _validate_points(points, "points")
    roi_min = np.array([cfg.roi_x[0], cfg.roi_y[0], cfg.roi_z[0]], dtype=np.float64)
    roi_max = np.array([cfg.roi_x[1], cfg.roi_y[1], cfg.roi_z[1]], dtype=np.float64)
    mask = np.all((points >= roi_min) & (points <= roi_max), axis=1)
    roi_points = points[mask]

    if len(roi_points) < 3:
        raise RuntimeError("ROI result has fewer than 3 points.")

    return roi_points


def remove_ground(
    points: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> tuple[np.ndarray, np.ndarray]:
    points = _validate_points(points, "points")

    x_ground_ref = np.quantile(points[:, 0], 0.99)
    ground_candidate = points[points[:, 0] >= x_ground_ref - cfg.ground_fit_band]

    if len(ground_candidate) < 3:
        raise RuntimeError("Ground candidate has fewer than 3 points.")

    ground_plane, _ = make_pcd(ground_candidate).segment_plane(
        distance_threshold=cfg.ground_ransac_dist,
        ransac_n=3,
        num_iterations=cfg.ransac_iter,
    )
    ground_plane = normalize_plane(ground_plane)
    distances = point_to_plane_distance(points, ground_plane)
    after_ground = points[distances > cfg.ground_delete_dist]

    if len(after_ground) < 3:
        raise RuntimeError("Ground filtering left fewer than 3 points.")

    return after_ground, ground_plane


def fit_board(
    points: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points = _validate_points(points, "points")

    board_plane, _ = make_pcd(points).segment_plane(
        distance_threshold=cfg.board_ransac_dist,
        ransac_n=3,
        num_iterations=cfg.ransac_iter,
    )
    board_plane = normalize_plane(board_plane)
    distances = point_to_plane_distance(points, board_plane)
    board_band = points[distances <= cfg.board_keep_dist]

    if len(board_band) < 3:
        raise RuntimeError("Board plane band has fewer than 3 points.")

    board_points = keep_largest_cluster(board_band, cfg)
    return board_points, board_plane, board_band


def keep_largest_cluster(
    points: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> np.ndarray:
    points = _validate_points(points, "points")

    if len(points) < cfg.dbscan_min_points:
        raise RuntimeError("Too few points for DBSCAN clustering.")

    labels = np.asarray(
        make_pcd(points).cluster_dbscan(
            eps=cfg.dbscan_eps,
            min_points=cfg.dbscan_min_points,
            print_progress=False,
        ),
        dtype=np.int32,
    )
    valid_mask = labels >= 0

    if not np.any(valid_mask):
        raise RuntimeError("DBSCAN found no valid board cluster.")

    unique_labels, counts = np.unique(labels[valid_mask], return_counts=True)
    largest_label = unique_labels[np.argmax(counts)]
    board_points = points[labels == largest_label]

    if len(board_points) < 3:
        raise RuntimeError("Largest board cluster has fewer than 3 points.")

    return board_points


def light_statistical_filter(
    points: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> np.ndarray:
    points = _validate_points(points, "points")

    if len(points) <= cfg.sor_nb_neighbors:
        return points.copy()

    filtered_pcd, _ = make_pcd(points).remove_statistical_outlier(
        nb_neighbors=cfg.sor_nb_neighbors,
        std_ratio=cfg.sor_std_ratio,
    )
    filtered_points = np.asarray(filtered_pcd.points, dtype=np.float64)
    deleted_ratio = (len(points) - len(filtered_points)) / len(points)

    if len(filtered_points) < 3 or deleted_ratio > cfg.sor_max_delete_ratio:
        return points.copy()

    return filtered_points


def extract_board_corners(
    points: np.ndarray,
    board_plane: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> dict:
    points = _validate_points(points, "points")

    if len(points) < 100:
        raise RuntimeError("Too few board points for stable corner extraction.")

    pca_info = compute_pca_info(points, board_plane, cfg)
    u, v, _ = project_points_to_uvn(points, pca_info)
    uv = np.column_stack([u, v])

    edge_candidates, slice_info = extract_edge_candidates_2d(uv, cfg)

    lines = {}
    line_inlier_counts = {}
    for edge_name in ("bottom", "top", "left", "right"):
        line, inliers = ransac_fit_line_2d(edge_candidates[edge_name], cfg)
        if line is None or inliers is None:
            raise RuntimeError(f"{edge_name} edge line fitting failed.")
        lines[edge_name] = line
        line_inlier_counts[edge_name] = int(len(inliers))

    corners_uv = np.asarray(
        [
            line_intersection(lines["left"], lines["bottom"]),
            line_intersection(lines["left"], lines["top"]),
            line_intersection(lines["right"], lines["bottom"]),
            line_intersection(lines["right"], lines["top"]),
        ],
        dtype=np.float64,
    )

    if np.any(~np.isfinite(corners_uv)):
        raise RuntimeError("Corner line intersection failed.")

    corners_3d = np.asarray(
        [uv_to_3d(corner[0], corner[1], pca_info) for corner in corners_uv],
        dtype=np.float64,
    )

    return {
        "corners_3d": corners_3d,
        "corners_uv": corners_uv,
        "uv": uv,
        "edge_candidates": edge_candidates,
        "lines": lines,
        "line_inlier_counts": line_inlier_counts,
        "slice_info": slice_info,
        "pca_info": pca_info,
    }


def compute_corner_geometry(corners_3d: np.ndarray) -> dict:
    corners_3d = _validate_points(corners_3d, "corners_3d")

    if len(corners_3d) != 4:
        raise RuntimeError("corners_3d must contain exactly 4 points.")

    p1, p2, p3, p4 = corners_3d
    length_mm = 0.5 * (np.linalg.norm(p3 - p1) + np.linalg.norm(p4 - p2))
    width_mm = 0.5 * (np.linalg.norm(p2 - p1) + np.linalg.norm(p4 - p3))

    return {
        "length_mm": float(length_mm),
        "width_mm": float(width_mm),
        "center_xyz_mm": np.mean(corners_3d, axis=0),
    }


def compute_pca_info(
    points: np.ndarray,
    plane_model: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> dict:
    points = _validate_points(points, "points")
    plane_model = normalize_plane(plane_model)
    normal_dir = plane_model[:3]

    raw_center = np.mean(points, axis=0)
    signed_distance = np.dot(normal_dir, raw_center) + plane_model[3]
    center = raw_center - signed_distance * normal_dir

    centered = points - center
    covariance = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    eigenvectors = eigenvectors[:, np.argsort(eigenvalues)[::-1]]

    long_dir = eigenvectors[:, 0]
    long_dir = long_dir - np.dot(long_dir, normal_dir) * normal_dir
    long_dir = normalize_vector(long_dir)

    short_dir = np.cross(normal_dir, long_dir)
    short_dir = normalize_vector(short_dir)

    head_dir = np.asarray(cfg.truck_head_dir, dtype=np.float64)
    head_dir = head_dir - np.dot(head_dir, normal_dir) * normal_dir
    head_dir = normalize_vector(head_dir)

    # Keep U numbering stable: the small-U side points roughly to the truck head.
    if np.dot(-long_dir, head_dir) < 0:
        long_dir = -long_dir
        short_dir = -short_dir

    return {
        "center": center,
        "long_dir": long_dir,
        "short_dir": short_dir,
        "normal_dir": normal_dir,
    }


def project_points_to_uvn(points: np.ndarray, pca_info: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points = _validate_points(points, "points")
    centered = points - pca_info["center"]
    u = centered @ pca_info["long_dir"]
    v = centered @ pca_info["short_dir"]
    n = centered @ pca_info["normal_dir"]
    return u, v, n


def uv_to_3d(u: float, v: float, pca_info: dict) -> np.ndarray:
    return pca_info["center"] + float(u) * pca_info["long_dir"] + float(v) * pca_info["short_dir"]


def extract_edge_candidates_2d(
    uv: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> tuple[dict[str, np.ndarray], dict]:
    uv = _validate_uv(uv)
    u = uv[:, 0]
    v = uv[:, 1]
    edge_u_bins, edge_v_bins, slice_info = compute_dynamic_bins(uv, cfg)

    bottom_points = []
    top_points = []
    left_points = []
    right_points = []

    u_edges = np.linspace(np.min(u), np.max(u), edge_u_bins + 1)
    for index in range(edge_u_bins):
        low, high = u_edges[index], u_edges[index + 1]
        upper_mask = u <= high if index == edge_u_bins - 1 else u < high
        mask = (u >= low) & upper_mask
        bin_points = uv[mask]
        if len(bin_points) < cfg.slice_min_points:
            continue
        bottom_points.append(pick_edge_point(bin_points, "bottom", cfg))
        top_points.append(pick_edge_point(bin_points, "top", cfg))

    v_edges = np.linspace(np.min(v), np.max(v), edge_v_bins + 1)
    for index in range(edge_v_bins):
        low, high = v_edges[index], v_edges[index + 1]
        upper_mask = v <= high if index == edge_v_bins - 1 else v < high
        mask = (v >= low) & upper_mask
        bin_points = uv[mask]
        if len(bin_points) < cfg.slice_min_points:
            continue
        left_points.append(pick_edge_point(bin_points, "left", cfg))
        right_points.append(pick_edge_point(bin_points, "right", cfg))

    edge_candidates = {
        "bottom": np.asarray(bottom_points, dtype=np.float64),
        "top": np.asarray(top_points, dtype=np.float64),
        "left": np.asarray(left_points, dtype=np.float64),
        "right": np.asarray(right_points, dtype=np.float64),
    }

    for edge_name, edge_points in edge_candidates.items():
        if len(edge_points) < cfg.line_min_inliers:
            raise RuntimeError(
                f"{edge_name} edge candidates are insufficient: "
                f"{len(edge_points)} < {cfg.line_min_inliers}."
            )

    return edge_candidates, slice_info


def compute_dynamic_bins(
    uv: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> tuple[int, int, dict]:
    uv = _validate_uv(uv)
    u_range = float(np.max(uv[:, 0]) - np.min(uv[:, 0]))
    v_range = float(np.max(uv[:, 1]) - np.min(uv[:, 1]))

    if u_range < 1e-9 or v_range < 1e-9:
        raise RuntimeError("UV range is too small for slicing.")

    u_bins = int(np.ceil(u_range / cfg.slice_width_u))
    v_bins = int(np.ceil(v_range / cfg.slice_width_v))
    u_bins = int(np.clip(u_bins, cfg.edge_u_bins_min, cfg.edge_u_bins_max))
    v_bins = int(np.clip(v_bins, cfg.edge_v_bins_min, cfg.edge_v_bins_max))

    return u_bins, v_bins, {
        "u_range": u_range,
        "v_range": v_range,
        "u_bins": u_bins,
        "v_bins": v_bins,
        "actual_u_slice_width": u_range / u_bins,
        "actual_v_slice_width": v_range / v_bins,
    }


def pick_edge_point(
    points_bin: np.ndarray,
    edge_type: str,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> np.ndarray:
    if cfg.edge_pick_mode == "percentile":
        if edge_type == "bottom":
            target = np.percentile(points_bin[:, 1], cfg.edge_percentile)
            return points_bin[np.argmin(np.abs(points_bin[:, 1] - target))]
        if edge_type == "top":
            target = np.percentile(points_bin[:, 1], 100.0 - cfg.edge_percentile)
            return points_bin[np.argmin(np.abs(points_bin[:, 1] - target))]
        if edge_type == "left":
            target = np.percentile(points_bin[:, 0], cfg.edge_percentile)
            return points_bin[np.argmin(np.abs(points_bin[:, 0] - target))]
        if edge_type == "right":
            target = np.percentile(points_bin[:, 0], 100.0 - cfg.edge_percentile)
            return points_bin[np.argmin(np.abs(points_bin[:, 0] - target))]
        raise ValueError(f"Unknown edge type: {edge_type}")

    if edge_type == "bottom":
        return points_bin[np.argmin(points_bin[:, 1])]
    if edge_type == "top":
        return points_bin[np.argmax(points_bin[:, 1])]
    if edge_type == "left":
        return points_bin[np.argmin(points_bin[:, 0])]
    if edge_type == "right":
        return points_bin[np.argmax(points_bin[:, 0])]

    raise ValueError(f"Unknown edge type: {edge_type}")


def ransac_fit_line_2d(
    points_2d: np.ndarray,
    cfg: TruckFloorCornerConfig = DEFAULT_CONFIG,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    points_2d = _validate_uv(points_2d)

    if len(points_2d) < max(2, cfg.line_min_inliers):
        return None, None

    rng = np.random.default_rng(cfg.random_seed)
    best_inliers = None
    best_count = 0

    for _ in range(cfg.line_ransac_iter):
        index1, index2 = rng.choice(len(points_2d), size=2, replace=False)
        line = line_from_two_points(points_2d[index1], points_2d[index2])
        if line is None:
            continue

        inliers = np.where(line_distances(points_2d, line) <= cfg.line_ransac_dist)[0]
        if len(inliers) > best_count:
            best_count = len(inliers)
            best_inliers = inliers

    if best_inliers is None or len(best_inliers) < cfg.line_min_inliers:
        return None, None

    refined_line = refit_line_svd(points_2d[best_inliers])
    if refined_line is None:
        return None, None

    refined_inliers = np.where(line_distances(points_2d, refined_line) <= cfg.line_ransac_dist)[0]
    if len(refined_inliers) < cfg.line_min_inliers:
        return None, None

    return refit_line_svd(points_2d[refined_inliers]), refined_inliers


def line_from_two_points(point1: np.ndarray, point2: np.ndarray) -> np.ndarray | None:
    point1 = np.asarray(point1, dtype=np.float64)
    point2 = np.asarray(point2, dtype=np.float64)
    direction = point2 - point1

    if np.linalg.norm(direction) < 1e-12:
        return None

    a = direction[1]
    b = -direction[0]
    norm = np.hypot(a, b)

    if norm < 1e-12:
        return None

    a /= norm
    b /= norm
    c = -(a * point1[0] + b * point1[1])
    return np.array([a, b, c], dtype=np.float64)


def line_distances(points_2d: np.ndarray, line: np.ndarray) -> np.ndarray:
    a, b, c = line
    return np.abs(a * points_2d[:, 0] + b * points_2d[:, 1] + c)


def refit_line_svd(points_2d: np.ndarray) -> np.ndarray | None:
    points_2d = _validate_uv(points_2d)

    if len(points_2d) < 2:
        return None

    center = np.mean(points_2d, axis=0)
    _, _, vt = np.linalg.svd(points_2d - center, full_matrices=False)
    direction = normalize_vector(vt[0])
    normal = normalize_vector(np.array([direction[1], -direction[0]], dtype=np.float64))
    a, b = normal
    c = -(a * center[0] + b * center[1])
    return np.array([a, b, c], dtype=np.float64)


def line_intersection(line1: np.ndarray, line2: np.ndarray) -> np.ndarray:
    matrix = np.array([[line1[0], line1[1]], [line2[0], line2[1]]], dtype=np.float64)
    right_side = np.array([-line1[2], -line2[2]], dtype=np.float64)

    if abs(np.linalg.det(matrix)) < 1e-9:
        return np.array([np.nan, np.nan], dtype=np.float64)

    return np.linalg.solve(matrix, right_side)


def make_pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    points = _validate_points(points, "points")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    norm = np.linalg.norm(vector)

    if norm < 1e-12:
        raise RuntimeError("Cannot normalize a near-zero vector.")

    return vector / norm


def normalize_plane(plane: np.ndarray) -> np.ndarray:
    plane = np.asarray(plane, dtype=np.float64)
    normal_norm = np.linalg.norm(plane[:3])

    if normal_norm < 1e-12:
        raise RuntimeError("Invalid plane normal.")

    plane = plane / normal_norm
    dominant_axis = int(np.argmax(np.abs(plane[:3])))

    if plane[dominant_axis] < 0:
        plane = -plane

    return plane


def point_to_plane_distance(points: np.ndarray, plane: np.ndarray) -> np.ndarray:
    points = _validate_points(points, "points")
    plane = normalize_plane(plane)
    return np.abs(points @ plane[:3] + plane[3])


def _validate_points(points: np.ndarray, name: str) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{name} must be an N x 3 array.")

    if len(points) == 0:
        raise ValueError(f"{name} is empty.")

    if not np.all(np.isfinite(points)):
        raise ValueError(f"{name} contains NaN or Inf.")

    return points


def _validate_uv(points_2d: np.ndarray) -> np.ndarray:
    points_2d = np.asarray(points_2d, dtype=np.float64)

    if points_2d.ndim != 2 or points_2d.shape[1] != 2:
        raise ValueError("points_2d must be an N x 2 array.")

    if len(points_2d) == 0:
        raise ValueError("points_2d is empty.")

    if not np.all(np.isfinite(points_2d)):
        raise ValueError("points_2d contains NaN or Inf.")

    return points_2d


__all__ = [
    "TruckFloorCornerConfig",
    "extract_truck_floor_corners",
    "crop_roi",
    "remove_ground",
    "fit_board",
    "light_statistical_filter",
    "extract_board_corners",
    "compute_corner_geometry",
]
