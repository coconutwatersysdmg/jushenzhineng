"""
camera_geometry_module_hybrid.py

相机 + 雷达混合角点最终修正模块

最终角点来源
------------
P1, P2, P3, P4, P7, P8:
    由相机识别得到像素坐标 (u, v) + depth
    -> 相机内参反投影到相机坐标
    -> T_world_camera 外参转换到世界坐标

P5, P6:
    不再由相机重新拍摄。
    直接保留雷达阶段得到的世界坐标，并根据第一作业面辅助块
    占用长度进行沿第二块板纵向方向的必要修正。

最终输出仍然得到 P1~P8 共 8 个世界坐标角点。

修正规则
--------
1. 用 P1~P4（相机最终点）计算第一块板几何和装载规划；
2. loading_plan 给出 remaining_workface_compensation_mm；
3. 计算 P3/P4 中点 M34 与雷达 P5/P6 中点 M56 的当前纵向间距；
4. 若 compensation_mm > 当前间距：
       shift = compensation_mm - 当前间距
   否则：
       shift = 0
5. P5、P6 同时沿第二块板 P5/P6 -> P7/P8 的纵向方向向后移动 shift；
6. 用修正后的 P5/P6 + 相机 P7/P8 重新计算第二块板几何；
7. 第二块板宽度方向对半，长度方向每 1200 mm 一排进行区域划分。

坐标约定
--------
列向量：
    P_world = T_world_camera @ P_camera

所有输入雷达角点必须已经位于 world 坐标系。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class CameraGeometryConfig:
    pallet_length_mm: float = 1200.0
    pad_trigger_ratio: float = 0.50
    reference_forward_xy: tuple[float, float] = (0.0, 1.0)


DEFAULT_CONFIG = CameraGeometryConfig()


def load_json(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def get_camera_intrinsic(intrinsic_config: dict, camera_name: str) -> dict:
    cameras = intrinsic_config.get("cameras", {})
    if camera_name not in cameras:
        raise KeyError(f"camera_intrinsic.json 中不存在相机：{camera_name}")

    camera = cameras[camera_name]
    K = np.asarray(camera["K"], dtype=np.float64)
    if K.shape != (3, 3):
        raise ValueError(f"{camera_name} 的 K 必须为 3x3")
    return camera


def get_world_extrinsic(coordinate_config: dict, camera_name: str) -> np.ndarray:
    transforms = coordinate_config.get("transforms", {})
    if camera_name not in transforms:
        raise KeyError(f"coordinate_config.json 中不存在坐标系：{camera_name}")

    T = np.asarray(
        transforms[camera_name]["T_world_sensor"],
        dtype=np.float64,
    )
    if T.shape != (4, 4):
        raise ValueError(f"{camera_name} 的 T_world_sensor 必须为 4x4")
    return T


def depth_to_mm(
    depth_value: float,
    camera_intrinsic: dict,
    depth_mode: str = "mm",
) -> float:
    value = float(depth_value)

    if depth_mode == "mm":
        return value
    if depth_mode == "m":
        return value * 1000.0
    if depth_mode == "raw":
        depth_scale = float(camera_intrinsic.get("depth_scale", 0.001))
        return value * depth_scale * 1000.0

    raise ValueError("depth_mode 仅支持 mm / m / raw")


def pixel_depth_to_camera_xyz(
    u: float,
    v: float,
    depth_value: float,
    camera_intrinsic: dict,
    depth_mode: str = "mm",
) -> np.ndarray:
    """像素坐标 + 深度 -> 相机自身 XYZ(mm)。"""

    K = np.asarray(camera_intrinsic["K"], dtype=np.float64)

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])

    if abs(fx) < 1e-12 or abs(fy) < 1e-12:
        raise ValueError("相机内参 fx/fy 无效")

    z = depth_to_mm(
        depth_value,
        camera_intrinsic,
        depth_mode,
    )
    if z <= 0:
        raise ValueError("角点深度必须 > 0")

    x = (float(u) - cx) * z / fx
    y = (float(v) - cy) * z / fy

    return np.asarray([x, y, z], dtype=np.float64)


def camera_xyz_to_world(
    point_camera_mm,
    T_world_camera,
) -> np.ndarray:
    """相机 XYZ(mm) -> 世界 XYZ(mm)。"""

    point = np.asarray(point_camera_mm, dtype=np.float64)
    T = np.asarray(T_world_camera, dtype=np.float64)

    if point.shape != (3,):
        raise ValueError("point_camera_mm 必须为 [3]")
    if T.shape != (4, 4):
        raise ValueError("T_world_camera 必须为 4x4")

    point_h = np.asarray(
        [point[0], point[1], point[2], 1.0],
        dtype=np.float64,
    )
    world_h = T @ point_h

    if abs(world_h[3]) < 1e-12:
        raise ValueError("齐次坐标转换结果无效")

    return world_h[:3] / world_h[3]


def camera_corner_to_world(
    corner: dict,
    camera_intrinsic: dict,
    T_world_camera,
    depth_mode: str = "mm",
) -> np.ndarray:
    camera_xyz = pixel_depth_to_camera_xyz(
        corner["u"],
        corner["v"],
        corner["depth"],
        camera_intrinsic,
        depth_mode,
    )
    return camera_xyz_to_world(
        camera_xyz,
        T_world_camera,
    )


def compute_board_geometry(
    corners_world_mm,
    corner_ids,
    reference_forward_xy=(0.0, 1.0),
) -> dict:
    """使用 4 个世界坐标角点计算几何与姿态。

    顺序：
        [Umin,Vmin], [Umin,Vmax], [Umax,Vmin], [Umax,Vmax]
    """

    corners = np.asarray(corners_world_mm, dtype=np.float64)
    if corners.shape != (4, 3):
        raise ValueError(f"角点必须为 [4,3]，当前为 {corners.shape}")

    p1, p2, p3, p4 = corners

    length_mm = (
        np.linalg.norm(p3 - p1)
        + np.linalg.norm(p4 - p2)
    ) / 2.0

    width_mm = (
        np.linalg.norm(p2 - p1)
        + np.linalg.norm(p4 - p3)
    ) / 2.0

    front_center = (p1 + p2) / 2.0
    rear_center = (p3 + p4) / 2.0
    u_vector = rear_center - front_center

    horizontal_run = float(np.linalg.norm(u_vector[:2]))
    delta_height = float(u_vector[2])

    if horizontal_run < 1e-9:
        slope_percent = 0.0
        tilt_angle_deg = 0.0
    else:
        slope_percent = delta_height / horizontal_run * 100.0
        tilt_angle_deg = float(
            np.degrees(np.arctan2(delta_height, horizontal_run))
        )

    reference = np.asarray(reference_forward_xy, dtype=np.float64)
    if np.linalg.norm(reference) < 1e-12:
        raise ValueError("reference_forward_xy 不能为零向量")

    u_xy = np.asarray(u_vector[:2], dtype=np.float64)
    if np.linalg.norm(u_xy) < 1e-12:
        offset_angle_deg = 0.0
    else:
        reference = reference / np.linalg.norm(reference)
        u_xy = u_xy / np.linalg.norm(u_xy)

        cross_z = reference[0] * u_xy[1] - reference[1] * u_xy[0]
        dot = float(np.clip(np.dot(reference, u_xy), -1.0, 1.0))

        offset_angle_deg = float(
            np.degrees(np.arctan2(cross_z, dot))
        )

    return {
        "corner_ids": list(corner_ids),
        "corners_world_xyz_mm": [
            [round(float(v), 3) for v in point]
            for point in corners
        ],
        "length_mm": round(float(length_mm), 3),
        "width_mm": round(float(width_mm), 3),
        "height_mean_mm": round(float(np.mean(corners[:, 2])), 3),
        "slope_percent": round(float(slope_percent), 6),
        "tilt_angle_deg": round(float(tilt_angle_deg), 6),
        "offset_angle_deg": round(float(offset_angle_deg), 6),
    }


def compute_first_workface_loading_plan(
    board_length_mm: float,
    pallet_length_mm: float = 1200.0,
    pad_trigger_ratio: float = 0.50,
) -> dict:
    """计算第一块板的剩余空间辅助块补偿长度。"""

    board_length_mm = float(board_length_mm)
    pallet_length_mm = float(pallet_length_mm)
    pad_trigger_ratio = float(pad_trigger_ratio)

    if board_length_mm < 0:
        raise ValueError("board_length_mm 不能为负")
    if pallet_length_mm <= 0:
        raise ValueError("pallet_length_mm 必须 > 0")
    if not 0.0 <= pad_trigger_ratio <= 1.0:
        raise ValueError("pad_trigger_ratio 必须位于 [0,1]")

    normal_count = int(np.floor(board_length_mm / pallet_length_mm))
    remaining_mm = board_length_mm - normal_count * pallet_length_mm

    if abs(remaining_mm) < 1e-6:
        remaining_mm = 0.0

    remaining_ratio = remaining_mm / pallet_length_mm

    if remaining_mm <= 1e-6:
        pad_required = False
        compensation_mm = 0.0
        final_count = normal_count
    elif remaining_ratio >= pad_trigger_ratio:
        pad_required = True
        compensation_mm = pallet_length_mm - remaining_mm
        final_count = normal_count + 1
    else:
        pad_required = False
        compensation_mm = 0.0
        final_count = normal_count

    return {
        "pallet_length_mm": round(pallet_length_mm, 3),
        "normal_pallet_count": normal_count,
        "remaining_length_mm": round(float(remaining_mm), 3),
        "remaining_ratio": round(float(remaining_ratio), 6),
        "pad_trigger_ratio": round(float(pad_trigger_ratio), 6),
        "pad_required": pad_required,
        "remaining_workface_compensation_mm": round(
            float(compensation_mm),
            3,
        ),
        "final_pallet_count": final_count,
    }


def _midpoint(point_a, point_b) -> np.ndarray:
    return (
        np.asarray(point_a, dtype=np.float64)
        + np.asarray(point_b, dtype=np.float64)
    ) / 2.0


def correct_p5_p6(
    p3,
    p4,
    p5_lidar,
    p6_lidar,
    p7,
    p8,
    required_compensation_mm: float,
) -> dict:
    """根据辅助块占用长度修正雷达 P5/P6。

    方向：
        优先使用第二块板的纵向方向：
            midpoint(P5,P6) -> midpoint(P7,P8)

    当前间距：
        midpoint(P3,P4) -> midpoint(P5,P6)
        在第二块板纵向方向上的投影长度。

    仅当当前间距小于 required_compensation_mm 时向后补足。
    """

    p3 = np.asarray(p3, dtype=np.float64)
    p4 = np.asarray(p4, dtype=np.float64)
    p5 = np.asarray(p5_lidar, dtype=np.float64)
    p6 = np.asarray(p6_lidar, dtype=np.float64)
    p7 = np.asarray(p7, dtype=np.float64)
    p8 = np.asarray(p8, dtype=np.float64)

    m34 = _midpoint(p3, p4)
    m56 = _midpoint(p5, p6)
    m78 = _midpoint(p7, p8)

    second_u = m78 - m56
    second_u_norm = float(np.linalg.norm(second_u))

    if second_u_norm < 1e-9:
        # 极端情况下退化为 P3/P4 -> P7/P8
        second_u = m78 - m34
        second_u_norm = float(np.linalg.norm(second_u))

    if second_u_norm < 1e-9:
        raise ValueError("无法确定第二块板纵向移动方向")

    second_u = second_u / second_u_norm

    current_gap_mm = float(np.dot(m56 - m34, second_u))
    required_gap_mm = max(0.0, float(required_compensation_mm))

    shift_mm = max(
        0.0,
        required_gap_mm - current_gap_mm,
    )

    shift_vector = second_u * shift_mm

    corrected_p5 = p5 + shift_vector
    corrected_p6 = p6 + shift_vector

    corrected_gap_mm = float(
        np.dot(
            _midpoint(corrected_p5, corrected_p6) - m34,
            second_u,
        )
    )

    return {
        "original_p5_world_xyz_mm": [
            round(float(v), 3) for v in p5
        ],
        "original_p6_world_xyz_mm": [
            round(float(v), 3) for v in p6
        ],
        "required_gap_mm": round(required_gap_mm, 3),
        "current_gap_mm": round(current_gap_mm, 3),
        "shift_distance_mm": round(float(shift_mm), 3),
        "shift_vector_world_mm": [
            round(float(v), 3) for v in shift_vector
        ],
        "corrected_gap_mm": round(corrected_gap_mm, 3),
        "corrected_p5_world_xyz_mm": [
            round(float(v), 3) for v in corrected_p5
        ],
        "corrected_p6_world_xyz_mm": [
            round(float(v), 3) for v in corrected_p6
        ],
        "p5_corrected": bool(shift_mm > 1e-6),
        "p6_corrected": bool(shift_mm > 1e-6),
        "_p5": corrected_p5,
        "_p6": corrected_p6,
    }



def divide_loading_regions(
    corners_world_mm,
    row_length_mm: float = 1200.0,
) -> dict:
    """将一块板划分为 2 列 × N 排装载区域。

    输入角点顺序：
        P1/P2：该板起始端左右角点
        P3/P4：该板末端左右角点

    划分规则：
        - 宽度方向固定对半，得到 A/B 两列；
        - 长度方向从起始端向末端，每 1200 mm 一排；
        - 只生成完整 1200 mm 排；
        - 末端不足 1200 mm 的剩余长度只记录，不生成装载区域；
        - 不计算每个小区域的平均高度。
    """

    corners = np.asarray(corners_world_mm, dtype=np.float64)
    if corners.shape != (4, 3):
        raise ValueError(
            f"区域划分角点必须为 [4,3]，当前为 {corners.shape}"
        )

    if row_length_mm <= 0:
        raise ValueError("row_length_mm 必须 > 0")

    start_left, start_right, end_left, end_right = corners

    left_length = float(np.linalg.norm(end_left - start_left))
    right_length = float(np.linalg.norm(end_right - start_right))
    board_length_mm = (left_length + right_length) / 2.0

    if board_length_mm < 1e-9:
        raise ValueError("板长度过小，无法划分装载区域")

    full_row_count = int(np.floor(board_length_mm / row_length_mm))
    remaining_length_mm = (
        board_length_mm - full_row_count * row_length_mm
    )

    def lerp(a, b, fraction):
        return a + (b - a) * float(fraction)

    regions = []

    for row_index in range(full_row_count):
        start_distance = row_index * row_length_mm
        end_distance = (row_index + 1) * row_length_mm

        f0 = start_distance / board_length_mm
        f1 = end_distance / board_length_mm

        row_start_left = lerp(start_left, end_left, f0)
        row_start_right = lerp(start_right, end_right, f0)
        row_end_left = lerp(start_left, end_left, f1)
        row_end_right = lerp(start_right, end_right, f1)

        row_start_mid = (row_start_left + row_start_right) / 2.0
        row_end_mid = (row_end_left + row_end_right) / 2.0

        # A列：Vmin侧半区
        regions.append(
            {
                "region_id": f"A{row_index + 1}",
                "row_index": row_index + 1,
                "column": "A",
                "row_length_mm": round(float(row_length_mm), 3),
                "corners_world_xyz_mm": [
                    [round(float(v), 3) for v in row_start_left],
                    [round(float(v), 3) for v in row_start_mid],
                    [round(float(v), 3) for v in row_end_left],
                    [round(float(v), 3) for v in row_end_mid],
                ],
            }
        )

        # B列：Vmax侧半区
        regions.append(
            {
                "region_id": f"B{row_index + 1}",
                "row_index": row_index + 1,
                "column": "B",
                "row_length_mm": round(float(row_length_mm), 3),
                "corners_world_xyz_mm": [
                    [round(float(v), 3) for v in row_start_mid],
                    [round(float(v), 3) for v in row_start_right],
                    [round(float(v), 3) for v in row_end_mid],
                    [round(float(v), 3) for v in row_end_right],
                ],
            }
        )

    return {
        "columns": 2,
        "row_length_mm": round(float(row_length_mm), 3),
        "full_row_count": full_row_count,
        "region_count": len(regions),
        "remaining_length_mm": round(float(remaining_length_mm), 3),
        "regions": regions,
    }


def process_hybrid_corners(
    camera_corners: list[dict],
    lidar_p5_p6_world: dict,
    camera_name: str,
    intrinsic_config: dict,
    coordinate_config: dict,
    depth_mode: str = "mm",
    config: CameraGeometryConfig = DEFAULT_CONFIG,
) -> dict:
    """系统集成主接口。

    camera_corners 必须包含：
        P1, P2, P3, P4, P7, P8

    lidar_p5_p6_world 必须包含：
        P5: [x,y,z]
        P6: [x,y,z]

    返回最终 P1~P8 世界坐标以及两块板最终几何。
    """

    expected_camera_ids = {"P1", "P2", "P3", "P4", "P7", "P8"}

    camera_by_id = {
        str(corner["id"]): corner
        for corner in camera_corners
    }

    missing = expected_camera_ids - set(camera_by_id)
    if missing:
        raise ValueError(
            f"缺少相机最终角点：{sorted(missing)}"
        )

    if "P5" not in lidar_p5_p6_world or "P6" not in lidar_p5_p6_world:
        raise ValueError("雷达输入必须包含 P5 和 P6 世界坐标")

    intrinsic = get_camera_intrinsic(
        intrinsic_config,
        camera_name,
    )
    T_world_camera = get_world_extrinsic(
        coordinate_config,
        camera_name,
    )

    world = {}

    for corner_id in ("P1", "P2", "P3", "P4", "P7", "P8"):
        world[corner_id] = camera_corner_to_world(
            camera_by_id[corner_id],
            intrinsic,
            T_world_camera,
            depth_mode,
        )

    # P5/P6：直接使用雷达阶段已经转换到 world 的坐标
    world["P5"] = np.asarray(
        lidar_p5_p6_world["P5"],
        dtype=np.float64,
    )
    world["P6"] = np.asarray(
        lidar_p5_p6_world["P6"],
        dtype=np.float64,
    )

    if world["P5"].shape != (3,) or world["P6"].shape != (3,):
        raise ValueError("雷达 P5/P6 必须为 [x,y,z]")

    # 第一块板全部由相机最终角点构成
    first_geometry = compute_board_geometry(
        [
            world["P1"],
            world["P2"],
            world["P3"],
            world["P4"],
        ],
        ["P1", "P2", "P3", "P4"],
        config.reference_forward_xy,
    )

    loading_plan = compute_first_workface_loading_plan(
        first_geometry["length_mm"],
        config.pallet_length_mm,
        config.pad_trigger_ratio,
    )

    compensation_mm = float(
        loading_plan["remaining_workface_compensation_mm"]
    )

    correction = correct_p5_p6(
        world["P3"],
        world["P4"],
        world["P5"],
        world["P6"],
        world["P7"],
        world["P8"],
        compensation_mm,
    )

    # 用修正后的 P5/P6 替换雷达原始点
    world["P5"] = correction.pop("_p5")
    world["P6"] = correction.pop("_p6")

    # 第二块板：P5/P6（雷达+修正） + P7/P8（相机）
    second_geometry = compute_board_geometry(
        [
            world["P5"],
            world["P6"],
            world["P7"],
            world["P8"],
        ],
        ["P5", "P6", "P7", "P8"],
        config.reference_forward_xy,
    )

    # P5/P6 修正完成后，对第二块板进行最终装载区域划分：
    # 宽度方向对半为 A/B 两列；长度方向每 1200 mm 一排。
    # 每个小区域不计算平均高度。
    second_loading_regions = divide_loading_regions(
        [
            world["P5"],
            world["P6"],
            world["P7"],
            world["P8"],
        ],
        row_length_mm=config.pallet_length_mm,
    )

    final_corners = {
        corner_id: [
            round(float(v), 3)
            for v in world[corner_id]
        ]
        for corner_id in (
            "P1", "P2", "P3", "P4",
            "P5", "P6", "P7", "P8",
        )
    }

    return {
        "final_corners_world_xyz_mm": final_corners,
        "first_board": {
            **first_geometry,
            "loading_plan": loading_plan,
        },
        "p5_p6_correction": correction,
        "second_board": {
            **second_geometry,
            "loading_regions": second_loading_regions,
        },
    }


def process_input_file(
    input_json: str | Path,
    intrinsic_json: str | Path,
    coordinate_json: str | Path,
    output_json: str | Path,
) -> dict:
    payload = load_json(input_json)
    intrinsic_config = load_json(intrinsic_json)
    coordinate_config = load_json(coordinate_json)

    planning = payload.get("planning_config", {})
    config = CameraGeometryConfig(
        pallet_length_mm=float(
            planning.get("pallet_length_mm", 1200.0)
        ),
        pad_trigger_ratio=float(
            planning.get("pad_trigger_ratio", 0.50)
        ),
        reference_forward_xy=tuple(
            planning.get("reference_forward_xy", [0.0, 1.0])
        ),
    )

    result = process_hybrid_corners(
        camera_corners=payload["camera_corners"],
        lidar_p5_p6_world=payload["lidar_p5_p6_world"],
        camera_name=payload["camera_name"],
        intrinsic_config=intrinsic_config,
        coordinate_config=coordinate_config,
        depth_mode=payload.get("depth_mode", "mm"),
        config=config,
    )

    output_json = Path(output_json)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return result
