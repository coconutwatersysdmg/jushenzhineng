from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable


_POINT_NAMES = ("P1", "P2", "P3", "P4")
_DEFAULT_EXTRINSIC = Path(__file__).with_name("lidar_extrinsic.json")


def load_extrinsic(extrinsic_path: str | Path = _DEFAULT_EXTRINSIC) -> dict:
    """读取雷达到世界坐标系的固定外参。"""
    path = Path(extrinsic_path)
    if not path.is_file():
        raise FileNotFoundError(f"找不到雷达外参文件：{path}")

    data = json.loads(path.read_text(encoding="utf-8"))
    rotation = data.get("R_world_from_lidar")
    translation = data.get("T_world_from_lidar_mm")

    if (
        not isinstance(rotation, list)
        or len(rotation) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in rotation)
    ):
        raise ValueError("R_world_from_lidar 必须是 3×3 矩阵。")
    if not isinstance(translation, list) or len(translation) != 3:
        raise ValueError("T_world_from_lidar_mm 必须包含 X、Y、Z 三个值。")

    return {
        "R_world_from_lidar": [[float(value) for value in row] for row in rotation],
        "T_world_from_lidar_mm": [float(value) for value in translation],
    }


def lidar_to_world(
    point_xyz_mm: Iterable[float],
    extrinsic_path: str | Path = _DEFAULT_EXTRINSIC,
) -> list[float]:
    """将一个雷达坐标点转换为世界坐标，单位 mm。"""
    point = [float(value) for value in point_xyz_mm]
    if len(point) != 3:
        raise ValueError("角点坐标必须包含 X、Y、Z 三个值。")

    calibration = load_extrinsic(extrinsic_path)
    rotation = calibration["R_world_from_lidar"]
    translation = calibration["T_world_from_lidar_mm"]

    return [
        sum(row[index] * point[index] for index in range(3)) + translation[row_index]
        for row_index, row in enumerate(rotation)
    ]


def _format_result(extractor_result: dict) -> dict:
    """只保留原实验室软件实际使用的最终结果。"""
    corners = extractor_result.get("corners_xyz_mm")
    if not isinstance(corners, dict):
        raise RuntimeError("角点提取结果缺少 corners_xyz_mm。")

    output = {}
    for name in _POINT_NAMES:
        if name not in corners:
            raise RuntimeError(f"角点提取结果缺少 {name}。")
        output[name] = lidar_to_world(corners[name])

    output["length_mm"] = float(extractor_result["length_mm"])
    output["width_mm"] = float(extractor_result["width_mm"])
    return output


def process_points(points_xyz_mm) -> dict:
    """处理 N×3 点云，返回 P1-P4 世界坐标、长度和宽度。"""
    try:
        from .truck_floor_corner_extractor import extract_truck_floor_corners
    except ImportError:  # 允许在本目录直接 python run.py
        from truck_floor_corner_extractor import extract_truck_floor_corners

    extractor_result = extract_truck_floor_corners(points_xyz_mm, return_debug=False)
    return _format_result(extractor_result)


def load_pcd_points(pcd_path: str | Path):
    """按原软件方式读取 PCD；只读，不删除、不覆盖输入文件。"""
    import numpy as np

    path = Path(pcd_path)
    if not path.is_file():
        raise FileNotFoundError(f"找不到 PCD 文件：{path}")

    raw = path.read_bytes()
    marker_index = raw.find(b"DATA ascii")
    if marker_index >= 0:
        line_end = raw.find(b"\n", marker_index)
        if line_end < 0:
            raise RuntimeError(f"PCD ASCII 数据头不完整：{path}")
        header = raw[:line_end].decode("ascii", errors="ignore")
        fields = None
        for line in header.splitlines():
            if line.upper().startswith("FIELDS "):
                fields = line.split()[1:]
                break
        if fields and all(axis in fields for axis in ("x", "y", "z")):
            values = np.fromstring(raw[line_end + 1 :], sep=" ", dtype=np.float64)
            column_count = len(fields)
            if values.size and values.size % column_count == 0:
                table = values.reshape(-1, column_count)
                indices = [fields.index(axis) for axis in ("x", "y", "z")]
                points = table[:, indices]
                if points.size:
                    return points

    import open3d as o3d

    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.size == 0:
        raise RuntimeError(f"PCD 文件为空或读取失败：{path}")
    return points


def process_pcd(pcd_path: str | Path, output_json: str | Path | None = None) -> dict:
    """处理一个 PCD 文件；原始 PCD 始终保留。"""
    result = process_points(load_pcd_points(pcd_path))
    if output_json is not None:
        save_json(result, output_json)
    return result


def save_json(result: dict, output_json: str | Path) -> Path:
    path = Path(output_json)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


__all__ = [
    "load_extrinsic",
    "lidar_to_world",
    "process_points",
    "process_pcd",
    "load_pcd_points",
    "save_json",
]
