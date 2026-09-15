# -*- coding: utf-8 -*-
"""实验室雷达算法适配：输出对齐主流程的雷达结果字典。"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Optional

from algorithm_modules.lab.lidar_lab.lidar_module import process_pcd


LAB_LIDAR_DIR = Path(__file__).resolve().parents[1] / "algorithm_modules" / "lab" / "lidar_lab"
DEFAULT_EXTRINSIC = LAB_LIDAR_DIR / "lidar_extrinsic.json"


def process_lab_lidar_pcd(
    pcd_path: str | Path,
    *,
    result_tag: str = "",
    extrinsic_path: Optional[str | Path] = None,
) -> Dict[str, Any]:
    """调用项目内 lidar_lab，返回可被 FlowController 归一化的结果。"""
    path = Path(pcd_path)
    if not path.is_file():
        return {
            "success": False,
            "message": f"实验室雷达：找不到 PCD：{path}",
            "algorithm": "lab_lidar",
        }

    extrinsic = Path(extrinsic_path) if extrinsic_path else DEFAULT_EXTRINSIC
    try:
        # process_pcd 内部默认读同目录 extrinsic；此处显式校验文件存在
        if not extrinsic.is_file():
            raise FileNotFoundError(f"找不到实验室雷达外参：{extrinsic}")
        raw = process_pcd(path)
    except Exception as exc:
        return {
            "success": False,
            "message": f"实验室雷达点云处理失败：{exc}",
            "algorithm": "lab_lidar",
            "pcd_path": str(path),
            "result_tag": result_tag,
        }

    ids = ["P1", "P2", "P3", "P4"]
    world_points = {}
    for name in ids:
        xyz = raw.get(name)
        if not isinstance(xyz, (list, tuple)) or len(xyz) < 3:
            return {
                "success": False,
                "message": f"实验室雷达结果缺少有效 {name}",
                "algorithm": "lab_lidar",
                "raw": deepcopy(raw),
            }
        world_points[name] = {
            "x": float(xyz[0]),
            "y": float(xyz[1]),
            "z": float(xyz[2]),
            "name": name,
        }

    return {
        "success": True,
        "algorithm": "lab_lidar",
        "message": "实验室雷达几何提取完成（P1-P4）",
        "coordinate_frame": "world",
        "coordinate_unit": "mm",
        "corner_ids": ids,
        "world_points": world_points,
        "length_mm": float(raw.get("length_mm", 0.0) or 0.0),
        "width_mm": float(raw.get("width_mm", 0.0) or 0.0),
        "pcd_path": str(path),
        "extrinsic_path": str(extrinsic),
        "result_tag": result_tag,
        "lab_raw": deepcopy(raw),
    }
