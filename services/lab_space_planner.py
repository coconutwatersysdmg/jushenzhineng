# -*- coding: utf-8 -*-
"""实验室模式：由相机最终 WORLD 角点生成后台装载区域。"""
from __future__ import annotations

from copy import deepcopy
from math import sqrt
from typing import Any, Mapping, Sequence

from services.camera_board_geometry_service import CameraBoardGeometryService
from services.space_manager import SpaceManager


LAB_EQUAL_ROW_COUNT = 5


def _world_xyz(point: Any) -> list[float]:
    if isinstance(point, Mapping):
        values = [
            point.get("x", point.get("x_mm")),
            point.get("y", point.get("y_mm")),
            point.get("z", point.get("z_mm")),
        ]
    else:
        values = list(point[:3])
    if len(values) != 3 or any(value is None for value in values):
        raise RuntimeError(f"实验室最终 WORLD 角点无效：{point!r}")
    return [float(value) for value in values]


def _lerp_point(start: Sequence[float], end: Sequence[float], fraction: float) -> list[float]:
    return [
        float(start[index]) + (float(end[index]) - float(start[index])) * float(fraction)
        for index in range(3)
    ]


def _midpoint(left: Sequence[float], right: Sequence[float]) -> list[float]:
    return [0.5 * (float(left[index]) + float(right[index])) for index in range(3)]


def _rounded(point: Sequence[float]) -> list[float]:
    return [round(float(value), 3) for value in point]


def _equal_lab_regions_from_world_corners(
    final_world_points: Mapping[str, Any],
    corner_ids: Sequence[str],
    *,
    row_count: int = LAB_EQUAL_ROW_COUNT,
) -> tuple[list[dict[str, Any]], float]:
    """使用相机最终 WORLD 边界插值生成两列×等长五排。"""
    ids = [str(value) for value in corner_ids]
    if len(ids) not in {4, 6}:
        raise RuntimeError(f"实验室等分需要 4/6 个最终 WORLD 角点，当前：{ids}")
    if any(point_id not in final_world_points for point_id in ids):
        raise RuntimeError("实验室最终 WORLD 角点不完整")
    start_left = _world_xyz(final_world_points[ids[0]])
    start_right = _world_xyz(final_world_points[ids[1]])
    end_left = _world_xyz(final_world_points[ids[-2]])
    end_right = _world_xyz(final_world_points[ids[-1]])
    left_length = sqrt(sum((end_left[i] - start_left[i]) ** 2 for i in range(3)))
    right_length = sqrt(sum((end_right[i] - start_right[i]) ** 2 for i in range(3)))
    row_length_mm = 0.5 * (left_length + right_length) / int(row_count)

    regions: list[dict[str, Any]] = []
    for row_index in range(1, int(row_count) + 1):
        f0 = (row_index - 1) / float(row_count)
        f1 = row_index / float(row_count)
        left0 = _lerp_point(start_left, end_left, f0)
        right0 = _lerp_point(start_right, end_right, f0)
        left1 = _lerp_point(start_left, end_left, f1)
        right1 = _lerp_point(start_right, end_right, f1)
        middle0 = _midpoint(left0, right0)
        middle1 = _midpoint(left1, right1)
        for column, corners in (
            ("A", [left0, middle0, left1, middle1]),
            ("B", [middle0, right0, middle1, right1]),
        ):
            center = [sum(point[axis] for point in corners) / 4.0 for axis in range(3)]
            region_id = f"{column}{row_index}"
            regions.append({
                "section": "LAB_EQUAL_WORLD_GRID",
                "local_row_index": row_index,
                "column": column,
                "row_length_mm": round(row_length_mm, 3),
                "corners_world_xyz_mm": [_rounded(point) for point in corners],
                "center_world_xyz_mm": _rounded(center),
                "status": "AVAILABLE",
                "cargo_id": None,
                "requires_support_block": False,
                "support_height_compensation_mm": 0.0,
                "blind_code": region_id,
                "region_id": region_id,
            })
    return regions, round(row_length_mm, 3)


def _relabel_lab_regions_from_p1_p2(geometry: dict[str, Any]) -> dict[str, Any]:
    """实验室按图片约定编号：P1/P2 端为第 1 排，向 P3/P4 递增。"""
    out = deepcopy(geometry)
    regions = list(out.get("regions") or [])
    regions.sort(key=lambda item: (
        int(item.get("local_row_index", 0) or 0),
        str(item.get("column") or ""),
    ))
    for region in regions:
        row = int(region.get("local_row_index", 0) or 0)
        column = str(region.get("column") or "")
        if row <= 0 or column not in {"A", "B"}:
            raise RuntimeError(f"实验室区域编号无效：column={column!r}, row={row}")
        code = f"{column}{row}"
        region["blind_code"] = code
        region["region_id"] = code
    out["regions"] = regions
    return out


def lab_b_regions(space_snapshot: Mapping[str, Any]) -> list[dict[str, Any]]:
    """返回实验室实际装货列，固定 B1 → B2 → … → B5。"""
    regions = [
        deepcopy(region)
        for region in (space_snapshot.get("regions") or [])
        if str(region.get("column") or "").upper() == "B"
    ]
    regions.sort(key=lambda item: int(item.get("local_row_index", 0) or 0))
    return regions


def lab_b_region_for_round(space_snapshot: Mapping[str, Any], round_index: int) -> dict[str, Any]:
    regions = lab_b_regions(space_snapshot)
    index = int(round_index)
    if index < 0 or index >= len(regions):
        raise RuntimeError(f"实验室 B 列没有第 {index + 1} 个可用区域")
    return regions[index]


def build_lab_space_plan(
    final_world_points: Mapping[str, Any],
    corner_ids: Sequence[str],
    *,
    board_geometry: CameraBoardGeometryService | None = None,
    space_manager: SpaceManager | None = None,
) -> dict[str, Any]:
    """把实验室最终 WORLD 角点划分为区域，但不生成 UI 专用显示数据。"""
    ids = [str(value) for value in corner_ids]
    non_camera = [
        point_id
        for point_id in ids
        if str((final_world_points.get(point_id) or {}).get("source") or "") != "camera_yolo"
    ]
    if non_camera:
        raise RuntimeError(
            "相机最终 WORLD 四角不完整，禁止用雷达粗角点划格：" + ", ".join(non_camera)
        )
    geometry_service = board_geometry or CameraBoardGeometryService()
    manager = space_manager or SpaceManager()
    geometry = geometry_service.analyze(dict(final_world_points), ids)
    equal_regions, equal_row_length_mm = _equal_lab_regions_from_world_corners(
        final_world_points,
        ids,
    )
    geometry["regions"] = equal_regions
    geometry["lab_equal_row_count"] = LAB_EQUAL_ROW_COUNT
    geometry["lab_equal_column_count"] = 2
    geometry["lab_grid_source"] = "final_world_corners"
    geometry["row_length_mm"] = equal_row_length_mm
    geometry["remaining_length_mm"] = 0.0
    geometry["message"] = "实验室相机最终 WORLD 四角已动态等分为两列×五排"
    geometry = _relabel_lab_regions_from_p1_p2(geometry)
    space = manager.initialize_from_camera_geometry(geometry)
    loading_order = [region["region_id"] for region in lab_b_regions(space)]
    return {
        "success": True,
        "geometry": deepcopy(geometry),
        "space": deepcopy(space),
        "loading_order": loading_order,
        "loading_column": "B",
        "ui_display": False,
        "message": (
            f"实验室后台划格完成：{geometry.get('board_mode')}，"
            f"生成 {len(space.get('regions') or [])} 个区域；装货顺序 "
            + " → ".join(loading_order)
        ),
    }
