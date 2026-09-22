# -*- coding: utf-8 -*-
"""实验室模式：由相机最终 WORLD 角点生成后台装载区域。"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping, Sequence

from services.camera_board_geometry_service import CameraBoardGeometryService
from services.space_manager import SpaceManager


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
    """返回实验室实际装货列，固定 B1 → B2 → B3…。"""
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
    geometry_service = board_geometry or CameraBoardGeometryService()
    manager = space_manager or SpaceManager()
    geometry = geometry_service.analyze(dict(final_world_points), list(corner_ids))
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
