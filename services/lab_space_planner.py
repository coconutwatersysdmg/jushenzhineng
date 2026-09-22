# -*- coding: utf-8 -*-
"""实验室模式：由相机最终 WORLD 角点生成后台装载区域。"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping, Sequence

from services.camera_board_geometry_service import CameraBoardGeometryService
from services.space_manager import SpaceManager


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
    space = manager.initialize_from_camera_geometry(geometry)
    return {
        "success": True,
        "geometry": deepcopy(geometry),
        "space": deepcopy(space),
        "ui_display": False,
        "message": f"实验室后台划格完成：{geometry.get('board_mode')}，生成 {len(space.get('regions') or [])} 个区域",
    }
