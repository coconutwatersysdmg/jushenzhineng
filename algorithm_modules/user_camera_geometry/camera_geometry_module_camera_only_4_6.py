# -*- coding: utf-8 -*-
"""用户上传 hybrid 版本在 v8 中的系统化替代入口。

旧 hybrid 版本：相机 P1-P4/P7-P8 + 雷达 P5/P6 -> P1-P8。
v8 当前流程：雷达只给 4/6 个粗搜索坐标；所有最终角点均由相机得到。

输入：
- image_points: 角点.pt输出的 P1~P4/P6 像素点；
- capture_meta: 每个角点对应的 camera_id / depth_path / camera_world_pose；
- radar_coarse_result: 只用于误差对比，不参与最终板型/区域判断。

输出：
- final_corners_world_xyz_mm
- camera_board_geometry
- loading_regions
"""
from __future__ import annotations

from services.camera_corner_world_service import CameraCornerWorldService
from services.camera_board_geometry_service import CameraBoardGeometryService


def process_camera_only_4_6(image_points, capture_meta, radar_coarse_result=None, depth_mode="raw"):
    converter=CameraCornerWorldService()
    world=converter.convert(image_points,capture_meta,radar_coarse_result or {},depth_mode=depth_mode)
    geometry=CameraBoardGeometryService().analyze(world["world_points"],world["corner_ids"])
    return {
        "final_corners_world_xyz_mm": {k:[v["x"],v["y"],v["z"]] for k,v in world["world_points"].items()},
        "corner_conversion_details": world["details"],
        "camera_board_geometry": geometry,
        "loading_regions": geometry["regions"],
        "board_mode": geometry["board_mode"],
        "radar_used_for_final_board_judgement": False,
    }
