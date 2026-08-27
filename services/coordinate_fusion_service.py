# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, Mapping, Sequence

from core.geometry import Pose6D, plane_from_points, ray_plane_intersection, rotate_vector


class CoordinateFusionService:
    """角点图像像素 -> WORLD XYZ。

    相机为机械臂挂载相机，capture_meta 内 camera_world_pose 来自机械臂位姿×安装外参。
    雷达粗角点只用于确定搜索区域和车板平面，不直接覆盖视觉点。
    """

    @staticmethod
    def _xyz(value):
        if isinstance(value, Mapping):
            return [float(value["x"]), float(value["y"]), float(value.get("z", 0.0))]
        return [float(value[0]), float(value[1]), float(value[2])]

    def _plane_for_point(self, point_id: str, radar_result: Dict[str, Any]):
        world = radar_result.get("world_points") or {}
        groups = radar_result.get("board_groups") or []
        for group in groups:
            ids = [str(x) for x in group.get("corner_ids", [])]
            if point_id in ids and len(ids) >= 3:
                pts = [self._xyz(world[n]) for n in ids[:3] if n in world]
                if len(pts) == 3:
                    return plane_from_points(*pts)
        # 平板/无组信息：从全部雷达粗角点中取三个。
        ids = [str(x) for x in radar_result.get("corner_ids", []) if str(x) in world]
        if len(ids) >= 3:
            return plane_from_points(self._xyz(world[ids[0]]), self._xyz(world[ids[1]]), self._xyz(world[ids[2]]))
        raise RuntimeError("雷达数据不足，无法确定车板平面")

    def fuse(self, image_points: Dict[str, Any], capture_meta: Dict[str, Any], radar_result: Dict[str, Any]) -> Dict[str, Any]:
        fused = {}
        details = {}
        for pid, det in image_points.items():
            meta = capture_meta.get(pid) or {}
            pose = Pose6D.from_any(meta.get("camera_world_pose"))
            intr = meta.get("intrinsics") or {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0}
            u, v = float(det["x"]), float(det["y"])
            ray_cam = [(u-float(intr["cx"]))/float(intr["fx"]), (v-float(intr["cy"]))/float(intr["fy"]), 1.0]
            ray_world = rotate_vector(pose, ray_cam)
            origin = [pose.x_mm, pose.y_mm, pose.z_mm]
            n, d = self._plane_for_point(pid, radar_result)
            hit = ray_plane_intersection(origin, ray_world, n, d)
            if hit is None:
                rough = (radar_result.get("world_points") or {}).get(pid)
                if rough is None:
                    raise RuntimeError(f"{pid} 视觉射线与车板平面无有效交点")
                hit = self._xyz(rough)
                mode = "radar_fallback"
            else:
                mode = "visual_ray_plane"
            fused[pid] = {"x": hit[0], "y": hit[1], "z": hit[2]}
            details[pid] = {"mode": mode, "uv": [u, v], "camera_id": meta.get("camera_id"), "camera_world_pose": pose.to_dict()}
        ids = [str(x) for x in radar_result.get("corner_ids", []) if str(x) in fused]
        return {
            "success": True,
            "coordinate_frame": "WORLD", "coordinate_unit": "mm",
            "world_points": fused,
            "corner_ids": ids,
            "corner_points_xyz_mm": [[fused[i]["x"], fused[i]["y"], fused[i]["z"]] for i in ids],
            "fusion_details": details,
        }
