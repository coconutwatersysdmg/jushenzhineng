from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

from algorithm_modules.lab.cam_yolo_lab.camera_world_module import CameraWorldTransform


LAB_CORNER_PAIRS = (("P3", "P4"), ("P1", "P2"))
LAB_CAMERA_Z_MM = 380.0
LAB_CAMERA_R_DEG = -80.0


def merge_lab_corner_points(
    radar_points: Mapping[str, Any],
    camera_points: Mapping[str, Any] | None,
) -> dict[str, dict[str, Any]]:
    """Merge camera WORLD points over radar WORLD points one corner at a time."""
    merged: dict[str, dict[str, Any]] = {}
    camera_points = camera_points or {}
    for name, radar_raw in (radar_points or {}).items():
        radar_xyz = _try_point_xyz(radar_raw)
        if radar_xyz is None:
            continue
        camera_xyz = _try_point_xyz(camera_points.get(name))
        if camera_xyz is not None:
            xyz = camera_xyz
            source = "camera_yolo"
        else:
            xyz = radar_xyz
            source = "lidar_fallback"
        merged[str(name)] = {
            "x": float(xyz[0]),
            "y": float(xyz[1]),
            "z": float(xyz[2]),
            "source": source,
            "camera_world_xyz_mm": list(camera_xyz) if camera_xyz is not None else None,
            "lidar_world_xyz_mm": list(radar_xyz),
            "final_world_xyz_mm": list(xyz),
        }
    return merged


class LabCameraVisitPlanner:
    """Plan the archived laboratory two-shot camera visit in WORLD coordinates."""

    def __init__(self, transform: CameraWorldTransform):
        self.transform = transform

    def build_pair_targets(
        self,
        world_points: Mapping[str, Any],
        planning_pose: Mapping[str, float] | None = None,
    ) -> list[dict[str, Any]]:
        pose = dict(planning_pose or {})
        z = float(pose.get("z", LAB_CAMERA_Z_MM))
        # R 由启动锁定角度传入；不再强制改成 -80
        r = float(pose.get("r", LAB_CAMERA_R_DEG))
        if not math.isclose(z, LAB_CAMERA_Z_MM):
            raise ValueError(f"实验室相机访问姿态 Z 必须是 {LAB_CAMERA_Z_MM}，当前为 {z}")

        targets: list[dict[str, Any]] = []
        for pair in LAB_CORNER_PAIRS:
            points = [self._point_xyz(world_points, name) for name in pair]
            center = tuple(sum(point[index] for point in points) / 2.0 for index in range(3))
            xy = self.transform.plc_xy_for_camera_axis_target(
                center,
                {"x": 0.0, "y": 0.0, "z": LAB_CAMERA_Z_MM, "r": r},
            )
            targets.append(
                {
                    "pair": pair,
                    "target_world": list(center),
                    "plc_command": {
                        "X": float(xy["X"]),
                        "Y": float(xy["Y"]),
                        "Z": LAB_CAMERA_Z_MM,
                        "R": float(r),
                    },
                }
            )
        return targets

    def build_world_target(
        self,
        target_world: Sequence[float],
        planning_pose: Mapping[str, float] | None = None,
        *,
        task: str = "LAB_REGION_MONITOR",
    ) -> dict[str, Any]:
        """让相机光轴对准一个 WORLD 点；Z 固定，R 保持启动时角度。"""
        values = tuple(float(value) for value in target_world[:3])
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError("实验室相机目标 WORLD 坐标无效")
        pose = dict(planning_pose or {})
        z = float(pose.get("z", LAB_CAMERA_Z_MM))
        r = float(pose.get("r", LAB_CAMERA_R_DEG))
        if not math.isclose(z, LAB_CAMERA_Z_MM):
            raise ValueError(f"实验室相机访问姿态 Z 必须是 {LAB_CAMERA_Z_MM}，当前为 {z}")
        xy = self.transform.plc_xy_for_camera_axis_target(
            values,
            {"x": 0.0, "y": 0.0, "z": z, "r": r},
        )
        return {
            "task": str(task),
            "target_world": list(values),
            "plc_command": {
                "X": float(xy["X"]),
                "Y": float(xy["Y"]),
                "Z": z,
                "R": r,
            },
        }

    @staticmethod
    def plc_to_world_pose(
        command: Mapping[str, float],
        mapping: Mapping[str, Any],
    ) -> dict[str, float]:
        """Invert the existing linear WORLD→gantry mapping for move_tool_world."""
        cfg = dict(mapping or {})
        if not cfg:
            raise ValueError("world_to_gantry 映射为空")
        if bool(cfg.get("placeholder", False)):
            raise ValueError("world_to_gantry.placeholder=True，不能规划实验室相机移动")
        world_origin = dict(cfg.get("world_origin_mm") or {})
        gantry_origin = dict(cfg.get("gantry_origin_xyzr") or {})
        scale = dict(cfg.get("scale") or {})

        def inverse(axis: str, world_key: str, origin_key: str) -> float:
            s = float(scale.get(axis, scale.get(axis.lower(), 1.0)))
            if abs(s) < 1e-12:
                raise ValueError(f"world_to_gantry.scale.{axis} 不能为 0")
            g = float(gantry_origin.get(axis, gantry_origin.get(axis.lower(), 0.0)))
            value = float(command[axis])
            return float(world_origin.get(world_key, 0.0)) + (value - g) / s

        return {
            "x_mm": inverse("X", "x", "X"),
            "y_mm": inverse("Y", "y", "Y"),
            "z_mm": inverse("Z", "z", "Z"),
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": inverse("R", "yaw", "R"),
        }

    @staticmethod
    def _point_xyz(world_points: Mapping[str, Any], name: str) -> tuple[float, float, float]:
        raw = world_points.get(name)
        if isinstance(raw, Mapping):
            values = (raw.get("x", raw.get("x_mm")), raw.get("y", raw.get("y_mm")), raw.get("z", raw.get("z_mm")))
        elif isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
            values = tuple(raw[:3])
        else:
            raise ValueError(f"缺少实验室雷达 WORLD 点 {name}")
        if len(values) != 3 or any(value is None for value in values):
            raise ValueError(f"实验室雷达 WORLD 点 {name} 无效")
        point = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in point):
            raise ValueError(f"实验室雷达 WORLD 点 {name} 无效")
        return point


def _try_point_xyz(raw: Any) -> tuple[float, float, float] | None:
    if isinstance(raw, Mapping):
        values = (
            raw.get("x", raw.get("x_mm")),
            raw.get("y", raw.get("y_mm")),
            raw.get("z", raw.get("z_mm")),
        )
    elif isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
        values = tuple(raw[:3])
    else:
        return None
    if len(values) != 3 or any(value is None for value in values):
        return None
    try:
        point = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    return point if all(math.isfinite(value) for value in point) else None
