# -*- coding: utf-8 -*-
"""最小流程模拟设备。真实机器人/相机/雷达只需实现 adapters.base 中接口。"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Sequence

from .base import CameraAdapter, RadarAdapter, RobotAdapter


class MockRobotAdapter(RobotAdapter):
    def check_status(self) -> bool:
        return True

    def move_to_pallet(self) -> dict:
        return {"success": True, "message": "机器人已到达托盘识别位"}

    def align_with_pallet(self, pallet_result: dict) -> dict:
        return {
            "success": True,
            "message": "已依据左右插孔 XYZ 完成叉臂对位规划",
            "left_xyz_mm": pallet_result.get("left_xyz_mm"),
            "right_xyz_mm": pallet_result.get("right_xyz_mm"),
        }

    def move_to_cargo(self, cargo: dict) -> dict:
        return {
            "success": True,
            "message": f"已前往货位 {cargo.get('storage_location') or '未指定货位'}",
            "cargo": cargo,
        }

    def load_cargo_to_pallet(self, cargo: dict) -> dict:
        return {"success": True, "message": "货物已放置到托盘上", "cargo": cargo}

    def fork_pallet(self, pallet_result: dict, deviation_result: dict) -> dict:
        return {
            "success": True,
            "message": "插取前偏差合格，已按插孔位置插取托盘",
            "left_xyz_mm": pallet_result.get("left_xyz_mm"),
            "right_xyz_mm": pallet_result.get("right_xyz_mm"),
            "deviation_gate": deviation_result,
        }

    def place_cargo_on_truck(self, cargo: dict, target: dict, corner_result: dict) -> dict:
        return {
            "success": True,
            "message": f"货物/托盘已按点云规划放置到 {target.get('label') or target}",
            "cargo": cargo,
            "target": target,
            "corner_points_xyz_mm": corner_result.get("corner_points_xyz_mm") or corner_result.get("corner_points_xyz") or [],
        }

    def return_for_next_round(self) -> dict:
        return {"success": True, "message": "机器人已返回下一轮起始位"}


class MockCameraAdapter(CameraAdapter):
    def __init__(self):
        self.pallet_rgb_path = ""
        self.pallet_depth_path = ""
        self.corner_image_paths = {f"P{i}": "" for i in range(1, 9)}

    def check_status(self) -> bool:
        return True

    def set_pallet_rgbd(self, rgb_path: str, depth_path: str):
        self.pallet_rgb_path = str(rgb_path or "")
        self.pallet_depth_path = str(depth_path or "")

    def set_corner_images(self, paths: dict):
        paths = paths or {}
        self.corner_image_paths = {f"P{i}": str(paths.get(f"P{i}") or "") for i in range(1, 9)}

    @staticmethod
    def _file(path: str):
        p = Path(path).expanduser() if path else None
        return p.resolve() if p and p.is_file() else None

    def capture_pallet_rgbd(self) -> dict:
        rgb = self._file(self.pallet_rgb_path)
        depth = self._file(self.pallet_depth_path)
        return {
            "success": bool(rgb and depth),
            "rgb_path": str(rgb) if rgb else "",
            "depth_path": str(depth) if depth else "",
            "message": "已取得托盘 RGB-D 数据" if rgb and depth else "当前未配置完整托盘 RGB-D 数据",
        }

    def capture_corner_images(self, cargo: dict, expected_corner_ids: Sequence[str] | None = None) -> dict:
        expected = [str(x) for x in (expected_corner_ids or ("P1", "P2", "P3", "P4"))]
        cargo_images = cargo.get("corner_images") if isinstance(cargo, dict) else None
        cargo_images = cargo_images if isinstance(cargo_images, dict) else {}
        configured = {
            name: cargo_images.get(name) or self.corner_image_paths.get(name) or ""
            for name in expected
        }
        image_paths = {}
        missing = []
        for name in expected:
            path = self._file(str(configured.get(name) or "").strip())
            if path:
                image_paths[name] = str(path)
            else:
                missing.append(name)
        return {
            "success": len(missing) == 0,
            "image_paths_by_point": image_paths,
            "image_count": len(image_paths),
            "expected_corner_ids": expected,
            "missing_points": missing,
            "message": (
                f"已取得 {len(expected)} 张车板角点局部图"
                if not missing
                else f"角点图像不完整：当前 {len(image_paths)}/{len(expected)}，缺少 {', '.join(missing)}"
            ),
        }


class MockRadarAdapter(RadarAdapter):
    """点云文件模式的雷达适配器 + 现有偏差/动态监测联调接口。"""

    def __init__(self):
        self.point_cloud_path = ""

    def check_status(self) -> bool:
        return True

    def set_point_cloud_path(self, path: str):
        self.point_cloud_path = str(path or "")

    @staticmethod
    def _file(path: str):
        p = Path(path).expanduser() if path else None
        return p.resolve() if p and p.is_file() else None

    def capture_truck_point_cloud(self, cargo: dict) -> dict:
        cargo_path = str((cargo or {}).get("point_cloud_path") or "")
        path = self._file(cargo_path or self.point_cloud_path)
        return {
            "success": bool(path and path.suffix.lower() == ".pcd"),
            "pcd_path": str(path) if path else "",
            "message": "已取得雷达 PCD 点云" if path else "当前未配置雷达 PCD 点云文件",
        }

    def measure_pallet_cargo_deviation(self, cargo: dict, phase: str) -> dict:
        prefix = "pre_pick" if phase == "pre_pick" else "after_place"
        return {
            "success": True,
            "phase": phase,
            "coordinate_frame": "pallet_local",
            "coordinate_unit": "mm",
            "dx_mm": float(cargo.get(f"{prefix}_dx_mm", 0.0) or 0.0),
            "dy_mm": float(cargo.get(f"{prefix}_dy_mm", 0.0) or 0.0),
            "dz_mm": float(cargo.get(f"{prefix}_dz_mm", 0.0) or 0.0),
            "yaw_deg": float(cargo.get(f"{prefix}_yaw_deg", 0.0) or 0.0),
            "message": "已测量货物相对托盘中心/朝向偏差",
        }

    @staticmethod
    def _target_pose(entry: dict) -> dict:
        cargo = entry.get("cargo", entry) if isinstance(entry, dict) else {}
        plan = entry.get("placement_plan") if isinstance(entry, dict) else None
        plan = plan if isinstance(plan, dict) else {}
        seq = int(cargo.get("sequence") or 1)

        # 如果未来规划器给出实际参考坐标，优先用于动态监测；否则使用联调默认值。
        center = plan.get("reference_center_xyz_mm")
        if isinstance(center, (list, tuple)) and len(center) >= 3:
            x, y, z = [float(center[i]) for i in range(3)]
        else:
            x = float(cargo.get("target_x_mm", seq * 1200.0) or seq * 1200.0)
            y = float(cargo.get("target_y_mm", 0.0) or 0.0)
            z = float(cargo.get("target_z_mm", 0.0) or 0.0)
        yaw = float(cargo.get("target_yaw_deg", 0.0) or 0.0)
        return {"x_mm": x, "y_mm": y, "z_mm": z, "yaw_deg": yaw}

    def monitor_truck_pallets(self, loaded_pallets: list, phase: str, sample_count: int = 5) -> dict:
        pallets = []
        count = max(1, int(sample_count))
        for entry in loaded_pallets:
            cargo = entry.get("cargo", entry) if isinstance(entry, dict) else {}
            pose = self._target_pose(entry if isinstance(entry, dict) else {})
            drift = float(cargo.get("monitor_drift_mm", 0.0) or 0.0)
            yaw_drift = float(cargo.get("monitor_yaw_drift_deg", 0.0) or 0.0)
            samples = []
            for i in range(count):
                ratio = i / max(1, count - 1)
                samples.append({
                    "x_mm": pose["x_mm"] + drift * ratio,
                    "y_mm": pose["y_mm"],
                    "z_mm": pose["z_mm"],
                    "yaw_deg": pose["yaw_deg"] + yaw_drift * ratio,
                })
            pallets.append({
                "instance_id": cargo.get("instance_id"),
                "target_label": (entry.get("placement_plan") or {}).get("label") if isinstance(entry, dict) else None,
                "samples": samples,
            })
        return {
            "success": True,
            "phase": phase,
            "coordinate_frame": "truck_bed",
            "coordinate_unit": "mm",
            "pallets": pallets,
            "message": f"已完成 {phase} 卡车底板托盘位置动态采样",
        }
