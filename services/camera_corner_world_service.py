# -*- coding: utf-8 -*-
from __future__ import annotations

from copy import deepcopy
from math import sqrt
from typing import Any, Dict, Mapping

from services.sensor_calibration_service import SensorCalibrationService


class CameraCornerWorldService:
    """角点 .pt 的像素结果 + 同步深度 -> 最终 WORLD 角点。

    雷达粗点只用于：
    1) 把机械臂/相机移动到角点附近；
    2) 计算视觉点与粗定位点的误差作为质量检查。

    雷达点绝不覆盖相机最终世界坐标，也不参与板型/装载区域最终判断。
    """

    def __init__(self, calibration: SensorCalibrationService | None = None, depth_window: int = 5):
        self.calibration = calibration or SensorCalibrationService()
        self.depth_window = int(depth_window)

    @staticmethod
    def _rough_xyz(radar_result: Mapping[str, Any], pid: str):
        raw = (radar_result.get("world_points") or {}).get(pid)
        if isinstance(raw, Mapping):
            return [float(raw.get("x", raw.get("x_mm", 0))), float(raw.get("y", raw.get("y_mm", 0))), float(raw.get("z", raw.get("z_mm", 0)))]
        if isinstance(raw, (list, tuple)) and len(raw) >= 3:
            return [float(raw[0]), float(raw[1]), float(raw[2])]
        return None

    @staticmethod
    def _distance(a, b):
        return sqrt(sum((float(a[i])-float(b[i]))**2 for i in range(3)))

    def convert(
        self,
        image_points: Dict[str, Any],
        capture_meta: Dict[str, Any],
        radar_result: Dict[str, Any] | None = None,
        depth_mode: str = "raw",
    ) -> Dict[str, Any]:
        radar_result = radar_result or {}
        ids = [str(x) for x in (radar_result.get("corner_ids") or image_points.keys())]
        if len(ids) not in {4, 6}:
            raise RuntimeError(f"角点最终转换只支持 4 或 6 点，当前：{ids}")

        world_points: Dict[str, Dict[str, float]] = {}
        details = {}
        for pid in ids:
            det = image_points.get(pid)
            meta = capture_meta.get(pid)
            if not isinstance(det, Mapping):
                raise RuntimeError(f"缺少 {pid} 的角点模型像素结果")
            if not isinstance(meta, Mapping):
                raise RuntimeError(f"缺少 {pid} 的相机采集元数据")

            camera_id = str(meta.get("camera_id") or "")
            depth_path = str(meta.get("depth_path") or "")
            camera_world_pose = meta.get("camera_world_pose") or {}
            if not camera_id or not depth_path:
                raise RuntimeError(f"{pid} 缺少 camera_id 或同步 depth_path")

            u = float(det.get("x", det.get("u")))
            v = float(det.get("y", det.get("v")))
            # Offline examples and some camera profiles expose RGB and aligned
            # Z16 at different resolutions.  Map the model pixel into the
            # depth/intrinsic raster before depth sampling and back-projection.
            source_size = det.get("image_size_px")
            depth_size = None
            if isinstance(source_size, (list, tuple)) and len(source_size) >= 2:
                try:
                    import cv2
                    depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
                    if depth_image is not None:
                        depth_size = [int(depth_image.shape[1]), int(depth_image.shape[0])]
                        if float(source_size[0]) > 0 and float(source_size[1]) > 0:
                            u = u * depth_size[0] / float(source_size[0])
                            v = v * depth_size[1] / float(source_size[1])
                except Exception:
                    depth_size = None
            depth_raw = meta.get("depth_value")
            if depth_raw in (None, ""):
                depth_raw = self.calibration.sample_depth(depth_path, u, v, self.depth_window)

            conv = self.calibration.pixel_depth_to_world(
                camera_id=camera_id,
                u=u,
                v=v,
                depth_value=float(depth_raw),
                camera_world_pose=camera_world_pose,
                depth_mode=str(meta.get("depth_mode") or depth_mode),
            )
            xyz = conv["world_xyz_mm"]
            world_points[pid] = {"x": xyz[0], "y": xyz[1], "z": xyz[2]}

            rough = self._rough_xyz(radar_result, pid)
            conv["radar_coarse_world_xyz_mm"] = rough
            conv["camera_vs_radar_coarse_error_mm"] = None if rough is None else round(self._distance(xyz, rough), 3)
            conv["result_image"] = det.get("result_image")
            conv["model_pixel_original"] = [float(det.get("x", det.get("u"))), float(det.get("y", det.get("v")))]
            conv["model_image_size_px"] = source_size
            conv["depth_image_size_px"] = depth_size
            conv["depth_pixel_used"] = [u, v]
            details[pid] = conv

        return {
            "success": True,
            "source": "camera_pixel_depth_world",
            "coordinate_frame": "world",
            "coordinate_unit": "mm",
            "corner_ids": ids,
            "world_points": world_points,
            "corner_points_xyz_mm": [[world_points[p]["x"], world_points[p]["y"], world_points[p]["z"]] for p in ids],
            "details": details,
            "radar_used_for_final_geometry": False,
            "message": f"{len(ids)} 个视觉角点已由像素+深度+机械臂挂载相机动态外参转换到 WORLD",
        }
