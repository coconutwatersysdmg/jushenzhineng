# -*- coding: utf-8 -*-
from __future__ import annotations

"""传感器标定与像素/深度 -> WORLD 坐标转换。

重要约定
--------
用户提供的 coordinate_config.json 中相机 T_world_sensor 是“静态占位外参”。
当前系统的唯一活动相机 CAM_PICK 安装在 PICK_ARM 上，因此机械臂运动后不能继续把该静态矩阵
当作真实 T_world_camera 使用。

实际运行采用：
    T_world_camera(t) = T_world_robot(t) @ T_robot_camera

其中：
- T_world_robot(t)：PLC/机械臂实时位姿；
- T_robot_camera：相机相对机械臂的安装外参（device_config.json mount_pose）。

用户上传的 camera_intrinsic.json 直接作为相机内参来源；
用户上传的 coordinate_config.json 中 lidar 的 T_world_sensor 用于固定雷达转 WORLD。
相机静态 T_world_sensor 仅保留为标定参考/联调检查，不参与机械臂运动后的最终点计算。
"""

import json
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import numpy as np

from core.geometry import Pose6D, pose_matrix

PROJECT_ROOT = Path(__file__).resolve().parents[1]
INTRINSIC_FILE = PROJECT_ROOT / "config" / "sensor_coordinate_config" / "camera_intrinsic.json"
COORDINATE_FILE = PROJECT_ROOT / "config" / "sensor_coordinate_config" / "coordinate_config.json"


class SensorCalibrationService:
    ACTIVE_CAMERA_TO_CONFIG = {
        # 唯一活动相机：随PICK_ARM完成插孔、角点、放置与复检。
        "CAM_PICK": "hole_camera",
    }

    def __init__(self, intrinsic_file: Path = INTRINSIC_FILE, coordinate_file: Path = COORDINATE_FILE):
        self.intrinsic_file = Path(intrinsic_file)
        self.coordinate_file = Path(coordinate_file)
        self.intrinsic_cfg = self._read_json(self.intrinsic_file)
        self.coordinate_cfg = self._read_json(self.coordinate_file)

    @staticmethod
    def _read_json(path: Path) -> Dict[str, Any]:
        if not path.is_file():
            raise FileNotFoundError(path)
        return json.loads(path.read_text(encoding="utf-8"))

    @property
    def world_frame(self) -> Dict[str, Any]:
        return dict(self.coordinate_cfg.get("world_frame") or {})

    def camera_config_name(self, camera_id: str) -> str:
        if camera_id not in self.ACTIVE_CAMERA_TO_CONFIG:
            raise KeyError(f"未配置活动相机：{camera_id}")
        return self.ACTIVE_CAMERA_TO_CONFIG[camera_id]

    def camera_intrinsic(self, camera_id: str) -> Dict[str, Any]:
        name = self.camera_config_name(camera_id)
        cameras = self.intrinsic_cfg.get("cameras") or {}
        if name not in cameras:
            raise KeyError(f"camera_intrinsic.json 中不存在：{name}")
        cfg = dict(cameras[name])
        K = np.asarray(cfg.get("K"), dtype=np.float64)
        if K.shape != (3, 3):
            raise ValueError(f"{name}.K 必须为 3x3")
        cfg["camera_config_name"] = name
        cfg["fx"] = float(K[0, 0])
        cfg["fy"] = float(K[1, 1])
        cfg["cx"] = float(K[0, 2])
        cfg["cy"] = float(K[1, 2])
        return cfg

    def fixed_world_matrix(self, sensor_name: str) -> np.ndarray:
        transforms = self.coordinate_cfg.get("transforms") or {}
        if sensor_name not in transforms:
            raise KeyError(f"coordinate_config.json 中不存在：{sensor_name}")
        T = np.asarray(transforms[sensor_name].get("T_world_sensor"), dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError(f"{sensor_name}.T_world_sensor 必须为 4x4")
        return T

    def camera_reference_world_matrix(self, camera_id: str) -> np.ndarray:
        """用户上传的静态相机外参，仅作为参考，不作为运动相机最终外参。"""
        return self.fixed_world_matrix(self.camera_config_name(camera_id))

    @staticmethod
    def dynamic_camera_world_matrix(camera_world_pose: Mapping[str, Any]) -> np.ndarray:
        """由机械臂实时位姿×安装外参得到的相机 WORLD 位姿生成 4x4 矩阵。"""
        return np.asarray(pose_matrix(Pose6D.from_any(camera_world_pose)), dtype=np.float64)

    @staticmethod
    def transform_point(T_world_sensor: np.ndarray, point_sensor_mm: Sequence[float]) -> np.ndarray:
        p = np.asarray(point_sensor_mm, dtype=np.float64)
        if p.shape != (3,):
            raise ValueError("point_sensor_mm 必须为 [x,y,z]")
        T = np.asarray(T_world_sensor, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError("T_world_sensor 必须为 4x4")
        q = T @ np.asarray([p[0], p[1], p[2], 1.0], dtype=np.float64)
        if abs(q[3]) < 1e-12:
            raise ValueError("齐次坐标转换失败")
        return q[:3] / q[3]

    def lidar_point_to_world(self, point_lidar_mm: Sequence[float]) -> np.ndarray:
        return self.transform_point(self.fixed_world_matrix("lidar"), point_lidar_mm)

    @staticmethod
    def _depth_to_mm(depth_value: float, intrinsic: Mapping[str, Any], depth_mode: str) -> float:
        value = float(depth_value)
        mode = str(depth_mode or "raw").lower()
        if mode == "mm":
            return value
        if mode == "m":
            return value * 1000.0
        if mode == "raw":
            scale = float(intrinsic.get("depth_scale", 0.001))
            return value * scale * 1000.0
        raise ValueError("depth_mode 仅支持 raw/mm/m")

    @staticmethod
    def _undistorted_normalized_xy(u: float, v: float, intrinsic: Mapping[str, Any]) -> tuple[float, float]:
        K = np.asarray(intrinsic["K"], dtype=np.float64)
        distortion = np.asarray(intrinsic.get("distortion") or [0, 0, 0, 0, 0], dtype=np.float64).reshape(-1)
        if distortion.size and np.max(np.abs(distortion)) > 1e-12:
            try:
                import cv2
                uv = np.asarray([[[float(u), float(v)]]], dtype=np.float64)
                norm = cv2.undistortPoints(uv, K, distortion)
                return float(norm[0, 0, 0]), float(norm[0, 0, 1])
            except Exception:
                # 如果 OpenCV 不可用，退回针孔模型；结果中会由调用方保留标定状态。
                pass
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        if abs(fx) < 1e-12 or abs(fy) < 1e-12:
            raise ValueError("fx/fy 无效")
        return (float(u) - cx) / fx, (float(v) - cy) / fy

    def pixel_depth_to_camera_xyz(
        self,
        camera_id: str,
        u: float,
        v: float,
        depth_value: float,
        depth_mode: str = "raw",
    ) -> np.ndarray:
        intrinsic = self.camera_intrinsic(camera_id)
        z = self._depth_to_mm(depth_value, intrinsic, depth_mode)
        if z <= 0:
            raise ValueError("角点深度必须 > 0")
        xn, yn = self._undistorted_normalized_xy(u, v, intrinsic)
        return np.asarray([xn * z, yn * z, z], dtype=np.float64)

    def pixel_depth_to_world(
        self,
        camera_id: str,
        u: float,
        v: float,
        depth_value: float,
        camera_world_pose: Mapping[str, Any],
        depth_mode: str = "raw",
    ) -> Dict[str, Any]:
        p_cam = self.pixel_depth_to_camera_xyz(camera_id, u, v, depth_value, depth_mode)
        T_world_camera = self.dynamic_camera_world_matrix(camera_world_pose)
        p_world = self.transform_point(T_world_camera, p_cam)
        return {
            "camera_id": camera_id,
            "camera_config_name": self.camera_config_name(camera_id),
            "pixel_uv": [float(u), float(v)],
            "depth_value": float(depth_value),
            "depth_mode": str(depth_mode),
            "camera_xyz_mm": [float(x) for x in p_cam],
            "world_xyz_mm": [float(x) for x in p_world],
            "T_world_camera": T_world_camera.tolist(),
        }

    @staticmethod
    def sample_depth(depth_path: str | Path, u: float, v: float, window: int = 5) -> float:
        """在角点附近取有效深度中位数，避免单像素孔洞。返回原始深度值。"""
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("读取角点深度图需要 opencv-python") from exc
        path = Path(depth_path)
        from utils.cv_io import read_image
        depth = read_image(path, cv2.IMREAD_UNCHANGED)
        if depth is None:
            raise RuntimeError(f"无法读取深度图：{path}")
        if depth.ndim > 2:
            depth = depth[..., 0]
        x, y = int(round(float(u))), int(round(float(v)))
        radius = max(0, int(window) // 2)
        x0, x1 = max(0, x-radius), min(depth.shape[1], x+radius+1)
        y0, y1 = max(0, y-radius), min(depth.shape[0], y+radius+1)
        patch = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64).reshape(-1)
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            raise RuntimeError(f"角点 ({x},{y}) 周围没有有效深度：{path}")
        return float(np.median(valid))

    def diagnostic_summary(self) -> Dict[str, Any]:
        return {
            "world_frame": self.world_frame,
            "active_camera_mapping": dict(self.ACTIVE_CAMERA_TO_CONFIG),
            "intrinsic_parameter_status": self.intrinsic_cfg.get("parameter_status"),
            "coordinate_parameter_status": self.coordinate_cfg.get("parameter_status"),
            "moving_camera_rule": "T_world_camera(t)=T_world_robot(t)@T_robot_camera",
            "radar_rule": "P_world=T_world_lidar@P_lidar",
        }
