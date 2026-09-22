# -*- coding: utf-8 -*-
"""Adapter for the laboratory traditional OpenCV fork-hole algorithm."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from algorithm_modules.lab.fork_hole_lab.fork_hole_locator import locate_fork_holes


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INTRINSIC_PATH = PROJECT_ROOT / "config" / "sensor_coordinate_config" / "camera_intrinsic.json"


class TraditionalForkHoleService:
    def __init__(self, dark_threshold: int | None = None) -> None:
        # None = 使用 stable 算法自适应阈值，与 zip 默认行为一致
        self.dark_threshold = None if dark_threshold is None else int(dark_threshold)

    @staticmethod
    def _fallback_intrinsics() -> dict[str, float]:
        payload = json.loads(DEFAULT_INTRINSIC_PATH.read_text(encoding="utf-8"))
        camera = (payload.get("cameras") or {}).get("hole_camera") or {}
        matrix = camera.get("K") or []
        return {
            "fx": float(matrix[0][0]),
            "fy": float(matrix[1][1]),
            "cx": float(matrix[0][2]),
            "cy": float(matrix[1][2]),
        }

    @classmethod
    def _intrinsics(cls, capture: Mapping[str, Any]) -> dict[str, float]:
        raw = capture.get("intrinsics")
        if isinstance(raw, Mapping):
            try:
                cx = raw.get("cx", raw.get("ppx"))
                cy = raw.get("cy", raw.get("ppy"))
                return {
                    "fx": float(raw["fx"]),
                    "fy": float(raw["fy"]),
                    "cx": float(cx),
                    "cy": float(cy),
                }
            except (KeyError, TypeError, ValueError):
                pass
        if isinstance(raw, (list, tuple)) and len(raw) >= 4:
            try:
                return {
                    "fx": float(raw[0]),
                    "fy": float(raw[1]),
                    "cx": float(raw[2]),
                    "cy": float(raw[3]),
                }
            except (TypeError, ValueError):
                pass
        return cls._fallback_intrinsics()

    def recognize_capture(self, capture: Mapping[str, Any]) -> dict[str, Any]:
        import cv2
        import numpy as np

        from utils.cv_io import read_image

        rgb_path = Path(str(capture.get("rgb_path") or "")).expanduser().resolve()
        depth_path = Path(str(capture.get("depth_path") or "")).expanduser().resolve()
        rgb = read_image(rgb_path, cv2.IMREAD_COLOR)
        depth = read_image(depth_path, cv2.IMREAD_UNCHANGED)
        if rgb is None or depth is None:
            raise RuntimeError("传统插孔算法需要有效的 RGB 和对齐深度图")
        depth_array = np.asarray(depth)
        if depth_array.ndim == 3:
            depth_array = depth_array[:, :, 0]
        depth_mm = depth_array.astype(np.float32) * float(capture.get("depth_scale_mm", 1.0) or 1.0)
        raw = locate_fork_holes(
            rgb,
            depth_mm,
            self._intrinsics(capture),
            dark_threshold=self.dark_threshold,
        )
        left = raw.get("left_hole_xyz_mm")
        right = raw.get("right_hole_xyz_mm")
        success = left is not None and right is not None
        return {
            "success": success,
            "algorithm": "traditional_opencv_fork_hole",
            "source": "traditional_fork_hole_rgbd",
            "recognition_source": "traditional_fork_hole_rgbd",
            "coordinate_frame": "camera",
            "coordinate_unit": "mm",
            "left_xyz_mm": left,
            "right_xyz_mm": right,
            "left_hole_xyz_mm": left,
            "right_hole_xyz_mm": right,
            "result_image_path": str(rgb_path),
            "rgb_path": str(rgb_path),
            "depth_path": str(depth_path),
            "camera_intrinsics": self._intrinsics(capture),
            "dark_threshold": self.dark_threshold,
            "message": (
                "传统 OpenCV 算法已识别左右插孔并输出相机坐标"
                if success
                else "传统 OpenCV 算法未同时识别到两个有效插孔"
            ),
        }
