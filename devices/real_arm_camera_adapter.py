# -*- coding: utf-8 -*-
"""Arm-mounted camera adapter stub for Intel RealSense D435i.

The Livox / PLC zip packages do not include a camera SDK.  Install
``pyrealsense2`` and fill capture_rgbd() before enabling device_mode=real for
production imaging.  Until then this adapter returns a clear failure instead of
pretending to have camera frames.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from devices.base import ArmCameraAdapter

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class RealArmCameraAdapter(ArmCameraAdapter):
    def __init__(self, twin: DigitalTwinState, config: Mapping[str, Any] | None = None):
        self.twin = twin
        self.config = dict(config or {})
        self.demo_enabled = False
        self.inputs = {"CAM_PICK": {"rgb": "", "depth": ""}}
        self.tagged_images: dict[str, str] = {}
        self.tagged_depths: dict[str, str] = {}
        self.backend = str(self.config.get("backend") or "realsense_d435i")

    def set_demo_enabled(self, enabled: bool):
        # Real mode should not silently synthesize camera frames.
        self.demo_enabled = False

    def set_rgbd(self, rgb: str, depth: str):
        self.inputs["CAM_PICK"] = {"rgb": str(rgb or ""), "depth": str(depth or "")}

    def set_tagged_image(self, tag: str, path: str):
        self.tagged_images[str(tag)] = str(path or "")

    def set_tagged_depth(self, tag: str, path: str):
        self.tagged_depths[str(tag)] = str(path or "")

    @staticmethod
    def _file(path: str | Path | None):
        p = Path(str(path or "")).expanduser()
        return p.resolve() if p.is_file() else None

    def _manual_or_fail(self, camera_id: str, tag: str, need_depth: bool) -> dict:
        rgb = self._file(self.tagged_images.get(str(tag), "")) or self._file(self.inputs.get(camera_id, {}).get("rgb"))
        depth = self._file(self.tagged_depths.get(str(tag), "")) or self._file(self.inputs.get(camera_id, {}).get("depth"))
        if rgb and (not need_depth or depth):
            self.twin.update_camera(camera_id, task=f"MANUAL:{tag}", status="CAPTURE")
            out = {
                "success": True,
                "camera_id": camera_id,
                "tag": tag,
                "rgb_path": str(rgb),
                "source": "manual_file",
                "message": "使用调试输入中的相机文件（RealSense SDK 尚未接入）",
            }
            if depth:
                out["depth_path"] = str(depth)
            return out
        self.twin.update_camera(camera_id, status="FAILED", task=f"NO_SDK:{tag}")
        return {
            "success": False,
            "camera_id": camera_id,
            "tag": tag,
            "message": (
                "真机相机适配器尚未接入 Intel RealSense D435i SDK（pyrealsense2）。"
                "两个对接压缩包不含相机驱动。请安装 RealSense SDK 后实现采集，"
                "或在调试输入中手动指定 RGB/Depth 文件，或改回 device_mode=mock。"
            ),
        }

    def capture_rgbd(self, camera_id: str, tag: str = "") -> dict:
        return self._manual_or_fail(camera_id, tag, need_depth=True)

    def capture_rgb(self, camera_id: str, tag: str = "") -> dict:
        return self._manual_or_fail(camera_id, tag, need_depth=False)
