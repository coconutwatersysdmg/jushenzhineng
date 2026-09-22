# -*- coding: utf-8 -*-
"""Arm-mounted Intel RealSense D435i adapter.

配置与采集参数迁自 Automatic loading system：
config.ini [camera] + config/camera_extrinsic.json。
优先实拍；调试输入中若已指定 RGB/Depth 文件则优先用文件。
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from core.digital_twin_state import DigitalTwinState
from devices.base import ArmCameraAdapter
from utils.cv_io import write_image

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class RealArmCameraAdapter(ArmCameraAdapter):
    def __init__(self, twin: DigitalTwinState, config: Mapping[str, Any] | None = None):
        self.twin = twin
        self.config = dict(config or {})
        self.demo_enabled = False
        self.inputs = {"CAM_PICK": {"rgb": "", "depth": ""}}
        self.tagged_images: dict[str, str] = {}
        self.tagged_depths: dict[str, str] = {}

        self.backend = str(self.config.get("backend") or "realsense")
        self.serial = str(self.config.get("serial") or "").strip() or None
        self.color_width = int(self.config.get("color_width", 1280))
        self.color_height = int(self.config.get("color_height", 720))
        self.depth_width = int(self.config.get("depth_width", 1280))
        self.depth_height = int(self.config.get("depth_height", 720))
        self.fps = int(self.config.get("fps", 30))
        self._depth_scale_mm = float(self.config.get("depth_unit_mm", 1.0))

        capture_dir = Path(str(self.config.get("capture_dir") or "workdir/camera_captures"))
        self.capture_dir = capture_dir if capture_dir.is_absolute() else (PROJECT_ROOT / capture_dir)

        extrinsic = Path(str(self.config.get("extrinsic_file") or "config/camera_extrinsic.json"))
        self.extrinsic_file = extrinsic if extrinsic.is_absolute() else (PROJECT_ROOT / extrinsic)

        self._rs = None
        self._pipeline = None
        self._align = None
        self._profile = None
        self._last_error = ""

    def set_demo_enabled(self, enabled: bool):
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

    def connect(self) -> dict:
        if self._pipeline is not None:
            self.twin.update_camera("CAM_PICK", status="ONLINE", task="D435I_CONNECTED")
            return {"success": True, "device_id": "CAM_PICK", "message": "D435i 已连接"}
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            self._last_error = "缺少 pyrealsense2，请安装 Intel RealSense SDK / pip install pyrealsense2"
            self.twin.update_camera("CAM_PICK", status="OFFLINE", task="NO_SDK")
            return {
                "success": False,
                "device_id": "CAM_PICK",
                "message": self._last_error,
                "error": str(exc),
            }

        self._rs = rs
        config = rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(
            rs.stream.color, self.color_width, self.color_height, rs.format.bgr8, self.fps
        )
        config.enable_stream(
            rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16, self.fps
        )
        pipeline = rs.pipeline()
        try:
            self._profile = pipeline.start(config)
        except Exception as exc:
            self._last_error = f"D435i 连接失败：{exc}"
            self._pipeline = self._profile = self._align = None
            self.twin.update_camera("CAM_PICK", status="OFFLINE", task="CONNECT_FAILED")
            return {"success": False, "device_id": "CAM_PICK", "message": self._last_error}

        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color)
        try:
            sensor = self._profile.get_device().first_depth_sensor()
            self._depth_scale_mm = float(sensor.get_depth_scale()) * 1000.0
        except Exception:
            pass
        self._last_error = ""
        self.twin.update_camera("CAM_PICK", status="ONLINE", task="D435I_CONNECTED")
        return {
            "success": True,
            "device_id": "CAM_PICK",
            "message": "D435i 已连接",
            "extrinsic_file": str(self.extrinsic_file),
            "depth_scale_mm": self._depth_scale_mm,
        }

    def disconnect(self):
        pipeline = self._pipeline
        was_started = self._profile is not None
        self._pipeline = self._profile = self._align = None
        if pipeline is not None and was_started:
            try:
                pipeline.stop()
            except Exception:
                pass

    def _capture_realsense(self, camera_id: str, tag: str, need_depth: bool) -> dict:
        linked = self.connect()
        if not linked.get("success"):
            return {
                "success": False,
                "camera_id": camera_id,
                "tag": tag,
                "message": linked.get("message") or self._last_error or "D435i 未连接",
            }
        assert self._pipeline is not None and self._align is not None
        try:
            frames = self._align.process(self._pipeline.wait_for_frames(5000))
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color:
                raise RuntimeError("未获取到彩色帧")
            if need_depth and not depth:
                raise RuntimeError("未获取到深度帧")

            import cv2

            color_img = np.asanyarray(color.get_data())
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            safe_tag = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(tag or "capture"))
            self.capture_dir.mkdir(parents=True, exist_ok=True)
            rgb_path = self.capture_dir / f"{camera_id}_{safe_tag}_{stamp}_rgb.jpg"
            if not write_image(rgb_path, color_img, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError(f"RGB 写入失败：{rgb_path}")

            out = {
                "success": True,
                "camera_id": camera_id,
                "tag": tag,
                "rgb_path": str(rgb_path.resolve()),
                "image_path": str(rgb_path.resolve()),
                "source": "realsense_d435i",
                "camera_world_pose": self.twin.camera_world_pose(camera_id),
                "extrinsic_file": str(self.extrinsic_file),
                "message": f"D435i 实拍完成：{rgb_path.name}",
            }
            try:
                intr = color.profile.as_video_stream_profile().intrinsics
                out["intrinsics"] = {
                    "fx": float(intr.fx),
                    "fy": float(intr.fy),
                    "cx": float(intr.ppx),
                    "cy": float(intr.ppy),
                    "ppx": float(intr.ppx),
                    "ppy": float(intr.ppy),
                }
            except Exception:
                pass
            if need_depth and depth is not None:
                depth_img = np.asanyarray(depth.get_data())
                depth_path = self.capture_dir / f"{camera_id}_{safe_tag}_{stamp}_depth.png"
                if not write_image(depth_path, depth_img):
                    raise RuntimeError(f"Depth 写入失败：{depth_path}")
                out["depth_path"] = str(depth_path.resolve())
                out["depth_scale_mm"] = self._depth_scale_mm

            twin_kwargs = {"task": f"REALSENSE:{tag}", "status": "CAPTURE"}
            if out.get("intrinsics"):
                twin_kwargs["intrinsics"] = dict(out["intrinsics"])
            self.twin.update_camera(camera_id, **twin_kwargs)
            return out
        except Exception as exc:
            self._last_error = str(exc)
            self.twin.update_camera(camera_id, status="FAILED", task=f"REALSENSE_FAIL:{tag}")
            return {
                "success": False,
                "camera_id": camera_id,
                "tag": tag,
                "message": f"D435i 采集失败：{exc}",
            }

    def _manual_or_live(self, camera_id: str, tag: str, need_depth: bool) -> dict:
        rgb = self._file(self.tagged_images.get(str(tag), "")) or self._file(
            self.inputs.get(camera_id, {}).get("rgb")
        )
        depth = self._file(self.tagged_depths.get(str(tag), "")) or self._file(
            self.inputs.get(camera_id, {}).get("depth")
        )
        if rgb and (not need_depth or depth):
            self.twin.update_camera(camera_id, task=f"MANUAL:{tag}", status="CAPTURE")
            out = {
                "success": True,
                "camera_id": camera_id,
                "tag": tag,
                "rgb_path": str(rgb),
                "image_path": str(rgb),
                "source": "manual_file",
                "camera_world_pose": self.twin.camera_world_pose(camera_id),
                "message": "使用调试输入中的相机文件",
            }
            if depth:
                out["depth_path"] = str(depth)
            return out

        if str(self.backend).lower() in {"realsense", "realsense_d435i", "d435i"}:
            return self._capture_realsense(camera_id, tag, need_depth=need_depth)

        self.twin.update_camera(camera_id, status="FAILED", task=f"NO_CAMERA:{tag}")
        return {
            "success": False,
            "camera_id": camera_id,
            "tag": tag,
            "message": (
                self._last_error
                or (
                    "无法采集：请安装 pyrealsense2 并连接 D435i，"
                    "或在调试输入中指定 RGB/Depth 文件，或改回 DEVICE_MODE=mock。"
                )
            ),
        }

    def capture_rgbd(self, camera_id: str, tag: str = "") -> dict:
        return self._manual_or_live(camera_id, tag, need_depth=True)

    def capture_rgb(self, camera_id: str, tag: str = "") -> dict:
        return self._manual_or_live(camera_id, tag, need_depth=False)
