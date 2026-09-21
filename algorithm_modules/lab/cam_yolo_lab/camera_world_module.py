from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Mapping, Sequence


class CameraWorldTransform:
    def __init__(
        self,
        t_e_from_c: Sequence[Sequence[float]],
        plc_reference_r_deg: float = -80.0,
        r_axis_center_offset_mm: Sequence[float] = (104.0, 166.0, 270.0),
    ):
        if len(t_e_from_c) != 4 or any(len(row) != 4 for row in t_e_from_c):
            raise ValueError("T_E<-C must be a 4x4 matrix")
        if len(r_axis_center_offset_mm) != 3:
            raise ValueError("R-axis center offset must contain x, y and z")
        self._t_e_from_c = tuple(tuple(float(v) for v in row) for row in t_e_from_c)
        self._plc_reference_r_deg = float(plc_reference_r_deg)
        self._r_axis_center_offset_mm = tuple(float(v) for v in r_axis_center_offset_mm)

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraWorldTransform":
        with Path(path).open("r", encoding="utf-8") as f:
            cfg = json.load(f)
        return cls(
            cfg["t_e_from_c"],
            cfg["plc_reference_r_deg"],
            cfg["r_axis_center_offset_mm"],
        )

    def world_pose_from_plc(self, plc_pose: Mapping[str, float]):
        try:
            x, y, z, r = (float(plc_pose[k]) for k in ("x", "y", "z", "r"))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("PLC pose requires numeric x, y, z and r") from exc
        theta = math.radians(r - self._plc_reference_r_deg)
        c, s = math.cos(theta), math.sin(theta)
        cx, cy, cz = self._r_axis_center_offset_mm
        return (
            (c, -s, 0.0, x + cx),
            (s, c, 0.0, y + cy),
            (0.0, 0.0, 1.0, z + cz),
            (0.0, 0.0, 0.0, 1.0),
        )

    def camera_pose_from_plc(self, plc_pose: Mapping[str, float]):
        """Return WORLD_T_CAMERA for the supplied PLC pose."""
        return _matmul4(self.world_pose_from_plc(plc_pose), self._t_e_from_c)

    def plc_xy_for_camera_axis_target(
        self,
        target_world_xyz: Sequence[float],
        plc_pose: Mapping[str, float],
    ) -> dict[str, float]:
        """Solve the PLC X/Y correction that puts a WORLD target on the optical axis.

        The laboratory camera is mounted on the R-axis.  X/Y are therefore
        solved from the camera origin and its local Z axis while keeping the
        requested Z/R fixed, matching the archived laboratory workflow.
        """
        if len(target_world_xyz) != 3:
            raise ValueError("WORLD target must contain x, y and z")
        pose = {"x": 0.0, "y": 0.0, "z": plc_pose["z"], "r": plc_pose["r"]}
        t_world_from_camera = self.camera_pose_from_plc(pose)
        origin = tuple(float(t_world_from_camera[row][3]) for row in range(3))
        axis = tuple(float(t_world_from_camera[row][2]) for row in range(3))
        if abs(axis[2]) < 1e-9:
            raise ValueError("相机光轴 Z 分量过小，无法反算 PLC 坐标")
        distance = (float(target_world_xyz[2]) - origin[2]) / axis[2]
        if distance < 0:
            raise ValueError("目标位于相机光轴后方，无法反算 PLC 坐标")
        desired_origin = (
            float(target_world_xyz[0]) - distance * axis[0],
            float(target_world_xyz[1]) - distance * axis[1],
        )
        return {
            "X": desired_origin[0] - origin[0],
            "Y": desired_origin[1] - origin[1],
        }

    def camera_to_world(self, point_c: Sequence[float], plc_pose: Mapping[str, float]):
        if len(point_c) != 3:
            raise ValueError("camera point must contain x, y and z")
        p_e = _transform_point(self._t_e_from_c, point_c)
        return _transform_point(self.world_pose_from_plc(plc_pose), p_e)


class D435iCamera:
    """Minimal RealSense D435i RGB-D capture with depth aligned to color."""

    def __init__(self, width=1280, height=720, fps=30, serial=None):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.serial = serial
        self._pipeline = None
        self._align = None
        self._profile = None
        self._depth_scale_mm = 1.0

    def connect(self):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("缺少 pyrealsense2，请先安装 Intel RealSense SDK Python 组件") from exc
        config = rs.config()
        if self.serial:
            config.enable_device(str(self.serial))
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self._pipeline = rs.pipeline()
        self._profile = self._pipeline.start(config)
        self._align = rs.align(rs.stream.color)
        sensor = self._profile.get_device().first_depth_sensor()
        self._depth_scale_mm = float(sensor.get_depth_scale()) * 1000.0
        return self

    def disconnect(self):
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            finally:
                self._pipeline = None
                self._align = None
                self._profile = None

    def capture(self, timeout_ms=5000):
        if self._pipeline is None or self._align is None:
            raise RuntimeError("D435i 尚未连接")
        import numpy as np

        frames = self._align.process(self._pipeline.wait_for_frames(int(timeout_ms)))
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("未获取到完整的 RGB-D 帧")
        intr = color.profile.as_video_stream_profile().intrinsics
        return {
            "color_array": np.asanyarray(color.get_data()),
            "depth_array": np.asanyarray(depth.get_data()),
            "intrinsics": (float(intr.fx), float(intr.fy), float(intr.ppx), float(intr.ppy)),
            "depth_scale_mm": float(self._depth_scale_mm),
        }

    def __enter__(self):
        return self.connect()

    def __exit__(self, exc_type, exc, tb):
        self.disconnect()


class YoloD435iWorldLocalizer:
    def __init__(
        self,
        model_path: str | Path | None = None,
        extrinsic_path: str | Path | None = None,
        confidence: float = 0.30,
        imgsz: int = 960,
        model=None,
        transform: CameraWorldTransform | None = None,
    ):
        base = Path(__file__).resolve().parent
        self.model_path = Path(model_path) if model_path else base / "weights" / "best.pt"
        self.extrinsic_path = Path(extrinsic_path) if extrinsic_path else base / "camera_extrinsic.json"
        self.confidence = float(confidence)
        self.imgsz = int(imgsz)
        self._model = model
        self.transform = transform or CameraWorldTransform.from_json(self.extrinsic_path)

    def load_model(self):
        if self._model is not None:
            return self._model
        if not self.model_path.exists():
            raise FileNotFoundError(f"找不到 YOLO 权重：{self.model_path}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("缺少 ultralytics，请先安装 requirements.txt") from exc
        self._model = YOLO(str(self.model_path))
        return self._model

    def _device(self):
        try:
            import torch
            return 0 if torch.cuda.is_available() else "cpu"
        except ImportError:
            return "cpu"

    def _detect_box_centers(self, color):
        model = self.load_model()
        results = model.predict(
            source=color,
            conf=self.confidence,
            imgsz=self.imgsz,
            device=self._device(),
            verbose=False,
        )
        candidates = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None or getattr(boxes, "xyxy", None) is None:
                continue
            xyxy = _to_numpy(boxes.xyxy)
            conf = _to_numpy(getattr(boxes, "conf", None))
            for i, box in enumerate(xyxy):
                if len(box) < 4:
                    continue
                score = float(conf[i]) if conf is not None and len(conf) > i else 1.0
                if score < self.confidence:
                    continue
                x1, y1, x2, y2 = (float(v) for v in box[:4])
                candidates.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, score))
        candidates.sort(key=lambda item: item[2], reverse=True)
        if len(candidates) < 2:
            raise RuntimeError(f"YOLO 有效角点不足 2 个，当前为 {len(candidates)} 个")
        return candidates[:2]

    @staticmethod
    def _depth_to_camera(frame: Mapping, pixel_x: float, pixel_y: float):
        import numpy as np

        depth = np.asarray(frame["depth_array"])
        fx, fy, ppx, ppy = _normalise_intrinsics(frame["intrinsics"])
        h, w = depth.shape[:2]
        cx, cy = int(round(pixel_x)), int(round(pixel_y))
        radius = 2
        x0, x1 = max(0, cx - radius), min(w, cx + radius + 1)
        y0, y1 = max(0, cy - radius), min(h, cy + radius + 1)
        values = depth[y0:y1, x0:x1].astype(float).reshape(-1)
        values = values[np.isfinite(values) & (values > 0)]
        if values.size == 0:
            raise RuntimeError(f"角点 ({pixel_x:.1f}, {pixel_y:.1f}) 周围没有有效深度")
        depth_mm = float(np.median(values)) * float(frame.get("depth_scale_mm", 1.0))
        if not math.isfinite(depth_mm) or depth_mm <= 0:
            raise RuntimeError("角点深度无效")
        return (
            (float(pixel_x) - ppx) * depth_mm / fx,
            (float(pixel_y) - ppy) * depth_mm / fy,
            depth_mm,
        )

    def detect_pair(self, frame: Mapping, plc_pose: Mapping[str, float], pair_names=("P3", "P4")):
        if len(pair_names) != 2:
            raise ValueError("pair_names must contain exactly two point names")
        color = frame.get("color_array")
        depth = frame.get("depth_array")
        if color is None or depth is None:
            raise ValueError("RGB-D frame must contain color_array and depth_array")
        candidates = self._detect_box_centers(color)
        points = []
        for u, v, _ in candidates:
            camera_xyz = self._depth_to_camera(frame, u, v)
            world_xyz = self.transform.camera_to_world(camera_xyz, plc_pose)
            points.append((u, [float(x) for x in world_xyz]))
        points.sort(key=lambda item: item[0])
        return {pair_names[0]: points[0][1], pair_names[1]: points[1][1]}

    def locate_four_corners(
        self,
        p3p4_frame: Mapping,
        p3p4_plc_pose: Mapping[str, float],
        p1p2_frame: Mapping,
        p1p2_plc_pose: Mapping[str, float],
    ):
        p3p4 = self.detect_pair(p3p4_frame, p3p4_plc_pose, ("P3", "P4"))
        p1p2 = self.detect_pair(p1p2_frame, p1p2_plc_pose, ("P1", "P2"))
        return {
            "P1": p1p2["P1"],
            "P2": p1p2["P2"],
            "P3": p3p4["P3"],
            "P4": p3p4["P4"],
        }


def _transform_point(matrix, point: Sequence[float]):
    x, y, z = (float(v) for v in point)
    return tuple(
        matrix[row][0] * x
        + matrix[row][1] * y
        + matrix[row][2] * z
        + matrix[row][3]
        for row in range(3)
    )


def _matmul4(a, b):
    return [
        [sum(float(a[i][k]) * float(b[k][j]) for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def _to_numpy(value):
    if value is None:
        return None
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    import numpy as np
    return np.asarray(value)


def _normalise_intrinsics(value):
    if value is None:
        raise ValueError("当前 D435i 帧没有相机内参")
    if isinstance(value, (tuple, list)) and len(value) >= 4:
        fx, fy, ppx, ppy = value[:4]
    elif isinstance(value, Mapping):
        fx, fy, ppx, ppy = (value[k] for k in ("fx", "fy", "ppx", "ppy"))
    else:
        fx, fy, ppx, ppy = (getattr(value, k) for k in ("fx", "fy", "ppx", "ppy"))
    vals = tuple(float(v) for v in (fx, fy, ppx, ppy))
    if vals[0] == 0 or vals[1] == 0:
        raise ValueError("D435i 相机内参 fx/fy 无效")
    return vals
