# -*- coding: utf-8 -*-
"""Temporary D435i fork-hole locator.

Pipeline:
RGB -> detect two dark fork holes -> align Depth to RGB -> read D435i intrinsics
-> deproject the two hole centres -> camera-coordinate XYZ (mm).

Output contains only the left/right hole centre coordinates in the D435i camera
coordinate system. No world-coordinate or PLC conversion is performed here.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from itertools import combinations
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class _Candidate:
    x: int
    y: int
    w: int
    h: int
    area: float

    @property
    def cx(self) -> float:
        return self.x + self.w / 2.0

    @property
    def cy(self) -> float:
        return self.y + self.h / 2.0


def _empty_result() -> dict[str, Any]:
    return {
        "left_hole_xyz_mm": None,
        "right_hole_xyz_mm": None,
        "left_hole_pixel": None,
        "right_hole_pixel": None,
        "left_hole_box": None,
        "right_hole_box": None,
    }


def _detect_two_holes(image_bgr: np.ndarray, dark_threshold: int | None = None):
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("image_bgr must be a BGR image with shape HxWx3")

    height, width = image_bgr.shape[:2]
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # The field image is much darker than the earlier lab images, so a fixed
    # threshold is brittle.  Use the image's own lower-intensity distribution
    # to adapt to exposure while keeping a conservative range for the black
    # fork-hole interiors.  A caller can still pass an explicit threshold.
    if dark_threshold is None:
        dark_threshold = int(np.clip(np.percentile(gray, 10.0), 35, 80))

    mask = cv2.inRange(gray, 0, int(dark_threshold))
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 7)),
        iterations=2,
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
        iterations=1,
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = float(width * height)
    candidates: list[_Candidate] = []

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not (0.0015 * frame_area <= area <= 0.075 * frame_area):
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w < 0.045 * width or h < 0.035 * height:
            continue
        if w > 0.34 * width or h > 0.26 * height:
            continue
        aspect = w / max(h, 1)
        if not (0.9 <= aspect <= 4.8):
            continue
        cx, cy = x + w / 2.0, y + h / 2.0
        if not (0.10 * width <= cx <= 0.90 * width):
            continue
        if not (0.14 * height <= cy <= 0.88 * height):
            continue
        # A real fork hole should be substantially darker than a narrow ring
        # immediately around it (the cardboard face).  This rejects floor
        # shadows and dark machine parts that can otherwise satisfy the shape
        # constraints in the field scene.
        pad = max(6, int(round(min(w, h) * 0.18)))
        rx0, ry0 = max(0, x - pad), max(0, y - pad)
        rx1, ry1 = min(width, x + w + pad), min(height, y + h + pad)
        patch = gray[ry0:ry1, rx0:rx1]
        ring = np.ones(patch.shape, dtype=bool)
        ix0, iy0 = x - rx0, y - ry0
        ix1, iy1 = ix0 + w, iy0 + h
        ring[max(0, iy0):min(patch.shape[0], iy1), max(0, ix0):min(patch.shape[1], ix1)] = False
        ring_values = patch[ring]
        if ring_values.size == 0:
            continue
        inner_values = gray[y:y + h, x:x + w]
        local_contrast = float(np.median(ring_values) - np.median(inner_values))
        if local_contrast < 24.0:
            continue

        candidates.append(_Candidate(x, y, w, h, area))

    best = None
    for a, b in combinations(candidates, 2):
        left, right = sorted((a, b), key=lambda c: c.cx)
        dx = right.cx - left.cx
        dy = abs(right.cy - left.cy)
        if dx < 0.10 * width or dx > 0.62 * width or dy > 0.11 * height:
            continue

        area_ratio = min(left.area, right.area) / max(left.area, right.area)
        height_ratio = min(left.h, right.h) / max(left.h, right.h)
        width_ratio = min(left.w, right.w) / max(left.w, right.w)
        if area_ratio < 0.22 or height_ratio < 0.45 or width_ratio < 0.35:
            continue

        mid_x = (left.cx + right.cx) / 2.0
        mid_y = (left.cy + right.cy) / 2.0
        total_area_fraction = (left.area + right.area) / frame_area
        score = (
            4.0 * (dy / height)
            + 1.4 * (1.0 - area_ratio)
            + 0.8 * (1.0 - height_ratio)
            + 0.5 * (1.0 - width_ratio)
            + 0.8 * abs(mid_x - width / 2.0) / width
            + 0.20 * abs(mid_y - height / 2.0) / height
            - 2.5 * total_area_fraction
        )
        if best is None or score < best[0]:
            best = (score, left, right)

    if best is None:
        return None
    return best[1], best[2]


def _sample_hole_mouth_depth(depth_mm: np.ndarray, box: _Candidate) -> float | None:
    """Estimate the hole-mouth plane depth using valid depth around the aperture.

    The dark interior can lie much farther behind the opening, so the primary
    depth estimate comes from a thin ring around the opening. If too little ring
    depth is valid, a small centre patch is used as a fallback.
    """
    if depth_mm is None or depth_mm.ndim != 2:
        raise ValueError("depth_mm must be an aligned single-channel depth image in mm")

    h_img, w_img = depth_mm.shape
    pad = max(5, int(round(min(box.w, box.h) * 0.08)))
    x0 = max(0, box.x - pad)
    y0 = max(0, box.y - pad)
    x1 = min(w_img, box.x + box.w + pad)
    y1 = min(h_img, box.y + box.h + pad)

    patch = depth_mm[y0:y1, x0:x1].astype(np.float64, copy=False)
    ring = np.ones(patch.shape, dtype=bool)
    ix0, iy0 = box.x - x0, box.y - y0
    ix1, iy1 = ix0 + box.w, iy0 + box.h
    ring[max(0, iy0):min(patch.shape[0], iy1), max(0, ix0):min(patch.shape[1], ix1)] = False
    valid = patch[ring & np.isfinite(patch) & (patch > 0)]

    if valid.size < 20:
        u, v = int(round(box.cx)), int(round(box.cy))
        r = max(3, min(10, int(round(min(box.w, box.h) * 0.05))))
        px0, px1 = max(0, u - r), min(w_img, u + r + 1)
        py0, py1 = max(0, v - r), min(h_img, v + r + 1)
        local = depth_mm[py0:py1, px0:px1].astype(np.float64, copy=False)
        valid = local[np.isfinite(local) & (local > 0)]

    if valid.size == 0:
        return None
    return float(np.median(valid))


def _pixel_to_camera_xyz(pixel, depth_mm: float, intrinsics: dict[str, float]) -> list[float]:
    """Deproject an RGB pixel to D435i camera XYZ in millimetres.

    RealSense camera convention: +X right, +Y down, +Z forward.
    """
    u, v = float(pixel[0]), float(pixel[1])
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("fx and fy must be non-zero")

    z = float(depth_mm)
    return [(u - cx) * z / fx, (v - cy) * z / fy, z]


def locate_fork_holes(
    image_bgr: np.ndarray,
    depth_mm: np.ndarray,
    intrinsics: dict[str, float],
    dark_threshold: int | None = None,
) -> dict[str, Any]:
    """Return only left/right fork-hole centres in D435i camera XYZ, mm."""
    pair = _detect_two_holes(image_bgr, dark_threshold=dark_threshold)
    if pair is None:
        return _empty_result()
    if depth_mm.shape[:2] != image_bgr.shape[:2]:
        raise ValueError("depth_mm must be aligned to the RGB image")

    output: list[list[float] | None] = []
    for box in pair:
        z_mm = _sample_hole_mouth_depth(depth_mm, box)
        if z_mm is None:
            output.append(None)
            continue
        p_c = _pixel_to_camera_xyz([box.cx, box.cy], z_mm, intrinsics)
        output.append([round(v, 3) for v in p_c])

    return {
        "left_hole_xyz_mm": output[0],
        "right_hole_xyz_mm": output[1],
        # 仅供结果图叠加显示孔中心和轮廓；不参与 WORLD/PLC 坐标下发。
        "left_hole_pixel": [round(pair[0].cx, 1), round(pair[0].cy, 1)],
        "right_hole_pixel": [round(pair[1].cx, 1), round(pair[1].cy, 1)],
        "left_hole_box": [pair[0].x, pair[0].y, pair[0].w, pair[0].h],
        "right_hole_box": [pair[1].x, pair[1].y, pair[1].w, pair[1].h],
    }


class D435iForkHoleLocator:
    """Live D435i wrapper that automatically reads intrinsics from the SDK."""

    def __init__(self, width: int = 1280, height: int = 720, fps: int = 30, dark_threshold: int | None = None):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.dark_threshold = None if dark_threshold is None else int(dark_threshold)
        self._pipeline = None
        self._align = None
        self._depth_scale_mm = None

    def start(self):
        if self._pipeline is not None:
            return self
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("pyrealsense2 is not installed") from exc

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        profile = pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color)
        self._depth_scale_mm = float(depth_sensor.get_depth_scale()) * 1000.0
        return self

    def stop(self):
        if self._pipeline is not None:
            self._pipeline.stop()
        self._pipeline = None
        self._align = None
        self._depth_scale_mm = None

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    def locate_once(self) -> dict[str, Any]:
        """Capture one aligned RGB-D frame and return the two camera XYZ points."""
        if self._pipeline is None:
            self.start()

        frames = self._pipeline.wait_for_frames()
        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return _empty_result()

        image_bgr = np.asanyarray(color_frame.get_data())
        depth_mm = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self._depth_scale_mm

        # Intrinsics are read automatically from the active D435i color stream.
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        intrinsics = {"fx": intr.fx, "fy": intr.fy, "cx": intr.ppx, "cy": intr.ppy}

        return locate_fork_holes(
            image_bgr,
            depth_mm,
            intrinsics,
            dark_threshold=self.dark_threshold,
        )


def _main() -> int:
    with D435iForkHoleLocator() as locator:
        result = locator.locate_once()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
