# -*- coding: utf-8 -*-
"""D435i fork-hole locator from ``fork_hole_final.zip``.

RGB -> detect two dark fork holes -> aligned depth -> camera XYZ (mm).
World-coordinate conversion remains the responsibility of the main system.
"""
from __future__ import annotations

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
    return {"left_hole_xyz_mm": None, "right_hole_xyz_mm": None}


def _detect_two_holes(image_bgr: np.ndarray, dark_threshold: int = 88):
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("image_bgr must be a BGR image with shape HxWx3")

    height, width = image_bgr.shape[:2]
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
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
        candidates.append(_Candidate(x, y, w, h, area))

    best = None
    for a, b in combinations(candidates, 2):
        left, right = sorted((a, b), key=lambda candidate: candidate.cx)
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
    """Estimate hole-mouth depth from a thin valid-depth ring around the hole."""
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
        radius = max(3, min(10, int(round(min(box.w, box.h) * 0.05))))
        px0, px1 = max(0, u - radius), min(w_img, u + radius + 1)
        py0, py1 = max(0, v - radius), min(h_img, v + radius + 1)
        local = depth_mm[py0:py1, px0:px1].astype(np.float64, copy=False)
        valid = local[np.isfinite(local) & (local > 0)]

    if valid.size == 0:
        return None
    return float(np.median(valid))


def _pixel_to_camera_xyz(pixel, depth_mm: float, intrinsics: dict[str, float]) -> list[float]:
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
    dark_threshold: int = 88,
) -> dict[str, Any]:
    """Return left/right fork-hole centres in D435i camera XYZ, millimetres."""
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
        point = _pixel_to_camera_xyz([box.cx, box.cy], z_mm, intrinsics)
        output.append([round(value, 3) for value in point])

    return {
        "left_hole_xyz_mm": output[0],
        "right_hole_xyz_mm": output[1],
    }
