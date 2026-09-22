# -*- coding: utf-8 -*-
"""用户 dynamic_monitor_lab：LAB 颜色分割提取纸箱四角。"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np

try:
    import cv2
except ImportError:  # 允许无 OpenCV 的纯几何测试环境导入；真机调用时给出明确错误。
    cv2 = None


DEFAULT_CONFIG: Dict[str, Any] = {
    "lab_lower": [100, 115, 136],
    "lab_upper": [190, 138, 160],
    "roi_norm": [0.22, 0.00, 0.78, 1.00],
    "close_kernel": 11,
    "close_iterations": 2,
    "open_kernel": 5,
    "open_iterations": 1,
    "min_area_ratio": 0.015,
    "max_area_ratio": 0.140,
    "min_width_ratio": 0.12,
    "max_width_ratio": 0.35,
    "min_height_ratio": 0.08,
    "max_height_ratio": 0.32,
    "min_aspect_ratio": 1.15,
    "max_aspect_ratio": 2.70,
    "min_fill_ratio": 0.60,
    "planned_region_uv": None,
}


def _require_cv2():
    if cv2 is None:
        raise RuntimeError("dynamic_monitor_lab 需要 opencv-python")
    return cv2


def load_config(path: Optional[str]) -> Dict[str, Any]:
    config = dict(DEFAULT_CONFIG)
    if path:
        with open(path, "r", encoding="utf-8") as handle:
            config.update(json.load(handle))
    return config


def order_corners(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = pts.sum(axis=1)
    diffs = np.diff(pts, axis=1).reshape(-1)
    return np.asarray(
        [pts[np.argmin(sums)], pts[np.argmin(diffs)], pts[np.argmax(sums)], pts[np.argmax(diffs)]],
        dtype=np.float32,
    )


def make_mask(image: np.ndarray, config: Dict[str, Any]) -> np.ndarray:
    cv = _require_cv2()
    height, width = image.shape[:2]
    lab = cv.cvtColor(image, cv.COLOR_BGR2LAB)
    mask = cv.inRange(
        lab,
        np.asarray(config["lab_lower"], dtype=np.uint8),
        np.asarray(config["lab_upper"], dtype=np.uint8),
    )
    xmin, ymin, xmax, ymax = config["roi_norm"]
    roi = np.zeros_like(mask)
    roi[int(ymin * height):int(ymax * height), int(xmin * width):int(xmax * width)] = 255
    mask = cv.bitwise_and(mask, roi)
    close_size = max(1, int(config["close_kernel"]))
    open_size = max(1, int(config["open_kernel"]))
    mask = cv.morphologyEx(
        mask,
        cv.MORPH_CLOSE,
        cv.getStructuringElement(cv.MORPH_RECT, (close_size, close_size)),
        iterations=int(config["close_iterations"]),
    )
    return cv.morphologyEx(
        mask,
        cv.MORPH_OPEN,
        cv.getStructuringElement(cv.MORPH_RECT, (open_size, open_size)),
        iterations=int(config["open_iterations"]),
    )


def extract_candidates(mask: np.ndarray, config: Dict[str, Any]) -> List[Dict[str, Any]]:
    cv = _require_cv2()
    height, width = mask.shape[:2]
    image_area = float(height * width)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    output: List[Dict[str, Any]] = []
    for contour in contours:
        area = float(cv.contourArea(contour))
        if not config["min_area_ratio"] * image_area <= area <= config["max_area_ratio"] * image_area:
            continue
        x, y, box_width, box_height = cv.boundingRect(contour)
        if not config["min_width_ratio"] * width <= box_width <= config["max_width_ratio"] * width:
            continue
        if not config["min_height_ratio"] * height <= box_height <= config["max_height_ratio"] * height:
            continue
        aspect = max(box_width, box_height) / max(1, min(box_width, box_height))
        if not config["min_aspect_ratio"] <= aspect <= config["max_aspect_ratio"]:
            continue
        fill = area / max(1.0, float(box_width * box_height))
        if fill < config["min_fill_ratio"]:
            continue
        rectangle = cv.minAreaRect(contour)
        corners = order_corners(cv.boxPoints(rectangle))
        center_u, center_v = rectangle[0]
        p0, p1 = corners[0], corners[1]
        angle = math.degrees(math.atan2(float(p1[1] - p0[1]), float(p1[0] - p0[0])))
        output.append({
            "area_px2": round(area, 3),
            "bbox_xywh": [int(x), int(y), int(box_width), int(box_height)],
            "center_uv": [round(float(center_u), 3), round(float(center_v), 3)],
            "corners_uv": [[round(float(px), 3), round(float(py), 3)] for px, py in corners],
            "rotation_deg": round(float(angle), 3),
            "fill_ratio": round(float(fill), 4),
        })
    output.sort(key=lambda item: item["center_uv"][1])
    return output


def judge_plan(selected: Dict[str, Any], planned: Optional[Sequence[Sequence[float]]]) -> Dict[str, Any]:
    if planned is None:
        return {
            "plan_check_status": "NOT_CONFIGURED",
            "inside_planned_region": None,
            "corner_inside": None,
            "outside_corners": [],
            "min_corner_margin_px": None,
            "overlap_ratio": None,
        }
    cv = _require_cv2()
    polygon = np.asarray(planned, np.float32).reshape(-1, 2)
    corners = np.asarray(selected["corners_uv"], np.float32).reshape(4, 2)
    flags, distances, outside = [], [], []
    for index, point in enumerate(corners, 1):
        distance = float(cv.pointPolygonTest(polygon, (float(point[0]), float(point[1])), True))
        distances.append(distance)
        flags.append(distance >= 0)
        if distance < 0:
            outside.append(f"P{index}")
    all_inside = bool(all(flags))
    try:
        intersection_area, _ = cv.intersectConvexConvex(corners, polygon)
        box_area = abs(float(cv.contourArea(corners)))
        overlap = float(intersection_area / box_area) if box_area > 1e-6 else 0.0
    except cv.error:
        overlap = None
    return {
        "plan_check_status": "PASS" if all_inside else "OUT_OF_REGION",
        "inside_planned_region": all_inside,
        "corner_inside": [bool(value) for value in flags],
        "outside_corners": outside,
        "corner_signed_distance_px": [round(value, 3) for value in distances],
        "min_corner_margin_px": round(min(distances), 3),
        "overlap_ratio": None if overlap is None else round(overlap, 4),
    }


def draw_result(image: np.ndarray, result: Mapping[str, Any], planned=None):
    cv = _require_cv2()
    output = image.copy()
    if planned is not None:
        polygon = np.asarray(planned, np.int32).reshape(-1, 1, 2)
        cv.polylines(output, [polygon], True, (255, 255, 0), 3, cv.LINE_AA)
    boxes = result.get("all_boxes", [])
    selected = result.get("selected_box")
    for index, box in enumerate(boxes):
        points = np.asarray(box["corners_uv"], np.int32).reshape(-1, 1, 2)
        is_selected = selected is not None and index == 0
        color = (0, 255, 0) if is_selected else (0, 165, 255)
        cv.polylines(output, [points], True, color, 4 if is_selected else 2, cv.LINE_AA)
        if is_selected:
            for corner_index, (px, py) in enumerate(np.asarray(box["corners_uv"], np.int32), 1):
                cv.circle(output, (int(px), int(py)), 6, (0, 255, 0), -1, cv.LINE_AA)
                cv.putText(output, f"P{corner_index}", (int(px) + 7, int(py) - 7), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv.LINE_AA)
    cv.rectangle(output, (12, 12), (600, 105), (20, 20, 20), -1)
    cv.putText(output, f"Detected boxes: {result.get('box_count', 0)}", (28, 42), cv.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv.LINE_AA)
    status = "N/A" if selected is None else selected.get("plan_check_status", "N/A")
    cv.putText(output, f"Plan check: {status}", (28, 98), cv.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv.LINE_AA)
    return output


def analyze_image(image: np.ndarray, config=None, planned_region_uv=None):
    active = dict(DEFAULT_CONFIG if config is None else config)
    mask = make_mask(image, active)
    boxes = extract_candidates(mask, active)
    result = {
        "status": "OK" if boxes else "NO_BOX",
        "box_count": len(boxes),
        "selection_rule": "single_box" if len(boxes) == 1 else ("topmost_min_center_v" if boxes else None),
        "all_boxes": boxes,
        "selected_box": None,
    }
    if boxes:
        selected = dict(boxes[0])
        selected.update(judge_plan(selected, planned_region_uv))
        result["selected_box"] = selected
    return result, draw_result(image, result, planned_region_uv), mask
