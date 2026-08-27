"""RGB + D435i Z16 dynamic monitoring detector.

This module is based on the user-supplied ``Dynamic_ monitoring_module.zip``.
The OpenCV compatibility fix is intentional: recent OpenCV builds may return
``HoughLinesP`` as ``(N, 4)`` instead of ``(N, 1, 4)``.  Both forms are
normalized before iteration.  The documented ``offset_deg`` is the detected
box bottom-edge angle; the intersecting board-edge angle is exposed separately.
"""
from __future__ import annotations

from pathlib import Path
import json

import cv2
import numpy as np


class DetectorError(RuntimeError):
    pass


def _line_angle(line):
    x1, y1, x2, y2 = np.asarray(line, dtype=float).reshape(4)
    if x2 < x1:
        y1, y2 = y2, y1
        x1, x2 = x2, x1
    return float(np.degrees(np.arctan2(y2 - y1, x2 - x1)))


def _lines(value):
    if value is None:
        return []
    return np.asarray(value).reshape(-1, 4)


def _load_camera(path):
    with open(path, "r", encoding="utf-8") as stream:
        camera = json.load(stream)
    required = ("fx", "fy", "ppx", "ppy", "depth_scale")
    missing = [key for key in required if key not in camera]
    if missing:
        raise DetectorError(f"相机内参缺少字段：{missing}")
    return camera


def _cross(a, b):
    x1, y1, x2, y2 = a
    x3, y3, x4, y4 = b
    matrix = np.array([[y1 - y2, x2 - x1], [y3 - y4, x4 - x3]], float)
    vector = -np.array([x1 * y2 - x2 * y1, x3 * y4 - x4 * y3])
    return np.linalg.solve(matrix, vector)


def _detect_corner(color):
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    edge = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 20, 70)
    height, width = gray.shape

    raw_bottom_lines = cv2.HoughLinesP(
        edge, 1, np.pi / 720, threshold=45,
        minLineLength=int(width * 0.2), maxLineGap=30,
    )
    candidates = [
        line for line in _lines(raw_bottom_lines)
        if abs(_line_angle(line)) < 12 and (line[1] + line[3]) / 2 > height * 0.5
    ]
    if not candidates:
        raise DetectorError("未找到箱体底边")
    bottom = max(candidates, key=lambda line: (line[1] + line[3]) / 2)

    raw_board_lines = cv2.HoughLinesP(
        edge, 1, np.pi / 720, threshold=20,
        minLineLength=40, maxLineGap=50,
    )
    for board in _lines(raw_board_lines):
        board_angle = _line_angle(board)
        if -85 < board_angle < -20:
            try:
                point = _cross(bottom, board)
            except (ValueError, np.linalg.LinAlgError):
                continue
            if -30 < point[0] < width * 0.45 and height * 0.5 < point[1] < height:
                return point, bottom, board, _line_angle(bottom), board_angle
    raise DetectorError("未找到箱体底边与底板边缘交点")


def detect_from_files(color_path, depth_path, camera_path):
    color_path = Path(color_path).expanduser().resolve()
    depth_path = Path(depth_path).expanduser().resolve()
    camera_path = Path(camera_path).expanduser().resolve()
    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    if color is None or depth is None:
        raise DetectorError("图像读取失败")
    if depth.ndim == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2 or depth.dtype != np.uint16:
        raise DetectorError("必须输入单通道 Z16 深度图")
    camera = _load_camera(camera_path)

    corner, bottom, board, bottom_angle, board_angle = _detect_corner(color)
    uv_depth = np.array([
        corner[0] * depth.shape[1] / color.shape[1],
        corner[1] * depth.shape[0] / color.shape[0],
    ])
    u, v = np.round(uv_depth).astype(int)
    u = int(np.clip(u, 0, depth.shape[1] - 1))
    v = int(np.clip(v, 0, depth.shape[0] - 1))
    area = depth[max(0, v - 3):min(depth.shape[0], v + 4), max(0, u - 3):min(depth.shape[1], u + 4)]
    valid = area[area > 0]
    if len(valid) == 0:
        raise DetectorError("角点无有效深度")

    z_m = float(np.median(valid)) * float(camera["depth_scale"])
    x_m = (float(uv_depth[0]) - float(camera["ppx"])) * z_m / float(camera["fx"])
    y_m = (float(uv_depth[1]) - float(camera["ppy"])) * z_m / float(camera["fy"])
    return {
        "success": True,
        "algorithm": "dynamic_monitoring_opencv_rgbd",
        "corner_xyz_m": [float(x_m), float(y_m), float(z_m)],
        "corner_uv_color_px": [float(corner[0]), float(corner[1])],
        "corner_uv_depth_px": [float(uv_depth[0]), float(uv_depth[1])],
        "bottom_edge_xyxy_px": [int(value) for value in bottom],
        "board_edge_xyxy_px": [int(value) for value in board],
        "offset_deg": float(bottom_angle),
        "board_edge_deg": float(board_angle),
        "color_size_px": [int(color.shape[1]), int(color.shape[0])],
        "depth_size_px": [int(depth.shape[1]), int(depth.shape[0])],
    }
