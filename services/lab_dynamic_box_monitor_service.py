# -*- coding: utf-8 -*-
"""实验室纸箱四角 WORLD 区域判定的纯几何核心。"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from services.sensor_calibration_service import SensorCalibrationService


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _xyz(value: Any) -> np.ndarray:
    if isinstance(value, Mapping):
        data = (
            value.get("x", value.get("x_mm")),
            value.get("y", value.get("y_mm")),
            value.get("z", value.get("z_mm")),
        )
    else:
        data = tuple(value[:3])
    if len(data) != 3 or any(item is None for item in data):
        raise ValueError(f"无效三维点：{value!r}")
    point = np.asarray(data, dtype=np.float64)
    if not np.all(np.isfinite(point)):
        raise ValueError(f"无效三维点：{value!r}")
    return point


def _project_region_and_points(
    region_corners: Sequence[Any],
    points: Sequence[Any],
) -> tuple[np.ndarray, np.ndarray]:
    if len(region_corners) != 4:
        raise ValueError("规划区域必须包含四个 WORLD 角点")
    region = np.stack([_xyz(point) for point in region_corners])
    origin = region[0]
    axis_u = region[1] - origin
    axis_v_hint = region[2] - origin
    u_norm = float(np.linalg.norm(axis_u))
    if u_norm <= 1e-9:
        raise ValueError("规划区域横向边长度为 0")
    axis_u = axis_u / u_norm
    normal = np.cross(axis_u, axis_v_hint)
    n_norm = float(np.linalg.norm(normal))
    if n_norm <= 1e-9:
        raise ValueError("规划区域四角退化，无法建立平面")
    normal = normal / n_norm
    axis_v = np.cross(normal, axis_u)
    if float(np.dot(axis_v, axis_v_hint)) < 0:
        axis_v = -axis_v

    def project(point: np.ndarray) -> list[float]:
        delta = point - origin
        return [float(np.dot(delta, axis_u)), float(np.dot(delta, axis_v))]

    # 区域存储顺序为起点左/右、终点左/右；转成环绕多边形顺序。
    polygon = np.asarray([project(region[index]) for index in (0, 1, 3, 2)], dtype=np.float64)
    projected = np.asarray([project(_xyz(point)) for point in points], dtype=np.float64)
    return polygon, projected


def _inside_convex_polygon(point: np.ndarray, polygon: np.ndarray, tolerance_mm: float) -> bool:
    signs = []
    for index in range(len(polygon)):
        start = polygon[index]
        end = polygon[(index + 1) % len(polygon)]
        edge = end - start
        rel = point - start
        signs.append(float(edge[0] * rel[1] - edge[1] * rel[0]))
    tolerance = abs(float(tolerance_mm))
    return all(value >= -tolerance for value in signs) or all(value <= tolerance for value in signs)


def judge_box_world_region(
    box_corners_world_xyz_mm: Sequence[Any],
    region: Mapping[str, Any],
    *,
    tolerance_mm: float = 0.0,
) -> dict[str, Any]:
    """四个纸箱角点全部落在规划区域内才通过；高度不参与边界判定。"""
    if len(box_corners_world_xyz_mm) != 4:
        raise ValueError("纸箱必须包含四个 WORLD 角点")
    polygon, points = _project_region_and_points(
        region.get("corners_world_xyz_mm") or [],
        box_corners_world_xyz_mm,
    )
    flags = [
        _inside_convex_polygon(point, polygon, tolerance_mm)
        for point in points
    ]
    outside = [f"P{index + 1}" for index, inside in enumerate(flags) if not inside]
    passed = bool(all(flags))
    return {
        "plan_check_status": "PASS" if passed else "OUT_OF_REGION",
        "inside_planned_region": passed,
        "corner_inside": flags,
        "outside_corners": outside,
        "planned_region_id": str(region.get("region_id") or region.get("blind_code") or ""),
        "coordinate_frame": "world",
        "coordinate_unit": "mm",
    }


def evaluate_selected_box_world(
    selected_box: Mapping[str, Any],
    region: Mapping[str, Any],
    pixel_to_world: Callable[[float, float], Mapping[str, Any]],
    *,
    tolerance_mm: float = 0.0,
) -> dict[str, Any]:
    """把动态模块的四个 UV 角点转换到 WORLD 后再判定区域。"""
    corners_uv = list(selected_box.get("corners_uv") or [])
    if len(corners_uv) != 4:
        raise ValueError("动态监测结果缺少四个纸箱角点")
    converted = [pixel_to_world(float(point[0]), float(point[1])) for point in corners_uv]
    world = [list(item["world_xyz_mm"]) for item in converted]
    camera = [list(item["camera_xyz_mm"]) for item in converted]
    result = deepcopy(dict(selected_box))
    result.update(judge_box_world_region(world, region, tolerance_mm=tolerance_mm))
    result["corners_world_xyz_mm"] = world
    result["corners_camera_xyz_mm"] = camera
    result["corner_depth_values"] = [float(item.get("depth_value", 0.0)) for item in converted]
    return result


class LabDynamicBoxMonitorService:
    """接入用户 dynamic_monitor_lab：RGB 找箱角，Depth+外参转 WORLD 后判区。"""

    def __init__(
        self,
        *,
        calibration: SensorCalibrationService | None = None,
        detector: Callable[..., Any] | None = None,
        image_reader: Callable[[str], Any] | None = None,
        depth_sampler: Callable[[str, float, float, int], float] | None = None,
        image_writer: Callable[[str | Path, Any], bool] | None = None,
        result_root: str | Path | None = None,
        config_path: str | Path | None = None,
        depth_window: int = 5,
        boundary_tolerance_mm: float = 0.0,
    ) -> None:
        self.calibration = calibration or SensorCalibrationService()
        self.depth_window = int(depth_window)
        self.boundary_tolerance_mm = float(boundary_tolerance_mm)
        self.result_root = Path(result_root or (PROJECT_ROOT / "workdir" / "recognition_results" / "lab_dynamic_box"))
        self._detector = detector
        self._image_reader = image_reader or self._read_image
        self._depth_sampler = depth_sampler or self.calibration.sample_depth
        self._image_writer = image_writer or self._write_image
        self.config_path = Path(config_path) if config_path else (
            PROJECT_ROOT / "algorithm_modules" / "lab" / "dynamic_box_monitor" / "config.json"
        )
        self._config: dict[str, Any] | None = None

    @staticmethod
    def _read_image(path: str):
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("动态纸箱监测需要 opencv-python") from exc
        from utils.cv_io import read_image

        image = read_image(path, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"无法读取动态监测 RGB：{path}")
        return image

    @staticmethod
    def _write_image(path: str | Path, image: Any) -> bool:
        from utils.cv_io import write_image

        return bool(write_image(path, image))

    def _detector_and_config(self):
        from algorithm_modules.lab.dynamic_box_monitor.dynamic_box_monitor import (
            analyze_image,
            load_config,
        )

        if self._config is None:
            self._config = load_config(str(self.config_path) if self.config_path.is_file() else None)
        return self._detector or analyze_image, deepcopy(self._config)

    def _intrinsics(self, capture: Mapping[str, Any]) -> dict[str, float]:
        live = dict(capture.get("intrinsics") or {})
        if all(key in live for key in ("fx", "fy")):
            return {
                "fx": float(live["fx"]),
                "fy": float(live["fy"]),
                "cx": float(live.get("cx", live.get("ppx", 0.0))),
                "cy": float(live.get("cy", live.get("ppy", 0.0))),
            }
        configured = self.calibration.camera_intrinsic(str(capture.get("camera_id") or "CAM_PICK"))
        matrix = np.asarray(configured["K"], dtype=np.float64)
        return {
            "fx": float(matrix[0, 0]),
            "fy": float(matrix[1, 1]),
            "cx": float(matrix[0, 2]),
            "cy": float(matrix[1, 2]),
        }

    def _pixel_to_world(self, capture: Mapping[str, Any], u: float, v: float) -> dict[str, Any]:
        depth_path = str(capture.get("depth_path") or "")
        raw_depth = float(self._depth_sampler(depth_path, u, v, self.depth_window))
        depth_mm = raw_depth * float(capture.get("depth_scale_mm", 1.0) or 1.0)
        if depth_mm <= 0:
            raise RuntimeError(f"纸箱角点 ({u:.1f},{v:.1f}) 深度无效")
        intr = self._intrinsics(capture)
        if abs(intr["fx"]) < 1e-12 or abs(intr["fy"]) < 1e-12:
            raise RuntimeError("D435i 实拍内参 fx/fy 无效")
        camera = np.asarray([
            (float(u) - intr["cx"]) / intr["fx"] * depth_mm,
            (float(v) - intr["cy"]) / intr["fy"] * depth_mm,
            depth_mm,
        ], dtype=np.float64)
        world_matrix = self.calibration.dynamic_camera_world_matrix(capture.get("camera_world_pose") or {})
        world = self.calibration.transform_point(world_matrix, camera)
        return {
            "pixel_uv": [float(u), float(v)],
            "depth_value": raw_depth,
            "depth_mm": depth_mm,
            "camera_xyz_mm": [float(value) for value in camera],
            "world_xyz_mm": [float(value) for value in world],
        }

    def _save_artifacts(self, annotated: Any, mask: Any, phase: str, result_tag: str) -> tuple[str, str]:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(result_tag or "region"))
        output_dir = self.result_root / f"{stamp}_{phase}_{safe}"
        result_path = output_dir / "box_monitor_result.jpg"
        mask_path = output_dir / "box_monitor_mask.png"
        output_dir.mkdir(parents=True, exist_ok=True)
        if not self._image_writer(result_path, annotated):
            raise RuntimeError(f"动态监测结果图保存失败：{result_path}")
        if not self._image_writer(mask_path, mask):
            raise RuntimeError(f"动态监测掩膜图保存失败：{mask_path}")
        return str(result_path.resolve()), str(mask_path.resolve())

    def analyze_capture(
        self,
        capture: Mapping[str, Any],
        region: Mapping[str, Any],
        *,
        phase: str,
        result_tag: str,
    ) -> dict[str, Any]:
        rgb_path = str(capture.get("rgb_path") or capture.get("image_path") or "")
        depth_path = str(capture.get("depth_path") or "")
        if not capture.get("success") or not rgb_path or not depth_path:
            raise RuntimeError("实验室动态监测必须取得真实 RGB + 对齐深度")
        image = self._image_reader(rgb_path)
        detector, config = self._detector_and_config()
        detection, annotated, mask = detector(image, config, planned_region_uv=None)
        result_path, mask_path = self._save_artifacts(annotated, mask, str(phase), str(result_tag))
        selected = detection.get("selected_box") if isinstance(detection, Mapping) else None
        base = {
            "success": False,
            "algorithm": "dynamic_box_monitor_lab_traditional_opencv",
            "phase": str(phase),
            "planned_region_id": str(region.get("region_id") or region.get("blind_code") or ""),
            "rgb_path": rgb_path,
            "depth_path": depth_path,
            "result_image_path": result_path,
            "mask_image_path": mask_path,
            "box_count": int((detection or {}).get("box_count", 0)),
            "selection_rule": (detection or {}).get("selection_rule"),
            "all_boxes": deepcopy((detection or {}).get("all_boxes") or []),
        }
        if not selected:
            return {
                **base,
                "status": "NO_BOX",
                "inside_planned_region": False,
                "plan_check_status": "NO_BOX",
                "message": f"{base['planned_region_id']} 未检测到纸箱",
            }
        evaluated = evaluate_selected_box_world(
            selected,
            region,
            lambda u, v: self._pixel_to_world(capture, u, v),
            tolerance_mm=self.boundary_tolerance_mm,
        )
        passed = bool(evaluated.get("inside_planned_region"))
        return {
            **base,
            **evaluated,
            "success": passed,
            "status": "PASS" if passed else "OUT_OF_REGION",
            "message": (
                f"{base['planned_region_id']} 纸箱四角均在规划区域内"
                if passed
                else f"{base['planned_region_id']} 纸箱角点越界：{', '.join(evaluated.get('outside_corners') or [])}"
            ),
        }
