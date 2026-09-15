# -*- coding: utf-8 -*-
"""实验室相机 YOLO 角点适配：用项目已采集的 RGB-D + PLC 位姿出 WORLD。"""
from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

from algorithm_modules.lab.cam_yolo_lab.camera_world_module import YoloD435iWorldLocalizer


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LAB_CAM_DIR = PROJECT_ROOT / "algorithm_modules" / "lab" / "cam_yolo_lab"
DEFAULT_WEIGHT = LAB_CAM_DIR / "weights" / "best.pt"
DEFAULT_EXTRINSIC = LAB_CAM_DIR / "camera_extrinsic.json"
DEFAULT_INTRINSIC = PROJECT_ROOT / "config" / "sensor_coordinate_config" / "camera_intrinsic.json"


def _plc_pose_from_meta(meta: Mapping[str, Any]) -> Dict[str, float]:
    raw = meta.get("plc_pose") or meta.get("gantry_xyzr") or {}
    if isinstance(raw, Mapping) and all(k in raw for k in ("x", "y", "z", "r")):
        return {k: float(raw[k]) for k in ("x", "y", "z", "r")}
    if isinstance(raw, Mapping) and all(k in raw for k in ("X", "Y", "Z", "R")):
        return {
            "x": float(raw["X"]),
            "y": float(raw["Y"]),
            "z": float(raw["Z"]),
            "r": float(raw["R"]),
        }
    raise RuntimeError(
        "实验室相机算法需要采集元数据中的 plc_pose（x/y/z/r）或 gantry_xyzr（X/Y/Z/R）"
    )


def _load_intrinsics() -> tuple[float, float, float, float]:
    data = json.loads(DEFAULT_INTRINSIC.read_text(encoding="utf-8"))
    cameras = data.get("cameras") or {}
    cam = cameras.get("corner_camera") or next(iter(cameras.values()), {})
    k = cam.get("K") or []
    fx = float(k[0][0])
    fy = float(k[1][1])
    ppx = float(k[0][2])
    ppy = float(k[1][2])
    return fx, fy, ppx, ppy


def _depth_scale_mm(meta: Mapping[str, Any], frame_hint: Mapping[str, Any] | None = None) -> float:
    if frame_hint and frame_hint.get("depth_scale_mm") not in (None, ""):
        return float(frame_hint["depth_scale_mm"])
    if meta.get("depth_scale_mm") not in (None, ""):
        return float(meta["depth_scale_mm"])
    # Z16 常见：depth_scale=0.001 m → 1.0 mm/count
    try:
        data = json.loads(DEFAULT_INTRINSIC.read_text(encoding="utf-8"))
        cameras = data.get("cameras") or {}
        cam = cameras.get("corner_camera") or next(iter(cameras.values()), {})
        scale_m = float(cam.get("depth_scale", 0.001) or 0.001)
        return scale_m * 1000.0
    except Exception:
        return 1.0


def build_frame_from_paths(
    rgb_path: str | Path,
    depth_path: str | Path,
    *,
    depth_scale_mm: float = 1.0,
    intrinsics: Sequence[float] | None = None,
) -> Dict[str, Any]:
    import cv2
    import numpy as np
    from utils.cv_io import read_image

    rgb = read_image(str(rgb_path), cv2.IMREAD_COLOR)
    depth = read_image(str(depth_path), cv2.IMREAD_UNCHANGED)
    if rgb is None:
        raise RuntimeError(f"无法读取 RGB：{rgb_path}")
    if depth is None:
        raise RuntimeError(f"无法读取 Depth：{depth_path}")
    depth_arr = np.asarray(depth)
    if depth_arr.ndim == 3:
        depth_arr = depth_arr[:, :, 0]
    fx, fy, ppx, ppy = tuple(intrinsics) if intrinsics and len(intrinsics) >= 4 else _load_intrinsics()
    return {
        "color_array": rgb,
        "depth_array": depth_arr,
        "intrinsics": (float(fx), float(fy), float(ppx), float(ppy)),
        "depth_scale_mm": float(depth_scale_mm),
    }


def _pair_names_for_ids(point_ids: Sequence[str]) -> tuple[str, str]:
    names = [str(x) for x in point_ids]
    if set(names) >= {"P3", "P4"}:
        return ("P3", "P4")
    if set(names) >= {"P1", "P2"}:
        return ("P1", "P2")
    if len(names) == 2:
        ordered = tuple(sorted(names))
        return ordered[0], ordered[1]
    raise RuntimeError(f"实验室相机算法当前仅支持 P1/P2 或 P3/P4 成对识别，收到：{names}")


class LabCameraCornerService:
    def __init__(
        self,
        model_path: Optional[str | Path] = None,
        extrinsic_path: Optional[str | Path] = None,
    ) -> None:
        self.model_path = Path(model_path) if model_path else DEFAULT_WEIGHT
        self.extrinsic_path = Path(extrinsic_path) if extrinsic_path else DEFAULT_EXTRINSIC
        self._localizer: YoloD435iWorldLocalizer | None = None

    def _ensure(self) -> YoloD435iWorldLocalizer:
        if self._localizer is None:
            if not self.model_path.is_file():
                raise FileNotFoundError(f"实验室角点权重不存在：{self.model_path}")
            if not self.extrinsic_path.is_file():
                raise FileNotFoundError(f"实验室相机外参不存在：{self.extrinsic_path}")
            self._localizer = YoloD435iWorldLocalizer(
                model_path=self.model_path,
                extrinsic_path=self.extrinsic_path,
                confidence=0.30,
                imgsz=960,
            )
        return self._localizer

    def locate_from_capture_meta(
        self,
        corner_ids: Sequence[str],
        capture_meta: Mapping[str, Any],
        group_captures: Mapping[str, Any] | None = None,
    ) -> Dict[str, Any]:
        """按采集组（TAIL/HEAD）调用实验室 detect_pair，输出 P1-P4 WORLD。"""
        localizer = self._ensure()
        ids = [str(x) for x in corner_ids]
        if len(ids) != 4:
            return {
                "success": False,
                "algorithm": "lab_camera",
                "message": f"实验室相机算法仅支持 4 点平板，当前角点：{ids}",
            }

        groups: Dict[str, list[str]] = {}
        for pid in ids:
            meta = capture_meta.get(pid) or {}
            group = str(meta.get("capture_group") or "").upper() or "UNKNOWN"
            groups.setdefault(group, []).append(pid)

        world: Dict[str, Dict[str, float]] = {}
        details: Dict[str, Any] = {}
        for group, pids in groups.items():
            sample_meta = capture_meta.get(pids[0]) or {}
            rgb = sample_meta.get("rgb_path")
            depth = sample_meta.get("depth_path")
            if group_captures and group in group_captures:
                cap = group_captures[group] or {}
                rgb = rgb or cap.get("rgb_path")
                depth = depth or cap.get("depth_path")
                scale = _depth_scale_mm(sample_meta, cap)
            else:
                scale = _depth_scale_mm(sample_meta)
            if not rgb or not depth:
                return {
                    "success": False,
                    "algorithm": "lab_camera",
                    "message": f"实验室相机缺少 {group} 组 RGB/Depth",
                }
            plc_pose = _plc_pose_from_meta(sample_meta)
            frame = build_frame_from_paths(rgb, depth, depth_scale_mm=scale)
            pair = _pair_names_for_ids(pids)
            pair_result = localizer.detect_pair(frame, plc_pose, pair)
            for name, xyz in pair_result.items():
                world[name] = {"x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2])}
            details[group] = {
                "pair_names": list(pair),
                "plc_pose": deepcopy(plc_pose),
                "rgb_path": str(rgb),
                "depth_path": str(depth),
            }

        missing = [pid for pid in ids if pid not in world]
        if missing:
            return {
                "success": False,
                "algorithm": "lab_camera",
                "message": f"实验室相机未得到全部角点：缺少 {missing}",
                "world_points": world,
                "details": details,
            }

        return {
            "success": True,
            "algorithm": "lab_camera",
            "source": "lab_yolo_d435i_world",
            "recognition_source": "lab_yolo_d435i_world",
            "coordinate_frame": "world",
            "coordinate_unit": "mm",
            "corner_ids": ids,
            "world_points": world,
            "image_points": {
                pid: {
                    "x": 0.0,
                    "y": 0.0,
                    "point_name": pid,
                    "status": "lab_world_direct",
                    "source": "lab_yolo_d435i_world",
                    "image": (capture_meta.get(pid) or {}).get("rgb_path"),
                }
                for pid in ids
            },
            "details": details,
            "model_path": str(self.model_path),
            "extrinsic_path": str(self.extrinsic_path),
            "message": "实验室 YOLO+深度+外参 已直接输出 P1-P4 WORLD",
            "reviewed": False,
            "review_skipped": True,
        }
