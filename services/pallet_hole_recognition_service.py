# -*- coding: utf-8 -*-
"""托盘插孔识别服务：YOLO + 对齐 RGB-D 深度定位。

模型先检测左右两个托盘插孔，再在插孔两侧实心区域估计深度，
最后利用相机内参把检测框中心像素转换成相机坐标系 XYZ（mm）。
"""
from __future__ import annotations

import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

from config.system_config import get_system_config

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "models" / "pallet_hole_best.pt"
DEFAULT_INTRINSIC_PATH = PROJECT_ROOT / "config" / "sensor_coordinate_config" / "camera_intrinsic.json"
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "workdir" / "recognition_results" / "pallet_hole"


class PalletHoleRecognitionService:
    def __init__(
        self,
        model_path: Optional[str | Path] = None,
        intrinsics: Optional[Dict[str, float]] = None,
        confidence: Optional[float] = None,
        side_width_px: Optional[int] = None,
        vertical_fraction: Optional[float] = None,
        device: Optional[str] = None,
        result_root: Optional[str | Path] = None,
    ):
        cfg = self._load_config()
        ph = cfg.get("pallet_hole", {}) if isinstance(cfg, dict) else {}
        configured_model = model_path or ph.get("model_path") or DEFAULT_MODEL_PATH
        path = Path(configured_model)
        if not path.is_absolute():
            path = (PROJECT_ROOT / path).resolve()
        self.model_path = path
        calibrated = self._load_hole_camera_intrinsics()
        self.intrinsics = dict(intrinsics or ph.get("camera_intrinsics") or calibrated or {
            "fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0
        })
        self.confidence = float(confidence if confidence is not None else ph.get("confidence", 0.30))
        self.side_width_px = int(side_width_px if side_width_px is not None else ph.get("side_width_px", 12))
        self.vertical_fraction = float(
            vertical_fraction if vertical_fraction is not None else ph.get("vertical_fraction", 0.60)
        )
        self.device = device
        self.result_root = Path(result_root or DEFAULT_RESULT_ROOT).expanduser().resolve()
        self._model = None


    @staticmethod
    def _safe_tag(value: str) -> str:
        text = re.sub(r"[^0-9A-Za-z._-]+", "_", str(value or "").strip())
        return text.strip("._-") or "recognition"

    def _create_result_dir(self, result_tag: str = "") -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        tag = self._safe_tag(result_tag) if result_tag else "pallet"
        folder = self.result_root / f"{tag}_{stamp}"
        folder.mkdir(parents=True, exist_ok=True)
        return folder

    @staticmethod
    def _save_result_image(rgb, holes: list[Dict[str, Any]], output_dir: Path) -> str:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("保存托盘插孔识别结果图需要 opencv-python") from exc

        image = rgb.copy()
        for hole in holes:
            x1, y1, x2, y2 = [int(round(float(v))) for v in hole["bbox_xyxy"]]
            u, v = [int(round(float(x))) for x in hole["uv_px"]]
            x_mm, y_mm, z_mm = [float(x) for x in hole["xyz_mm"]]
            conf = float(hole.get("confidence", 0.0))
            name = str(hole.get("name") or "hole")
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.drawMarker(image, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
            label = f"{name} conf={conf:.3f} XYZ=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f})mm"
            cv2.putText(
                image, label, (max(4, x1), max(22, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2, cv2.LINE_AA,
            )

        output_path = output_dir / "pallet_holes_result.jpg"
        from utils.cv_io import write_image
        if not write_image(output_path, image):
            raise RuntimeError(f"托盘插孔识别结果图保存失败：{output_path}")
        return str(output_path.resolve())

    @staticmethod
    def _load_hole_camera_intrinsics() -> Dict[str, float]:
        try:
            payload = json.loads(DEFAULT_INTRINSIC_PATH.read_text(encoding="utf-8"))
            cam = (payload.get("cameras") or {}).get("hole_camera") or {}
            K = cam.get("K")
            if K and len(K) == 3:
                return {
                    "fx": float(K[0][0]), "fy": float(K[1][1]),
                    "cx": float(K[0][2]), "cy": float(K[1][2]),
                    "depth_scale": float(cam.get("depth_scale", 0.001)),
                    "calibration_name": "hole_camera",
                    "status": cam.get("status", "unknown"),
                }
        except Exception:
            pass
        return {}

    @staticmethod
    def _load_config() -> Dict[str, Any]:
        return get_system_config()

    def _ensure_model(self):
        if self._model is not None:
            return self._model
        if not self.model_path.is_file():
            raise RuntimeError(f"托盘插孔模型不存在：{self.model_path}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("缺少 ultralytics，请执行：pip install ultralytics") from exc
        self._model = YOLO(str(self.model_path))
        return self._model

    @staticmethod
    def _read_rgb_depth(rgb_path: str | Path, depth_path: str | Path):
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("托盘 RGB-D 识别需要 opencv-python 和 numpy") from exc
        from utils.cv_io import read_image
        rgb = read_image(rgb_path, cv2.IMREAD_COLOR)
        depth = read_image(depth_path, cv2.IMREAD_UNCHANGED)
        if rgb is None:
            raise RuntimeError(f"无法读取托盘 RGB 图：{rgb_path}")
        if depth is None:
            raise RuntimeError(f"无法读取托盘深度图：{depth_path}")
        if depth.ndim == 3 and depth.shape[2] == 1:
            depth = depth[:, :, 0]
        if depth.ndim != 2:
            raise RuntimeError(f"深度图必须是单通道，当前 shape={depth.shape}")
        if depth.dtype != np.uint16:
            raise RuntimeError(f"深度图必须是 uint16，当前 dtype={depth.dtype}")
        if tuple(rgb.shape[:2]) != tuple(depth.shape[:2]):
            raise RuntimeError(f"RGB/Depth 尺寸不一致：RGB={rgb.shape[:2]}, Depth={depth.shape[:2]}")
        return rgb, depth

    def _detect_two_boxes(self, rgb) -> list[Dict[str, Any]]:
        model = self._ensure_model()
        kwargs = {"source": rgb, "conf": self.confidence, "verbose": False}
        if self.device:
            kwargs["device"] = self.device
        results = model.predict(**kwargs)
        if not results or getattr(results[0], "boxes", None) is None:
            raise RuntimeError("未检测到托盘插孔")
        candidates = []
        for box in results[0].boxes:
            bbox = [float(v) for v in box.xyxy[0].detach().cpu().tolist()]
            conf = float(box.conf[0].detach().cpu())
            candidates.append({"bbox_xyxy": bbox, "confidence": conf})
        if len(candidates) < 2:
            raise RuntimeError(f"托盘插孔应检测到 2 个，当前仅检测到 {len(candidates)} 个")
        top_two = sorted(candidates, key=lambda x: x["confidence"], reverse=True)[:2]
        top_two.sort(key=lambda x: (x["bbox_xyxy"][0] + x["bbox_xyxy"][2]) / 2.0)
        return top_two

    def _estimate_depth_mm(self, depth, bbox) -> float:
        import numpy as np
        h_img, w_img = depth.shape
        x1, y1, x2, y2 = [int(round(v)) for v in bbox]
        x1, x2 = max(0, min(w_img, x1)), max(0, min(w_img, x2))
        y1, y2 = max(0, min(h_img, y1)), max(0, min(h_img, y2))
        if x2 <= x1 or y2 <= y1:
            raise RuntimeError(f"无效插孔框：{bbox}")
        box_h = y2 - y1
        trim = (1.0 - self.vertical_fraction) / 2.0
        ys = max(y1, min(y2 - 1, int(round(y1 + box_h * trim))))
        ye = max(ys + 1, min(y2, int(round(y2 - box_h * trim))))
        regions = (
            depth[ys:ye, max(0, x1 - self.side_width_px):x1],
            depth[ys:ye, x2:min(w_img, x2 + self.side_width_px)],
        )
        medians = []
        for roi in regions:
            if roi.size == 0:
                continue
            valid = roi[roi > 0]
            if valid.size:
                medians.append(float(np.median(valid.astype(np.float64))))
        if not medians:
            raise RuntimeError("插孔两侧没有有效深度值")
        return float(np.mean(medians))

    def _pixel_to_xyz_mm(self, u: float, v: float, z_mm: float) -> list[float]:
        fx = float(self.intrinsics["fx"]); fy = float(self.intrinsics["fy"])
        cx = float(self.intrinsics["cx"]); cy = float(self.intrinsics["cy"])
        if z_mm <= 0 or fx == 0 or fy == 0:
            raise RuntimeError("相机内参或深度无效")
        x = (float(u) - cx) * float(z_mm) / fx
        y = (float(v) - cy) * float(z_mm) / fy
        return [float(x), float(y), float(z_mm)]

    def recognize(
        self,
        rgb_path: str | Path,
        depth_path: str | Path,
        result_tag: str = "",
    ) -> Dict[str, Any]:
        rgb_path = Path(rgb_path).expanduser().resolve()
        depth_path = Path(depth_path).expanduser().resolve()
        if not rgb_path.is_file() or not depth_path.is_file():
            raise RuntimeError("托盘插孔识别需要同时提供 RGB 图和对齐的 16 位深度图")
        rgb, depth = self._read_rgb_depth(rgb_path, depth_path)
        detections = self._detect_two_boxes(rgb)
        holes = []
        for name, det in zip(("left", "right"), detections):
            x1, y1, x2, y2 = det["bbox_xyxy"]
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            z_mm = self._estimate_depth_mm(depth, det["bbox_xyxy"])
            xyz = self._pixel_to_xyz_mm(u, v, z_mm)
            holes.append({
                "name": name,
                "uv_px": [u, v],
                "xyz_mm": xyz,
                "bbox_xyxy": det["bbox_xyxy"],
                "confidence": det["confidence"],
            })
        result_dir = self._create_result_dir(result_tag=result_tag)
        result_image_path = self._save_result_image(rgb, holes, result_dir)
        return {
            "success": True,
            "recognition_source": "pallet_hole_yolo_rgbd",
            "coordinate_frame": "camera",
            "model_path": str(self.model_path),
            "rgb_path": str(rgb_path),
            "depth_path": str(depth_path),
            "result_dir": str(result_dir),
            "result_image_path": result_image_path,
            "left_xyz_mm": holes[0]["xyz_mm"],
            "right_xyz_mm": holes[1]["xyz_mm"],
            "fork_holes": holes,
            "camera_intrinsics": dict(self.intrinsics),
            "message": "托盘左右插孔已识别并转换为相机坐标系 XYZ(mm)",
        }
