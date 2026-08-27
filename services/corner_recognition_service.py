# -*- coding: utf-8 -*-
"""基于 Ultralytics .pt 模型的车板角点视觉识别服务。

当前 v8 定义：
- 雷达只给 4/6 个粗 WORLD 搜索点；
- 唯一机械臂沿车身左外侧轨道运动，车尾/车头各拍一组 RGB-D；
- 同一端的多个角点可来自同一张 JPG，.pt 输出多个 bbox 中心像素 (u,v)；
- 本服务只负责视觉像素识别和结果图保存，不再把雷达 XYZ 当成最终角点；
- 最终 WORLD XYZ 由 CameraCornerWorldService 使用 (u,v)+同步深度+动态 T_world_camera 计算。
"""
from __future__ import annotations

import os
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Union


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "models" / "corner_service.pt"
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "runtime" / "recognition_results" / "corner"
ALL_POINT_NAMES = tuple(f"P{i}" for i in range(1, 9))


class CornerRecognitionService:
    def __init__(
        self,
        model_path: Optional[Union[str, Path]] = None,
        confidence: float = 0.20,
        imgsz: int = 640,
        device: Optional[str] = None,
        result_root: Optional[Union[str, Path]] = None,
    ) -> None:
        configured = model_path or os.environ.get("CORNER_MODEL_PATH") or DEFAULT_MODEL_PATH
        self.model_path = Path(configured).expanduser().resolve()
        self.confidence = float(confidence)
        self.imgsz = int(imgsz)
        self.device = device or os.environ.get("CORNER_MODEL_DEVICE") or None
        self.result_root = Path(result_root or DEFAULT_RESULT_ROOT).expanduser().resolve()
        self._model = None

    @staticmethod
    def _safe_tag(value: str) -> str:
        text = re.sub(r"[^0-9A-Za-z._-]+", "_", str(value or "").strip())
        return text.strip("._-") or "corner"

    def _create_result_dir(self, result_tag: str = "") -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        folder = self.result_root / f"{self._safe_tag(result_tag)}_{stamp}"
        folder.mkdir(parents=True, exist_ok=True)
        return folder

    def _ensure_model(self):
        if self._model is not None:
            return self._model
        if not self.model_path.is_file():
            raise RuntimeError(f"角点模型不存在：{self.model_path}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("缺少 ultralytics，无法加载角点 .pt 模型") from exc
        self._model = YOLO(str(self.model_path))
        return self._model

    @staticmethod
    def _as_existing_path(value: Any) -> Optional[Path]:
        if not value:
            return None
        path = Path(str(value)).expanduser()
        if not path.is_absolute():
            path = (PROJECT_ROOT / path).resolve()
        return path if path.is_file() else None

    def _predict_one(self, image_path: Union[str, Path]) -> List[Dict[str, Any]]:
        path = self._as_existing_path(image_path)
        if path is None:
            raise RuntimeError(f"角点识别输入图片不存在：{image_path}")
        model = self._ensure_model()
        kwargs: Dict[str, Any] = {
            "source": str(path),
            "conf": self.confidence,
            "imgsz": self.imgsz,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device
        results = model.predict(**kwargs)
        if not results:
            return []
        result = results[0]
        orig_shape = getattr(result, "orig_shape", None)
        image_size = [int(orig_shape[1]), int(orig_shape[0])] if orig_shape and len(orig_shape) >= 2 else None
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = boxes.xyxy.detach().cpu().tolist()
        confs = boxes.conf.detach().cpu().tolist()
        classes = boxes.cls.detach().cpu().tolist()
        names = getattr(result, "names", None) or getattr(model, "names", {}) or {}
        has_point = isinstance(names, dict) and any(str(v).lower() == "point" for v in names.values())

        detections: List[Dict[str, Any]] = []
        for bbox, confidence, class_id in zip(xyxy, confs, classes):
            cid = int(class_id)
            class_name = str(names.get(cid, cid)) if isinstance(names, dict) else str(cid)
            if has_point and class_name.lower() != "point":
                continue
            x1, y1, x2, y2 = [float(v) for v in bbox]
            detections.append({
                "x": (x1 + x2) / 2.0,
                "y": (y1 + y2) / 2.0,
                "confidence": float(confidence),
                "class_id": cid,
                "class_name": class_name,
                "bbox_xyxy": [x1, y1, x2, y2],
                "image": str(path),
                "image_size_px": image_size,
            })
        detections.sort(key=lambda row: row["confidence"], reverse=True)
        return detections

    @staticmethod
    def _normalize_sources(value: Any) -> List[Any]:
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes, Path)):
            return [x for x in value if x]
        return [value] if value else []

    @staticmethod
    def _save_annotated_result_image(detection: Mapping[str, Any], point_name: str, output_dir: Path) -> str:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("保存角点识别结果图需要 opencv-python") from exc
        source = Path(str(detection.get("image") or ""))
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"无法读取角点结果图源文件：{source}")
        x1, y1, x2, y2 = [int(round(float(v))) for v in detection["bbox_xyxy"]]
        cx, cy = int(round(float(detection["x"]))), int(round(float(detection["y"])))
        conf = float(detection.get("confidence", 0.0))
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.drawMarker(image, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
        cv2.putText(
            image,
            f"{point_name} conf={conf:.3f} uv=({cx},{cy})",
            (max(4, x1), max(22, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        output_path = output_dir / f"{point_name}_result.jpg"
        if not cv2.imwrite(str(output_path), image):
            raise RuntimeError(f"角点识别结果图保存失败：{output_path}")
        return str(output_path.resolve())

    @staticmethod
    def _derive_expected_corner_ids(
        radar_result: Optional[Mapping[str, Any]],
        image_source: Optional[Mapping[str, Any]] = None,
    ) -> List[str]:
        radar = radar_result or {}
        ids = radar.get("corner_ids")
        if isinstance(ids, (list, tuple)) and ids:
            return [str(x) for x in ids]
        world = radar.get("world_points")
        if isinstance(world, Mapping) and world:
            names = [name for name in ALL_POINT_NAMES if name in world]
            if names:
                return names
        if isinstance(image_source, Mapping):
            names = [name for name in ALL_POINT_NAMES if image_source.get(name)]
            if names:
                return names
        return ["P1", "P2", "P3", "P4"]

    def recognize_image_points(
        self,
        image_source: Mapping[str, Any],
        point_names: Sequence[str],
        output_dir: Optional[Path] = None,
    ) -> Dict[str, Dict[str, Any]]:
        if not isinstance(image_source, Mapping):
            raise RuntimeError("角点输入必须按 Pn 分组，例如 {'P1':'p1.jpg', ...}")
        image_points: Dict[str, Dict[str, Any]] = {}
        # One physical endpoint image is referenced by every point visible in
        # that shot. Run inference once, then assign detections left-to-right
        # to the ordered Pn labels for that endpoint.
        grouped_single_sources: Dict[str, List[str]] = {}
        for point_name in point_names:
            sources = self._normalize_sources(image_source.get(point_name))
            if len(sources) == 1:
                grouped_single_sources.setdefault(str(sources[0]), []).append(str(point_name))
        for source, grouped_names in grouped_single_sources.items():
            if len(grouped_names) <= 1:
                continue
            detections = self._predict_one(source)
            if len(detections) < len(grouped_names):
                raise RuntimeError(f"共用端点图 {source} 需要 {len(grouped_names)} 个 point，实际检测 {len(detections)} 个")
            ordered_names=sorted(grouped_names,key=lambda name:int(name[1:]) if name[1:].isdigit() else 999)
            ordered_detections=sorted(detections,key=lambda row:(float(row.get("x",0.0)),float(row.get("y",0.0))))[:len(ordered_names)]
            for point_name,detection in zip(ordered_names,ordered_detections):
                best=dict(detection)
                best.update({"status":"模型识别","point_name":point_name,"candidate_image_count":1,"shared_endpoint_image":True})
                if output_dir is not None:
                    best["result_image"]=self._save_annotated_result_image(best,point_name,output_dir)
                image_points[point_name]=best
        for point_name in point_names:
            if point_name in image_points:
                continue
            sources = self._normalize_sources(image_source.get(point_name))
            if not sources:
                raise RuntimeError(f"缺少 {point_name} 所在车尾/车头组图")
            candidates: List[Dict[str, Any]] = []
            for source in sources:
                detections = self._predict_one(source)
                if detections:
                    candidates.append(dict(detections[0]))
            if not candidates:
                raise RuntimeError(f"{point_name} 的局部图均未检测到 point")
            best = max(candidates, key=lambda row: float(row.get("confidence", 0.0)))
            best.update({
                "status": "模型识别",
                "point_name": point_name,
                "candidate_image_count": len(sources),
            })
            if output_dir is not None:
                best["result_image"] = self._save_annotated_result_image(best, point_name, output_dir)
            image_points[point_name] = best
        return image_points

    def recognize_visual_only(
        self,
        image_source: Mapping[str, Any],
        expected_corner_ids: Sequence[str],
        result_tag: str = "cargo",
    ) -> Dict[str, Any]:
        point_names = [str(x) for x in expected_corner_ids]
        if len(point_names) not in {4, 6}:
            raise RuntimeError(f"视觉角点只支持4/6点，当前：{point_names}")
        result_dir = self._create_result_dir(result_tag)
        image_points = self.recognize_image_points(image_source, point_names, output_dir=result_dir)
        physical_images=len({str(path) for value in image_source.values() for path in self._normalize_sources(value)})
        return {
            "success": True,
            "recognition_source": "pt_model_visual_only",
            "model_path": str(self.model_path),
            "corner_ids": point_names,
            "image_points": image_points,
            "result_dir": str(result_dir),
            "result_image_paths_by_point": {name: image_points[name].get("result_image", "") for name in point_names},
            "physical_image_count":physical_images,
            "message": f"{physical_images} 组端点图已完成 {len(point_names)} 个角点 .pt 像素识别；WORLD坐标尚未计算",
        }

    @staticmethod
    def _normalize_world_point(value: Any) -> Optional[Dict[str, float]]:
        if isinstance(value, Mapping) and "x" in value and "y" in value:
            return {"x": float(value["x"]), "y": float(value["y"]), "z": float(value.get("z", 0.0))}
        if isinstance(value, (list, tuple)) and len(value) >= 3:
            return {"x": float(value[0]), "y": float(value[1]), "z": float(value[2])}
        return None

    @classmethod
    def build_world_points(
        cls,
        radar_result: Optional[Mapping[str, Any]],
        point_names: Sequence[str],
    ) -> Dict[str, Dict[str, float]]:
        radar = radar_result or {}
        raw = radar.get("world_points")
        if isinstance(raw, Mapping):
            result = {}
            for name in point_names:
                point = cls._normalize_world_point(raw.get(name))
                if point is not None:
                    result[name] = point
            if len(result) == len(point_names):
                return result

        raw_list = radar.get("corner_points") or []
        if isinstance(raw_list, (list, tuple)):
            by_name = {}
            for item in raw_list:
                if isinstance(item, Mapping):
                    name = str(item.get("name") or "")
                    point = cls._normalize_world_point(item)
                    if name and point is not None:
                        by_name[name] = point
            if all(name in by_name for name in point_names):
                return {name: by_name[name] for name in point_names}

        raise RuntimeError(
            f"视觉模型已识别 {len(point_names)} 个图像角点，但点云结果没有对应完整 3D 坐标：{list(point_names)}"
        )

    def recognize(
        self,
        image_source: Mapping[str, Any],
        radar_result: Optional[Mapping[str, Any]] = None,
        plate_no: str = "",
        result_tag: str = "",
        expected_corner_ids: Optional[Sequence[str]] = None,
    ) -> Dict[str, Any]:
        point_names = list(expected_corner_ids or self._derive_expected_corner_ids(radar_result, image_source))
        if len(point_names) < 4:
            raise RuntimeError(f"车板角点至少需要 4 个，当前仅得到：{point_names}")

        result_dir = self._create_result_dir(result_tag or plate_no)
        image_points = self.recognize_image_points(image_source, point_names, output_dir=result_dir)
        world_points = self.build_world_points(radar_result, point_names)
        xyz = [[world_points[n]["x"], world_points[n]["y"], world_points[n]["z"]] for n in point_names]
        image_paths = {n: image_points[n].get("image", "") for n in point_names}
        result_images = {n: image_points[n].get("result_image", "") for n in point_names}
        radar = dict(radar_result or {})

        return {
            "success": True,
            "recognition_source": "pt_model_visual_confirmation",
            "model_path": str(self.model_path),
            "plate_no": plate_no,
            "confirmed_corner_ids": point_names,
            "corner_image_count": len(point_names),
            "image_paths_by_point": image_paths,
            "result_image_paths_by_point": result_images,
            "result_dir": str(result_dir),
            "image_points": image_points,
            "coordinate_frame": radar.get("coordinate_frame") or "radar_world",
            "coordinate_unit": radar.get("coordinate_unit") or "mm",
            "world_points": world_points,
            "corner_points_xyz": xyz,
            "corner_points_xyz_mm": xyz if str(radar.get("coordinate_unit") or "mm").lower() == "mm" else None,
            "outline_points": [{"name": n, **world_points[n]} for n in point_names],
            "board_count": radar.get("board_count"),
            "board_mode": radar.get("board_mode"),
            "boards": radar.get("boards") or [],
            "first_workface_loading_plan": radar.get("first_workface_loading_plan"),
        }
