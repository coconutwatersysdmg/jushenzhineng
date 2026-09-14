# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from threading import RLock
from typing import Any, Dict, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = PROJECT_ROOT / "examples" / "module_io_manifest.json"
EVIDENCE_ROOT = PROJECT_ROOT / "workdir" / "module_call_evidence"
SUMMARY_PATH = PROJECT_ROOT / "workdir" / "module_call_summary.json"


class ModuleEvidenceService:
    """Loads example I/O and keeps durable evidence for every algorithm call."""

    def __init__(self, manifest_path: Path = MANIFEST_PATH):
        self.manifest_path = Path(manifest_path)
        self._lock = RLock()
        raw = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        previous_rows = {}
        previous = self._read_json(SUMMARY_PATH)
        if isinstance(previous, Mapping):
            previous_rows = {str(item.get("module_id")): item for item in previous.get("modules", []) if isinstance(item, Mapping)}
        self.modules: Dict[str, Dict[str, Any]] = {}
        for item in raw.get("modules", []):
            row = deepcopy(item)
            module_id = str(row["module_id"])
            row["example_inputs"] = self._resolve_tree(row.get("example_inputs") or {})
            output_file = row.get("example_output")
            if output_file:
                output_path = self._resolve_path(output_file)
                row["example_output"] = output_path
                row["example_output_data"] = self._read_json(Path(output_path))
            row.update({
                "call_count": 0,
                "live_status": "WAITING",
                "last_call": None,
                "last_elapsed_ms": None,
                "last_inputs": None,
                "last_output": None,
                "last_evidence_path": None,
                "model_invoked": False,
                "used_by_flow": False,
                "previous_call": (previous_rows.get(module_id) or {}).get("last_call"),
                "previous_status": (previous_rows.get(module_id) or {}).get("live_status"),
                "previous_evidence_path": (previous_rows.get(module_id) or {}).get("last_evidence_path"),
            })
            self.modules[module_id] = row

    @staticmethod
    def _read_json(path: Path):
        try:
            return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None
        except Exception as exc:
            return {"load_error": str(exc), "path": str(path)}

    @staticmethod
    def _resolve_path(value: Any) -> str:
        path = Path(str(value)).expanduser()
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        return str(path.resolve())

    @classmethod
    def _resolve_tree(cls, value):
        if isinstance(value, Mapping):
            return {str(key): cls._resolve_tree(item) for key, item in value.items()}
        if isinstance(value, list):
            return [cls._resolve_tree(item) for item in value]
        if isinstance(value, str) and ("/" in value or "\\" in value):
            return cls._resolve_path(value)
        return value

    @staticmethod
    def _compact(value, depth=0):
        if depth > 3:
            return "..."
        if isinstance(value, Mapping):
            important = (
                "success", "source", "recognition_source", "algorithm", "phase", "message",
                "model_path", "checkpoint_path", "result_json_path", "result_image_path",
                "result_image_paths_by_point", "corner_xyz_m", "offset_deg", "board_count",
                "corner_ids", "decision", "overhang_percent", "should_fork", "world_points",
                "board_mode", "available_count", "compensation_world_mm",
                "next_pallet_compensation_world_mm",
            )
            keys = [key for key in important if key in value]
            if not keys:
                keys = list(value)[:12]
            return {key: ModuleEvidenceService._compact(value[key], depth + 1) for key in keys}
        if isinstance(value, (list, tuple)):
            return [ModuleEvidenceService._compact(item, depth + 1) for item in list(value)[:12]]
        text = str(value)
        return text if len(text) <= 600 else text[:597] + "..."

    def record(self, module_id: str, inputs: Mapping[str, Any], output: Any, elapsed_ms: float,
               status: str = "SUCCESS", model_invoked: bool = False, used_by_flow: bool = True,
               note: str = "") -> Dict[str, Any]:
        with self._lock:
            if module_id not in self.modules:
                raise KeyError(f"未登记模块：{module_id}")
            now = datetime.now()
            stamp = now.strftime("%Y%m%d_%H%M%S_%f")
            row = self.modules[module_id]
            evidence = {
                "evidence_id": f"{module_id}-{stamp}",
                "module_id": module_id,
                "module_name": row.get("name"),
                "time": now.isoformat(timespec="milliseconds"),
                "status": str(status).upper(),
                "used_by_flow": bool(used_by_flow),
                "model_invoked": bool(model_invoked),
                "model_or_implementation": row.get("implementation"),
                "model_path": self._resolve_path(row["model_path"]) if row.get("model_path") else "",
                "elapsed_ms": round(float(elapsed_ms), 3),
                "inputs": deepcopy(inputs),
                "output": deepcopy(output),
                "note": str(note or ""),
            }
            output_dir = EVIDENCE_ROOT / module_id
            output_dir.mkdir(parents=True, exist_ok=True)
            evidence_path = output_dir / f"{stamp}.json"
            evidence["evidence_path"] = str(evidence_path.resolve())
            evidence_path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
            row["call_count"] += 1
            row["live_status"] = evidence["status"]
            row["last_call"] = evidence["time"]
            row["last_elapsed_ms"] = evidence["elapsed_ms"]
            row["last_inputs"] = self._compact(inputs)
            row["last_output"] = self._compact(output)
            row["last_evidence_path"] = evidence["evidence_path"]
            row["model_invoked"] = bool(row.get("model_invoked") or model_invoked)
            row["used_by_flow"] = bool(row.get("used_by_flow") or used_by_flow)
            self._save_summary()
            return deepcopy(evidence)

    def _save_summary(self):
        SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
        SUMMARY_PATH.write_text(json.dumps({"modules": list(self.modules.values())}, ensure_ascii=False, indent=2, default=str), encoding="utf-8")

    def snapshot(self):
        with self._lock:
            return deepcopy(list(self.modules.values()))

    def example_debug_inputs(self) -> Dict[str, Any]:
        dynamic = self.modules["DYNAMIC_PRE_PLACE"]["example_inputs"]
        return {
            "point_cloud_path": self.modules["POINTNET_TRUCK"]["example_inputs"]["pcd_path"],
            # The supplied offline PCD is executed to prove PointNet++ inference,
            # but its sensor/world calibration does not belong to this scene.
            # Keep it as evidence instead of sending its coordinates to motion.
            "point_cloud_example_only": True,
            # Example RGB-D may prove the corner model call, but its camera
            # calibration does not belong to this live scene. Keep scene
            # geometry aligned to the demo truck instead of treating those
            # pixels/depth values as calibrated production WORLD points.
            "corner_inputs_example_only": True,
            "pallet_rgb": self.modules["PALLET_HOLE_YOLO"]["example_inputs"]["rgb_path"],
            "pallet_depth": self.modules["PALLET_HOLE_YOLO"]["example_inputs"]["depth_path"],
            "images": {
                "pre_pick_offset": self.modules["PALLET_OVERHANG_PRE"]["example_inputs"]["image_path"],
                "neighbor_pose": self.modules["NEIGHBOR_POSE"]["example_inputs"]["image_path"],
                "region_deviation": self.modules["REGION_DEVIATION"]["example_inputs"]["image_path"],
                "face_a": self.modules["TWO_FACE_CAPTURE"]["example_inputs"]["face_a"],
                "face_b": self.modules["TWO_FACE_CAPTURE"]["example_inputs"]["face_b"],
                "post_place_offset": self.modules["PALLET_CARGO_POST"]["example_inputs"]["image_path"],
                "pre_place_monitor": dynamic["rgb_path"],
                "post_place_bottom_pallet": self.modules["DYNAMIC_POST_PLACE"]["example_inputs"]["rgb_path"],
            },
            "depths": {
                "pre_place_monitor": dynamic["depth_path"],
                "post_place_bottom_pallet": self.modules["DYNAMIC_POST_PLACE"]["example_inputs"]["depth_path"],
                "pallet_hole": self.modules["PALLET_HOLE_YOLO"]["example_inputs"]["depth_path"],
            },
            "dynamic_camera_path": dynamic["camera_path"],
            "corner_images": deepcopy(self.modules["CORNER_YOLO"]["example_inputs"]["image_paths_by_point"]),
            "corner_depths": deepcopy(self.modules["CAMERA_WORLD"]["example_inputs"]["depth_paths_by_point"]),
            "neighbor_pose_measurement": {"dx_mm": 4.0, "dy_mm": -3.0, "dz_mm": 0.0, "yaw_deg": 0.25},
            "post_region_measurement": {"dx_mm": -5.0, "dy_mm": 3.0, "dz_mm": 0.0, "yaw_deg": -0.2},
        }
