# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from time import perf_counter
from typing import Any, Dict

import cv2

from algorithm_modules.dynamic_monitoring_module import detect_from_files


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULT_ROOT = PROJECT_ROOT / "runtime" / "dynamic_monitoring_results"


class DynamicMonitoringService:
    """System adapter for pre-place monitoring and post-place bottom-pallet detection."""

    def __init__(self, result_root: Path = RESULT_ROOT):
        self.result_root = Path(result_root)

    @staticmethod
    def _safe(value: str) -> str:
        text = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(value or "monitor"))
        return text.strip("_-") or "monitor"

    def analyze(self, rgb_path: str, depth_path: str, camera_path: str, phase: str, result_tag: str) -> Dict[str, Any]:
        phase = str(phase or "").strip().lower()
        if phase not in {"pre_place", "post_place_bottom_pallet"}:
            raise ValueError(f"不支持的动态监测阶段：{phase}")
        started = perf_counter()
        result = detect_from_files(rgb_path, depth_path, camera_path)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output_dir = self.result_root / self._safe(result_tag) / f"{phase}_{stamp}"
        output_dir.mkdir(parents=True, exist_ok=True)

        image = cv2.imread(str(Path(rgb_path).resolve()), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"无法读取动态监测结果图源文件：{rgb_path}")
        bx1, by1, bx2, by2 = result["bottom_edge_xyxy_px"]
        px1, py1, px2, py2 = result["board_edge_xyxy_px"]
        cx, cy = [int(round(value)) for value in result["corner_uv_color_px"]]
        cv2.line(image, (bx1, by1), (bx2, by2), (0, 255, 255), 4)
        cv2.line(image, (px1, py1), (px2, py2), (255, 180, 20), 3)
        cv2.drawMarker(image, (cx, cy), (20, 20, 255), cv2.MARKER_CROSS, 36, 4)
        cv2.putText(image, f"{phase} offset={result['offset_deg']:.3f} deg", (28, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (60, 255, 255), 2, cv2.LINE_AA)
        image_path = output_dir / "dynamic_monitor_result.jpg"
        if not cv2.imwrite(str(image_path), image, [cv2.IMWRITE_JPEG_QUALITY, 94]):
            raise RuntimeError(f"动态监测结果图写入失败：{image_path}")

        result.update({
            "phase": phase,
            "rgb_path": str(Path(rgb_path).resolve()),
            "depth_path": str(Path(depth_path).resolve()),
            "camera_path": str(Path(camera_path).resolve()),
            "result_image_path": str(image_path.resolve()),
            "elapsed_ms": round((perf_counter() - started) * 1000.0, 3),
            "message": (
                "放置前 RGB-D 动态监测完成"
                if phase == "pre_place"
                else "放置后底层托盘 RGB-D 检测完成"
            ),
        })
        json_path = output_dir / "result.json"
        result["result_json_path"] = str(json_path.resolve())
        json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        return result
