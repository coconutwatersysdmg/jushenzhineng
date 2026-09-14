# -*- coding: utf-8 -*-
"""同一套托盘/货物横向偏移算法的双阶段系统适配。

- pre_pick: 输出偏移比例 + ACCEPT/REJECT + should_fork，REJECT 作为硬门禁。
- post_place: 仍调用同一检测核心，但输出货物相对底层托盘的中心偏移距离、边缘偏移和超出距离。

原始算法文件保持不改，便于与用户提供模块核对；这里使用其同一组内部边界检测函数扩展出
系统所需的几何量。
"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

from algorithm_modules.pallet_overhang_detection_module import pallet_overhang_detection as algo
from config.system_config import get_system_config
from utils.cv_io import read_image

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULT_ROOT = PROJECT_ROOT / "workdir" / "pallet_cargo_offset_results"


class PalletCargoOffsetService:
    def __init__(self, config_path: Path | None = None, result_root: Path = RESULT_ROOT):
        # config_path 保留仅为兼容旧调用；配置已改为 config/system_config.py
        self.config_path = Path(config_path) if config_path else PROJECT_ROOT / "config" / "system_config.py"
        self.result_root = Path(result_root)
        self.config = self._load_config()

    def _load_config(self) -> Dict[str, Any]:
        return get_system_config()

    @property
    def cfg(self) -> algo.Config:
        section = self.config.get("pallet_cargo_offset", {}) or {}
        threshold = float(section.get("max_overhang_percent", algo.DEFAULT_CONFIG.MAX_OVERHANG_PERCENT))
        # 仅覆盖门限，其余图像分割参数严格沿用用户模块默认值。
        return algo.Config(MAX_OVERHANG_PERCENT=threshold)

    def _analyze_detail(self, image_path: str | Path) -> Dict[str, Any]:
        """复用用户模块同一套边界提取逻辑，补充模块内部已计算但原 JSON 未暴露的几何量。"""
        path = Path(image_path).expanduser().resolve()
        image = read_image(path)
        if image is None:
            raise RuntimeError(f"偏移检测图片读取失败：{path}")

        cfg = self.cfg
        h = image.shape[0]
        pallets = algo._detect_blue_pallets(image, cfg)
        if len(pallets) < 2:
            raise RuntimeError(f"有效蓝色托盘不足：只检测到 {len(pallets)} 个")

        bottom = pallets[-1]
        middle = pallets[-2]
        white_mask = algo._make_white_mask(image, cfg)
        margin = cfg.GOODS_VERTICAL_MARGIN_PX

        top_left, top_right = algo._detect_goods_pair(
            white_mask,
            middle,
            int(round(h * cfg.TOP_SEARCH_START_RATIO)),
            middle["top"] - margin,
            cfg,
        )
        lower_left, lower_right = algo._detect_goods_pair(
            white_mask,
            middle,
            middle["bottom"] + margin,
            bottom["top"] - margin,
            cfg,
        )

        left_candidates = [x for x in (top_left, middle["left"], lower_left) if x is not None]
        right_candidates = [x for x in (top_right, middle["right"], lower_right) if x is not None]
        if not left_candidates or not right_candidates:
            raise RuntimeError("上部对象左右边界检测失败")

        upper_left = int(min(left_candidates))
        upper_right = int(max(right_candidates))
        bottom_left = int(bottom["left"])
        bottom_right = int(bottom["right"])
        bottom_width_px = float(bottom_right - bottom_left)
        if bottom_width_px <= 1:
            raise RuntimeError("底层托盘像素宽度异常")

        left_overhang_px = float(max(0, bottom_left - upper_left))
        right_overhang_px = float(max(0, upper_right - bottom_right))
        max_overhang_px = max(left_overhang_px, right_overhang_px)
        overhang_percent = 100.0 * max_overhang_px / bottom_width_px

        pallet_center_px = (bottom_left + bottom_right) / 2.0
        cargo_center_px = (upper_left + upper_right) / 2.0
        center_offset_px = cargo_center_px - pallet_center_px
        left_edge_delta_px = float(upper_left - bottom_left)
        right_edge_delta_px = float(upper_right - bottom_right)

        decision = "REJECT" if overhang_percent > cfg.MAX_OVERHANG_PERCENT else "ACCEPT"
        return {
            "image_path": str(path),
            "overhang_percent": round(float(overhang_percent), 3),
            "decision": decision,
            "threshold_percent": float(cfg.MAX_OVERHANG_PERCENT),
            "bottom_pallet": {
                "left_px": bottom_left,
                "right_px": bottom_right,
                "width_px": round(bottom_width_px, 3),
                "center_x_px": round(pallet_center_px, 3),
            },
            "upper_object": {
                "left_px": upper_left,
                "right_px": upper_right,
                "center_x_px": round(cargo_center_px, 3),
            },
            "left_overhang_px": round(left_overhang_px, 3),
            "right_overhang_px": round(right_overhang_px, 3),
            "max_overhang_px": round(max_overhang_px, 3),
            "center_offset_px": round(float(center_offset_px), 3),
            "absolute_center_offset_px": round(abs(float(center_offset_px)), 3),
            "left_edge_delta_px": round(left_edge_delta_px, 3),
            "right_edge_delta_px": round(right_edge_delta_px, 3),
        }

    def _reference_width_mm(self, cargo: Optional[Dict[str, Any]]) -> Optional[float]:
        cargo = cargo or {}
        for key in ("pallet_reference_width_mm", "pallet_width_mm"):
            raw = cargo.get(key)
            if raw not in (None, ""):
                try:
                    value = float(raw)
                    if value > 0:
                        return value
                except Exception:
                    pass
        raw = (self.config.get("pallet_cargo_offset", {}) or {}).get("reference_pallet_width_mm")
        if raw not in (None, ""):
            try:
                value = float(raw)
                if value > 0:
                    return value
            except Exception:
                pass
        return None

    def _save(self, result: Dict[str, Any], result_tag: str, phase: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(result_tag or "cargo"))
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        out_dir = self.result_root / safe
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"{phase}_{stamp}.json"
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
        return str(path)

    def detect(
        self,
        image_path: str | Path,
        phase: str,
        cargo: Optional[Dict[str, Any]] = None,
        result_tag: str = "cargo",
    ) -> Dict[str, Any]:
        phase = str(phase or "").strip().lower()
        if phase not in {"pre_pick", "post_place"}:
            raise ValueError("phase 只支持 pre_pick 或 post_place")

        detail = self._analyze_detail(image_path)
        base = {
            "analysis_success": True,
            "algorithm": "pallet_overhang_detection_module",
            "phase": phase,
            "image_path": detail["image_path"],
            "overhang_percent": detail["overhang_percent"],
            "threshold_percent": detail["threshold_percent"],
            "decision": detail["decision"],
        }

        if phase == "pre_pick":
            should_fork = detail["decision"] == "ACCEPT"
            result = {
                **base,
                # 流程门禁：REJECT 时 success=False，从而禁止进入“插取托盘”。
                "success": should_fork,
                "should_fork": should_fork,
                "left_overhang_px": detail["left_overhang_px"],
                "right_overhang_px": detail["right_overhang_px"],
                "max_overhang_px": detail["max_overhang_px"],
                "message": (
                    f"插取前横向偏移 {detail['overhang_percent']:.3f}%：允许插取"
                    if should_fork
                    else f"插取前横向偏移 {detail['overhang_percent']:.3f}%：超过 {detail['threshold_percent']:.3f}% 门限，禁止插取"
                ),
            }
        else:
            reference_width_mm = self._reference_width_mm(cargo)
            scale_mm_per_px = None
            if reference_width_mm is not None:
                scale_mm_per_px = reference_width_mm / float(detail["bottom_pallet"]["width_px"])

            def mm(px: float):
                return None if scale_mm_per_px is None else round(float(px) * scale_mm_per_px, 3)

            signed_center_mm = mm(detail["center_offset_px"])
            result = {
                **base,
                # 放置后阶段只表示“测量是否成功”，不把原 ACCEPT/REJECT 当作插取门禁。
                "success": True,
                "offset_distance_px": detail["absolute_center_offset_px"],
                "signed_center_offset_px": detail["center_offset_px"],
                "left_edge_delta_px": detail["left_edge_delta_px"],
                "right_edge_delta_px": detail["right_edge_delta_px"],
                "left_overhang_px": detail["left_overhang_px"],
                "right_overhang_px": detail["right_overhang_px"],
                "max_overhang_px": detail["max_overhang_px"],
                "reference_pallet_width_mm": reference_width_mm,
                "scale_mm_per_px": None if scale_mm_per_px is None else round(scale_mm_per_px, 6),
                "offset_distance_mm": None if signed_center_mm is None else round(abs(signed_center_mm), 3),
                "signed_center_offset_mm": signed_center_mm,
                "left_edge_delta_mm": mm(detail["left_edge_delta_px"]),
                "right_edge_delta_mm": mm(detail["right_edge_delta_px"]),
                "max_overhang_mm": mm(detail["max_overhang_px"]),
                "decision_reference_only": detail["decision"],
                "message": (
                    f"放置后货物/托盘中心偏移 {abs(signed_center_mm):.3f} mm"
                    if signed_center_mm is not None
                    else f"放置后货物/托盘中心偏移 {detail['absolute_center_offset_px']:.3f} px；未配置托盘实物参考宽度，暂不换算 mm"
                ),
            }

        result["result_json_path"] = self._save(result, result_tag=result_tag, phase=phase)
        return result
