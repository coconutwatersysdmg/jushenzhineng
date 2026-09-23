# -*- coding: utf-8 -*-
"""从 round_data 汇总实验室相机照片与识别摘要（供 UI / 测试）。"""
from __future__ import annotations

from typing import Any, Mapping


def _corner_summary(world_points: Mapping[str, Any] | None) -> str:
    points = world_points or {}
    parts = []
    for pid in ("P1", "P2", "P3", "P4"):
        point = points.get(pid) or {}
        if not point:
            continue
        try:
            parts.append(
                f"{pid}=({float(point.get('x', 0)):.1f},{float(point.get('y', 0)):.1f},{float(point.get('z', 0)):.1f})"
            )
        except (TypeError, ValueError):
            continue
    return "；".join(parts)


def build_lab_camera_gallery_entries(
    round_data: Mapping[str, Any] | None,
    *,
    round_index: int = 0,
) -> list[dict[str, Any]]:
    """按流程顺序拼出「照片 + 识别摘要」条目。"""
    rd = dict(round_data or {})
    # round_index 按调用方约定：通常传「第几轮」的展示编号（从 1 起）
    round_no = int(round_index) if int(round_index) > 0 else 1
    entries: list[dict[str, Any]] = []

    pick = rd.get("pick_result") or {}
    pick_image = str(pick.get("image_path") or "")
    if pick_image:
        entries.append(
            {
                "image_path": pick_image,
                "label": f"第{round_no}轮 · 插孔识别",
                "summary": str(pick.get("message") or ""),
                "step": "pick_hole",
            }
        )

    corner = rd.get("lab_corner_shell") or {}
    camera_points = (corner.get("camera_result") or {}).get("world_points") or corner.get("camera_world_corners") or {}
    for shot in corner.get("capture_log") or []:
        if not isinstance(shot, Mapping):
            continue
        image = str(shot.get("rgb_path") or "")
        if not image:
            continue
        pair = list(shot.get("pair") or [])
        pair_text = "/".join(str(x) for x in pair) if pair else str(shot.get("group") or "")
        entries.append(
            {
                "image_path": image,
                "label": f"第{round_no}轮 · 底板角点 {pair_text}",
                "summary": _corner_summary(camera_points) or str(shot.get("message") or ""),
                "step": "corner_capture",
                "group": shot.get("group"),
            }
        )

    for key, default_label in (
        ("lab_pre_place_monitor", "放货前监测"),
        ("lab_place_verify", "放货后检测"),
    ):
        item = rd.get(key) or {}
        image = str(item.get("result_image_path") or item.get("image_path") or "")
        if not image:
            continue
        region = str(item.get("planned_region_id") or (item.get("target_region") or {}).get("region_id") or "B")
        status = str(item.get("status") or "")
        message = str(item.get("message") or "")
        summary_bits = [bit for bit in (status, message) if bit]
        entries.append(
            {
                "image_path": image,
                "label": f"第{round_no}轮 · {region} {default_label}",
                "summary": "｜".join(summary_bits),
                "step": key,
                "region_id": region,
            }
        )

    # 若流程已实时写入 gallery，优先保留时间序（去重 image_path 后追加缺失项）
    live = rd.get("lab_camera_gallery")
    if isinstance(live, list) and live:
        seen = {str(e.get("image_path") or "") for e in entries if e.get("image_path")}
        merged = list(entries)
        for item in live:
            if not isinstance(item, Mapping):
                continue
            path = str(item.get("image_path") or item.get("result_image_path") or "")
            if not path or path in seen:
                continue
            seen.add(path)
            merged.append(
                {
                    "image_path": path,
                    "label": str(item.get("title") or item.get("step") or "相机照片"),
                    "summary": str(item.get("message") or item.get("status") or ""),
                    "step": item.get("step"),
                }
            )
        return merged
    return entries
