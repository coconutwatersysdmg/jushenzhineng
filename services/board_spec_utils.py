# -*- coding: utf-8 -*-
"""车板 / 托盘规格工具：从角点或计划字段提取尺寸（mm）。"""
from __future__ import annotations

from typing import Any, Mapping


# 托盘规格暂无默认值；视觉读入就绪后由 resolve_pallet_specs 替换。
DEFAULT_PALLET_LENGTH_MM = 1200.0
DEFAULT_PALLET_WIDTH_MM = 1000.0
DEFAULT_PALLET_HEIGHT_MM = 900.0
DEFAULT_PALLET_STACK_LAYERS = 1  # 1=一层托盘，2=两层托盘外形


def point_xy(point: Any) -> tuple[float, float, float] | None:
    if not isinstance(point, Mapping):
        return None
    try:
        x = float(point.get("x", point.get("x_mm")))
        y = float(point.get("y", point.get("y_mm")))
        z = float(point.get("z", point.get("z_mm", 0.0)) or 0.0)
    except (TypeError, ValueError):
        return None
    return x, y, z


def board_extent_from_corners(corners: Mapping[str, Any] | None) -> dict[str, float] | None:
    """WORLD：长度≈Y 跨度，宽度≈X 跨度（与 TwinScene 一致）。"""
    pts = []
    for point in (corners or {}).values():
        xyz = point_xy(point)
        if xyz is not None:
            pts.append(xyz)
    if len(pts) < 2:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    length_mm = max(50.0, max(ys) - min(ys))
    width_mm = max(50.0, max(xs) - min(xs))
    deck_height_mm = sum(zs) / len(zs)
    return {
        "length_mm": round(length_mm, 3),
        "width_mm": round(width_mm, 3),
        "deck_height_mm": round(deck_height_mm, 3),
    }


def board_extent_from_geometry(geometry: Mapping[str, Any] | None) -> dict[str, float] | None:
    """从 board geometry sections 取平均长宽；无 sections 则退回顶层字段。"""
    geometry = geometry or {}
    sections = list(geometry.get("sections") or [])
    lengths = []
    widths = []
    for section in sections:
        try:
            if section.get("length_mm") is not None:
                lengths.append(float(section["length_mm"]))
            if section.get("width_mm") is not None:
                widths.append(float(section["width_mm"]))
        except (TypeError, ValueError):
            continue
    length_mm = None
    width_mm = None
    if lengths:
        length_mm = sum(lengths) / len(lengths)
    elif geometry.get("length_mm") is not None:
        try:
            length_mm = float(geometry["length_mm"])
        except (TypeError, ValueError):
            length_mm = None
    if widths:
        width_mm = sum(widths) / len(widths)
    elif geometry.get("width_mm") is not None:
        try:
            width_mm = float(geometry["width_mm"])
        except (TypeError, ValueError):
            width_mm = None
    if length_mm is None and width_mm is None:
        return None
    out: dict[str, float] = {}
    if length_mm is not None:
        out["length_mm"] = round(max(50.0, length_mm), 3)
    if width_mm is not None:
        out["width_mm"] = round(max(50.0, width_mm), 3)
    try:
        if geometry.get("deck_height_mm") is not None:
            out["deck_height_mm"] = round(float(geometry["deck_height_mm"]), 3)
    except (TypeError, ValueError):
        pass
    return out or None


def merge_board_specs(*parts: Mapping[str, Any] | None) -> dict[str, float]:
    """后者覆盖前者；只保留正数尺寸。"""
    out: dict[str, float] = {}
    for part in parts:
        if not part:
            continue
        for key in ("length_mm", "width_mm", "deck_height_mm"):
            if key not in part or part[key] is None:
                continue
            try:
                value = float(part[key])
            except (TypeError, ValueError):
                continue
            if value > 0:
                out[key] = round(value, 3)
    return out


def resolve_pallet_specs(cargo: Mapping[str, Any] | None) -> dict[str, Any]:
    """解析托盘/货物外轮廓规格。

    当前：计划字段 → 默认值。
    预留：将来视觉/现场读入可写入 cargo['measured_*'] 或 attributes，此处优先采用。
    """
    cargo = dict(cargo or {})
    measured = cargo.get("measured_pallet_specs") or cargo.get("pallet_specs") or {}
    if not isinstance(measured, Mapping):
        measured = {}

    def _num(*keys, default: float) -> float:
        for key in keys:
            for source in (measured, cargo):
                if key in source and source[key] is not None:
                    try:
                        value = float(source[key])
                    except (TypeError, ValueError):
                        continue
                    if value > 0:
                        return value
        return float(default)

    length = _num("length_mm", "pallet_length_mm", default=DEFAULT_PALLET_LENGTH_MM)
    width = _num("width_mm", "pallet_width_mm", default=DEFAULT_PALLET_WIDTH_MM)
    height = _num("height_mm", "pallet_height_mm", default=DEFAULT_PALLET_HEIGHT_MM)
    layers_raw = measured.get("stack_layers", cargo.get("stack_layers", DEFAULT_PALLET_STACK_LAYERS))
    try:
        layers = int(layers_raw)
    except (TypeError, ValueError):
        layers = DEFAULT_PALLET_STACK_LAYERS
    layers = 2 if layers >= 2 else 1
    ref_w = _num("pallet_reference_width_mm", "pallet_width_mm", default=length)
    return {
        "length_mm": round(length, 3),
        "width_mm": round(width, 3),
        "height_mm": round(height, 3),
        "stack_layers": layers,
        "pallet_reference_width_mm": round(ref_w, 3),
        "pallet_specs_source": str(
            measured.get("source")
            or cargo.get("pallet_specs_source")
            or ("measured" if measured else "plan_or_default")
        ),
    }
