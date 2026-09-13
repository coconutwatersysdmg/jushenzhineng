# -*- coding: utf-8 -*-
"""WORLD 位姿 → 龙门架 PLC 轴坐标 XYZR。

配置来自 config/external_devices_config.py → GANTRY.world_to_gantry。
现场标定后把 placeholder 改为 False，并填真实零点/比例。
"""
from __future__ import annotations

from typing import Any, Mapping


class WorldToGantryError(ValueError):
    """映射配置不可用或换算失败。"""


def transform_world_to_gantry(pose: Mapping[str, Any], mapping: Mapping[str, Any] | None) -> dict[str, float]:
    """把软件 WORLD 位姿换成 PLC 轴目标 ``{X,Y,Z,R}``。"""
    cfg = dict(mapping or {})
    if not cfg:
        raise WorldToGantryError("world_to_gantry 为空，请先在 config/external_devices_config.py 填写映射")
    if bool(cfg.get("placeholder", False)):
        raise WorldToGantryError(
            "world_to_gantry.placeholder=True（仍是占位假数据）。"
            "现场标定后改成 False 并填入真实零点/比例，再开 allow_real_motion。"
        )

    world_origin = dict(cfg.get("world_origin_mm") or {})
    gantry_origin = dict(cfg.get("gantry_origin_xyzr") or {})
    scale = dict(cfg.get("scale") or {})

    wx = float(pose.get("x_mm", pose.get("x", 0.0)))
    wy = float(pose.get("y_mm", pose.get("y", 0.0)))
    wz = float(pose.get("z_mm", pose.get("z", 0.0)))
    yaw = float(pose.get("yaw_deg", pose.get("yaw", pose.get("R", 0.0))))

    ox = float(world_origin.get("x", 0.0))
    oy = float(world_origin.get("y", 0.0))
    oz = float(world_origin.get("z", 0.0))

    gx = float(gantry_origin.get("X", gantry_origin.get("x", 0.0)))
    gy = float(gantry_origin.get("Y", gantry_origin.get("y", 0.0)))
    gz = float(gantry_origin.get("Z", gantry_origin.get("z", 0.0)))
    gr = float(gantry_origin.get("R", gantry_origin.get("r", 0.0)))

    sx = float(scale.get("X", scale.get("x", 1.0)))
    sy = float(scale.get("Y", scale.get("y", 1.0)))
    sz = float(scale.get("Z", scale.get("z", 1.0)))
    sr = float(scale.get("R", scale.get("r", 1.0)))

    return {
        "X": gx + sx * (wx - ox),
        "Y": gy + sy * (wy - oy),
        "Z": gz + sz * (wz - oz),
        "R": gr + sr * yaw,
    }
