# -*- coding: utf-8 -*-
"""plc_console 真机运动：复用 devices.gantry_modbus_motion。"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping

from devices.gantry_modbus_motion import GantryModbusMotion
from devices.world_to_gantry import WorldToGantryError, transform_world_to_gantry


class PlcMotionRunner:
    def __init__(self, config: Mapping[str, Any] | None = None):
        cfg = dict(config or {})
        plc = dict(cfg.get("plc") or {})
        gantry = dict(cfg.get("gantry") or {})
        self.host = str(plc.get("ip") or plc.get("plc_ip") or "192.168.6.6")
        self.port = int(plc.get("port") or plc.get("plc_port") or 502)
        self.default_speed = float(gantry.get("default_speed", 30.0))
        self.move_timeout_s = float(gantry.get("move_timeout_s", 60.0))
        self.soft_limits = dict(gantry.get("soft_limits") or {})
        self.world_to_gantry = dict(gantry.get("world_to_gantry") or {})
        self.hold_r_axis = bool(gantry.get("hold_r_axis", True))
        self._motion = GantryModbusMotion(self.host, self.port)

    @property
    def connected(self) -> bool:
        return bool(self._motion.connected)

    def connect(self) -> dict[str, Any]:
        if self._motion.open():
            return {"success": True, "message": f"已连接 {self.host}:{self.port}"}
        return {"success": False, "message": f"连接失败 {self.host}:{self.port}"}

    def disconnect(self) -> None:
        self._motion.close()

    def read_positions(self) -> dict[str, float]:
        if not self.connected and not self._motion.open():
            raise RuntimeError("PLC 未连接")
        return self._motion.read_positions()

    def resolve_xyzr(self, cmd: Mapping[str, Any]) -> dict[str, float]:
        raw = cmd.get("gantry_xyzr")
        if isinstance(raw, Mapping) and all(k in raw for k in ("X", "Y", "Z", "R")):
            return {k: float(raw[k]) for k in ("X", "Y", "Z", "R")}
        world = cmd.get("world_pose") or {}
        try:
            return transform_world_to_gantry(world, self.world_to_gantry)
        except WorldToGantryError as exc:
            raise RuntimeError(f"无法换算 XYZR：{exc}") from exc

    def execute_command(self, cmd: Mapping[str, Any]) -> dict[str, Any]:
        targets = self.resolve_xyzr(cmd)
        if self.hold_r_axis:
            # 真机写轴时强制保持当前 R，避免指令里的 yaw/R 带动旋转
            current = self.read_positions()
            targets["R"] = float(current["R"])
        speed = float(cmd.get("speed") or self.default_speed)
        result = self._motion.move_absolute(
            targets,
            speed=speed,
            timeout_s=self.move_timeout_s,
            soft_limits=self.soft_limits or None,
        )
        out = deepcopy(result)
        out["cmd_id"] = cmd.get("cmd_id")
        out["task"] = cmd.get("task")
        out["targets"] = targets
        out["r_axis_held"] = bool(self.hold_r_axis)
        return out
