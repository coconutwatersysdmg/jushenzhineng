# -*- coding: utf-8 -*-
"""Gantry / forklift-arm robot adapter.

主系统只负责 WORLD→XYZR 换算、孪生更新与运动指令发布；
真机写寄存器由独立模块 ``plc_console`` 执行。
"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Callable, Mapping

from core.digital_twin_state import DigitalTwinState
from core.geometry import Pose6D
from devices.base import PLCAdapter, RobotAdapter
from devices.r_axis_hold import RAxisHold
from devices.world_to_gantry import WorldToGantryError, transform_world_to_gantry


class RealGantryRobotAdapter(RobotAdapter):
    def __init__(
        self,
        twin: DigitalTwinState,
        plc: PLCAdapter,
        config: Mapping[str, Any] | None = None,
    ):
        self.twin = twin
        self.plc = plc
        self.config = dict(config or {})
        self.motion_callback = None
        self.command_emitter: Callable[[dict], None] | None = None
        # 历史开关：现表示“允许换算并对外发布 XYZR”，主流程不再直写 PLC。
        self.allow_real_motion = bool(self.config.get("allow_real_motion", False))
        self.world_to_gantry = dict(self.config.get("world_to_gantry") or {})
        self.default_speed = float(self.config.get("default_speed", 30.0))
        self.move_timeout_s = float(self.config.get("move_timeout_s", 60.0))
        self.soft_limits = dict(self.config.get("soft_limits") or {})
        self.r_hold = RAxisHold(enabled=bool(self.config.get("hold_r_axis", True)))

    def _resolve_gantry(self, pose: Mapping[str, Any]) -> tuple[dict[str, float] | None, str | None]:
        if not self.world_to_gantry:
            return None, "缺少 world_to_gantry 映射"
        try:
            return transform_world_to_gantry(pose, self.world_to_gantry), None
        except WorldToGantryError as exc:
            return None, str(exc)

    def _current_pose(self, robot_id: str) -> dict[str, Any]:
        device = ((self.twin.snapshot().get("devices") or {}).get(robot_id) or {})
        return dict(device.get("pose") or {})

    def _with_held_r(self, robot_id: str, pose: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, float] | None, str | None]:
        current = self._current_pose(robot_id)
        current_gantry, _ = self._resolve_gantry(current) if current else (None, None)
        self.r_hold.capture_from_pose(current or pose, current_gantry)
        held_pose = self.r_hold.apply_to_pose(pose)
        gantry_xyzr, transform_error = self._resolve_gantry(held_pose)
        gantry_xyzr = self.r_hold.apply_to_gantry(gantry_xyzr)
        return held_pose, gantry_xyzr, transform_error

    def _emit_command(self, payload: dict) -> None:
        if callable(self.command_emitter):
            self.command_emitter(payload)

    def move_tool_world(self, robot_id: str, pose: dict, task: str = "") -> dict:
        held_pose, gantry_xyzr, transform_error = self._with_held_r(robot_id, pose)
        target = Pose6D.from_any(held_pose)
        self.twin.update_robot_pose(robot_id, target, task=task or "MOVED")
        if callable(self.motion_callback):
            self.motion_callback(robot_id, target.to_dict(), task or "MOVED")

        # 编排层记账（不写轴）；真机由 plc_console 执行
        cmd = self.plc.send_command(
            "MOVE_TOOL_WORLD",
            {
                "robot_id": robot_id,
                "pose": held_pose,
                "task": task,
                "gantry_xyzr": gantry_xyzr,
                "export_only": True,
            },
        )
        ack = self.plc.wait_ack(cmd["command_id"]) if cmd.get("success") else cmd

        emit_payload = {
            "robot_id": robot_id,
            "world_pose": target.to_dict(),
            "task": task or "MOVE_TOOL_WORLD",
            "gantry_xyzr": gantry_xyzr,
            "speed": self.default_speed,
            "transform_error": transform_error,
            "plc_command_id": cmd.get("command_id"),
            "r_axis_held": bool(self.r_hold.enabled),
            "locked_r_deg": self.r_hold.locked_r_deg,
        }
        self._emit_command(emit_payload)

        message = f"{robot_id} 目标已发布到运动指令通道"
        if self.r_hold.enabled and self.r_hold.locked_r_deg is not None:
            message += f"（R轴锁定 {self.r_hold.locked_r_deg:.2f}°）"
        if transform_error:
            message += f"（XYZR 未换算：{transform_error}）"
        elif not self.allow_real_motion:
            message += "（allow_real_motion=False，仍可手动在控制台执行）"

        return {
            "success": True,
            "robot_id": robot_id,
            "pose": target.to_dict(),
            "task": task,
            "gantry_xyzr": gantry_xyzr,
            "transform_error": transform_error,
            "export_only": True,
            "plc_ack": ack,
            "r_axis_held": bool(self.r_hold.enabled),
            "locked_r_deg": self.r_hold.locked_r_deg,
            "message": message,
        }

    def retract(self, robot_id: str) -> dict:
        return self.move_tool_world(
            robot_id,
            {"x_mm": -3500, "y_mm": 1000, "z_mm": 1800, "yaw_deg": -90},
            task="RETRACT",
        )

    def fork_pallet(self, pallet_result: dict) -> dict:
        cmd = self.plc.send_command("FORK_PALLET", deepcopy(pallet_result or {}))
        result = self.plc.wait_ack(cmd["command_id"]) if cmd.get("success") else cmd
        self._emit_command(
            {
                "robot_id": "PICK_ARM",
                "world_pose": ((self.twin.snapshot().get("devices") or {}).get("PICK_ARM") or {}).get("pose") or {},
                "task": "FORK_PALLET",
                "gantry_xyzr": None,
                "extra": {"fork": deepcopy(pallet_result or {})},
            }
        )
        return result

    def place(self, cargo: dict, target: dict) -> dict:
        cmd = self.plc.send_command("PLACE", {"cargo": cargo, "target": target})
        result = self.plc.wait_ack(cmd["command_id"]) if cmd.get("success") else cmd
        pose = (target or {}).get("final_world_pose") or {}
        gantry_xyzr, _ = self._resolve_gantry(pose) if pose else (None, None)
        self._emit_command(
            {
                "robot_id": "PICK_ARM",
                "world_pose": pose,
                "task": "PLACE",
                "gantry_xyzr": gantry_xyzr,
                "extra": {"target": deepcopy(target or {})},
            }
        )
        return result
