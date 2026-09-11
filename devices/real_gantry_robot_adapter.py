# -*- coding: utf-8 -*-
"""Gantry / forklift-arm robot adapter for the PLC finished app.

WORLD 位姿经 world_to_gantry 换成 XYZR 后，调用 RealPlcAdapter.move_absolute_xyzr
（与 plc_finished_console 写寄存器时序一致）。
"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from core.geometry import Pose6D
from devices.base import PLCAdapter, RobotAdapter
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
        # Explicit opt-in once calibration mapping is ready.
        self.allow_real_motion = bool(self.config.get("allow_real_motion", False))
        self.world_to_gantry = dict(self.config.get("world_to_gantry") or {})
        self.default_speed = float(self.config.get("default_speed", 30.0))
        self.move_timeout_s = float(self.config.get("move_timeout_s", 60.0))
        self.soft_limits = dict(self.config.get("soft_limits") or {})

    def _require_mapping(self, task: str) -> dict | None:
        if not self.allow_real_motion:
            return {
                "success": False,
                "task": task,
                "message": (
                    "真机运动未启用：请在 config/system_config.py 将 "
                    "devices.gantry.allow_real_motion=True（标定确认后再开）。"
                ),
            }
        if not self.world_to_gantry:
            return {
                "success": False,
                "task": task,
                "message": (
                    "缺少 world_to_gantry。请在 config/system_config.py 填写 "
                    "WORLD→XYZR 映射后再发运动。"
                ),
            }
        if bool(self.world_to_gantry.get("placeholder", False)):
            return {
                "success": False,
                "task": task,
                "message": (
                    "world_to_gantry 仍是占位假数据（placeholder=True）。"
                    "现场标定后改 False 并填真实数，再开 allow_real_motion。"
                ),
            }
        return None

    def move_tool_world(self, robot_id: str, pose: dict, task: str = "") -> dict:
        blocked = self._require_mapping(task or "MOVE_TOOL_WORLD")
        if blocked is not None:
            self.twin.update_device(robot_id, status="HOLD", task="MAP_REQUIRED")
            if hasattr(self.plc, "send_message"):
                # TODO: 与PLC交互 — 未配映射/未开真机时告知：运动被拦截
                self.plc.send_message("ROBOT", "BLOCKED", blocked["message"], {"robot_id": robot_id, "pose": pose})
            return blocked

        try:
            gantry_xyzr = transform_world_to_gantry(pose, self.world_to_gantry)
        except WorldToGantryError as exc:
            msg = str(exc)
            self.twin.update_device(robot_id, status="HOLD", task="TRANSFORM_FAILED")
            if hasattr(self.plc, "send_message"):
                self.plc.send_message("ROBOT", "BLOCKED", msg, {"robot_id": robot_id, "pose": pose})
            return {"success": False, "task": task, "message": msg}

        # TODO: 与PLC交互 — 记账 + 真写 XYZR（finished_app 绝对定位时序）
        cmd = self.plc.send_command(
            "MOVE_TOOL_WORLD",
            {
                "robot_id": robot_id,
                "pose": pose,
                "task": task,
                "gantry_xyzr": gantry_xyzr,
            },
        )
        if not cmd.get("success"):
            return cmd

        if not hasattr(self.plc, "move_absolute_xyzr"):
            return {
                "success": False,
                "message": "当前 PLC 适配器不支持 move_absolute_xyzr 真写入",
                "gantry_xyzr": gantry_xyzr,
            }

        moved = self.plc.move_absolute_xyzr(
            gantry_xyzr,
            speed=self.default_speed,
            timeout_s=self.move_timeout_s,
            soft_limits=self.soft_limits or None,
        )
        if not moved.get("success"):
            return moved

        # TODO: 与PLC交互 — 编排层 ACK（到位已在 move_absolute_xyzr 完成）
        ack = self.plc.wait_ack(cmd["command_id"])
        if not ack.get("success"):
            return ack

        target = Pose6D.from_any(pose)
        self.twin.update_robot_pose(robot_id, target, task=task or "MOVED")
        if callable(self.motion_callback):
            self.motion_callback(robot_id, target.to_dict(), task or "MOVED")
        return {
            "success": True,
            "robot_id": robot_id,
            "pose": target.to_dict(),
            "task": task,
            "gantry_xyzr": gantry_xyzr,
            "plc_move": moved,
            "message": f"{robot_id} 已按 XYZR 绝对定位完成",
        }

    def retract(self, robot_id: str) -> dict:
        return self.move_tool_world(
            robot_id,
            {"x_mm": -3500, "y_mm": 1000, "z_mm": 1800, "yaw_deg": -90},
            task="RETRACT",
        )

    def fork_pallet(self, pallet_result: dict) -> dict:
        blocked = self._require_mapping("FORK_PALLET")
        if blocked is not None:
            return blocked
        # TODO: 与PLC交互 — 插取动作寄存器协议待与机械确认；当前仅下发编排命令
        cmd = self.plc.send_command("FORK_PALLET", deepcopy(pallet_result or {}))
        return self.plc.wait_ack(cmd["command_id"]) if cmd.get("success") else cmd

    def place(self, cargo: dict, target: dict) -> dict:
        blocked = self._require_mapping("PLACE")
        if blocked is not None:
            return blocked
        # TODO: 与PLC交互 — 放货 IO/动作协议待与机械确认；当前仅下发编排命令
        cmd = self.plc.send_command("PLACE", {"cargo": cargo, "target": target})
        return self.plc.wait_ack(cmd["command_id"]) if cmd.get("success") else cmd
