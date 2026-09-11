# -*- coding: utf-8 -*-
"""Gantry / forklift-arm robot adapter for the PLC finished app.

Important: digital-twin WORLD poses (mm, X-right / Y-forward / Z-up) are NOT the
same as the gantry local XYZR used by plc_finished_app soft limits.  Until a
calibrated WORLD→gantry transform is configured, motion commands refuse to write
axis targets so the real machine cannot be driven with the wrong numbers.
"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from core.geometry import Pose6D
from devices.base import PLCAdapter, RobotAdapter
from devices.real_plc_adapter import RealPlcAdapter


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

    def _require_mapping(self, task: str) -> dict | None:
        if self.allow_real_motion and self.world_to_gantry:
            return None
        return {
            "success": False,
            "task": task,
            "message": (
                "真机机械臂运动已接入接口，但 WORLD→龙门架 XYZR 映射尚未启用。"
                "请在 config/system_config.json 的 devices.gantry 中配置 "
                "world_to_gantry，并设置 allow_real_motion=true 后再发运动。"
                "当前可先用 device_mode=mock 做完整流程演示；"
                "也可单独运行 third_party/plc_finished_app/plc_finished_console.py 做轴调试。"
            ),
        }

    def move_tool_world(self, robot_id: str, pose: dict, task: str = "") -> dict:
        blocked = self._require_mapping(task or "MOVE_TOOL_WORLD")
        if blocked is not None:
            self.twin.update_device(robot_id, status="HOLD", task="MAP_REQUIRED")
            if hasattr(self.plc, "send_message"):
                # TODO: 与PLC交互 — 未配映射时告知 PLC/孪生：真机运动被拦截
                self.plc.send_message("ROBOT", "BLOCKED", blocked["message"], {"robot_id": robot_id, "pose": pose})
            return blocked

        # TODO: 与PLC交互 — 真机：按 WORLD 位姿请求龙门架运动（需已配 world_to_gantry）
        cmd = self.plc.send_command(
            "MOVE_TOOL_WORLD",
            {"robot_id": robot_id, "pose": pose, "task": task, "gantry": self.world_to_gantry},
        )
        if not cmd.get("success"):
            return cmd
        # Mapping-enabled path still needs a concrete Modbus absolute move call;
        # keep the twin pose update only after a successful PLC write is wired.
        # TODO: 与PLC交互 — 真机：等 PLC 确认运动指令；后续还需接真实绝对定位写寄存器
        ack = self.plc.wait_ack(cmd["command_id"])
        if not ack.get("success"):
            return ack
        target = Pose6D.from_any(pose)
        self.twin.update_robot_pose(robot_id, target, task=task or "MOVED")
        return {"success": True, "robot_id": robot_id, "pose": target.to_dict(), "task": task}

    def retract(self, robot_id: str) -> dict:
        return self.move_tool_world(robot_id, {"x_mm": -3500, "y_mm": 1000, "z_mm": 1800, "yaw_deg": -90}, task="RETRACT")

    def fork_pallet(self, pallet_result: dict) -> dict:
        blocked = self._require_mapping("FORK_PALLET")
        if blocked is not None:
            return blocked
        # TODO: 与PLC交互 — 真机：下发插取托盘
        cmd = self.plc.send_command("FORK_PALLET", deepcopy(pallet_result or {}))
        # TODO: 与PLC交互 — 真机：等插取 ACK
        return self.plc.wait_ack(cmd["command_id"]) if cmd.get("success") else cmd

    def place(self, cargo: dict, target: dict) -> dict:
        blocked = self._require_mapping("PLACE")
        if blocked is not None:
            return blocked
        # TODO: 与PLC交互 — 真机：下发放货
        cmd = self.plc.send_command("PLACE", {"cargo": cargo, "target": target})
        # TODO: 与PLC交互 — 真机：等放货 ACK
        return self.plc.wait_ack(cmd["command_id"]) if cmd.get("success") else cmd