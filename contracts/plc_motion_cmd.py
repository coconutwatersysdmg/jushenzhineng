# -*- coding: utf-8 -*-
"""PLC 运动指令 JSON 协议（主系统 ↔ plc_console）。"""
from __future__ import annotations

import json
from copy import deepcopy
from datetime import datetime, timezone
from typing import Any, Mapping
from uuid import uuid4

SCHEMA_ID = "plc_motion_cmd_v1"
PLC_CONSOLE_SERVER_NAME = "jushenzhineng-plc-console"


def build_motion_command(
    *,
    robot_id: str,
    world_pose: Mapping[str, Any],
    task: str = "",
    step: str = "",
    round_index: int = 0,
    gantry_xyzr: Mapping[str, float] | None = None,
    speed: float = 30.0,
    source: str = "flow",
    extra: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    cmd: dict[str, Any] = {
        "schema": SCHEMA_ID,
        "type": "MOTION_CMD",
        "cmd_id": f"CMD-{uuid4().hex[:12].upper()}",
        "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "round": int(round_index),
        "step": str(step or ""),
        "task": str(task or ""),
        "robot_id": str(robot_id or "PICK_ARM"),
        "world_pose": {
            "x_mm": float(world_pose.get("x_mm", world_pose.get("x", 0.0)) or 0.0),
            "y_mm": float(world_pose.get("y_mm", world_pose.get("y", 0.0)) or 0.0),
            "z_mm": float(world_pose.get("z_mm", world_pose.get("z", 0.0)) or 0.0),
            "roll_deg": float(world_pose.get("roll_deg", world_pose.get("roll", 0.0)) or 0.0),
            "pitch_deg": float(world_pose.get("pitch_deg", world_pose.get("pitch", 0.0)) or 0.0),
            "yaw_deg": float(world_pose.get("yaw_deg", world_pose.get("yaw", 0.0)) or 0.0),
        },
        "gantry_xyzr": None,
        "speed": float(speed),
        "require_ack": True,
        "source": str(source),
    }
    if gantry_xyzr:
        cmd["gantry_xyzr"] = {
            "X": float(gantry_xyzr.get("X", gantry_xyzr.get("x", 0.0))),
            "Y": float(gantry_xyzr.get("Y", gantry_xyzr.get("y", 0.0))),
            "Z": float(gantry_xyzr.get("Z", gantry_xyzr.get("z", 0.0))),
            "R": float(gantry_xyzr.get("R", gantry_xyzr.get("r", 0.0))),
        }
    if extra:
        cmd["extra"] = deepcopy(dict(extra))
    return cmd


def validate_motion_command(payload: Mapping[str, Any]) -> dict[str, Any]:
    data = dict(payload or {})
    if data.get("schema") not in (None, SCHEMA_ID):
        raise ValueError(f"不支持的 schema: {data.get('schema')}")
    if data.get("type") not in (None, "MOTION_CMD"):
        raise ValueError(f"不支持的 type: {data.get('type')}")
    if not data.get("cmd_id"):
        data["cmd_id"] = f"CMD-{uuid4().hex[:12].upper()}"
    data["schema"] = SCHEMA_ID
    data["type"] = "MOTION_CMD"
    if not isinstance(data.get("world_pose"), Mapping) and not isinstance(data.get("gantry_xyzr"), Mapping):
        raise ValueError("指令至少需要 world_pose 或 gantry_xyzr")
    return data


def dumps_command(cmd: Mapping[str, Any]) -> str:
    return json.dumps(dict(cmd), ensure_ascii=False, separators=(",", ":"))


def loads_command(raw: str | bytes) -> dict[str, Any]:
    if isinstance(raw, bytes):
        text = raw.decode("utf-8", errors="replace")
    else:
        text = str(raw)
    text = text.strip()
    if not text:
        raise ValueError("空指令")
    return validate_motion_command(json.loads(text))


def build_ack(
    *,
    cmd_id: str,
    success: bool,
    message: str = "",
    accepted: bool = True,
    result: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "schema": SCHEMA_ID,
        "type": "MOTION_ACK",
        "cmd_id": str(cmd_id),
        "success": bool(success),
        "accepted": bool(accepted),
        "message": str(message or ("OK" if success else "FAILED")),
        "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "result": dict(result or {}),
    }
