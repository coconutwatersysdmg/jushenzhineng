# -*- coding: utf-8 -*-
"""PLC adapter backed by third_party/plc_finished_app Modbus clients."""
from __future__ import annotations

import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping
from uuid import uuid4

from core.digital_twin_state import DigitalTwinState
from devices.base import PLCAdapter

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PLC_APP_DIR = PROJECT_ROOT / "third_party" / "plc_finished_app"


def _ensure_plc_import_path() -> None:
    path = str(PLC_APP_DIR.resolve())
    if path not in sys.path:
        sys.path.insert(0, path)


class RealPlcAdapter(PLCAdapter):
    """Connect / message / command bookkeeping for the gantry PLC.

    Motion writes are owned by RealGantryRobotAdapter so WORLD→XYZR mapping can
    stay explicit and safe.
    """

    def __init__(self, twin: DigitalTwinState, config: Mapping[str, Any] | None = None):
        self.twin = twin
        self.config = dict(config or {})
        self.ip = str(self.config.get("ip") or self.config.get("plc_ip") or "192.168.6.6")
        self.port = int(self.config.get("port") or self.config.get("plc_port") or 502)
        self.connected = False
        self.commands: dict[str, dict] = {}
        self._client = None

    def connect(self):
        _ensure_plc_import_path()
        try:
            from plc_readonly_core import PlcModeClient
        except Exception as exc:
            self.twin.update_device("PLC", status="FAILED", task="IMPORT_FAILED")
            return {
                "success": False,
                "message": f"无法导入 PLC 核心模块（third_party/plc_finished_app）：{exc}",
            }

        try:
            client = PlcModeClient(host=self.ip, port=self.port)
            if not client.open():
                self.twin.update_device("PLC", status="OFFLINE", task="CONNECT_FAILED")
                return {"success": False, "message": f"PLC 连接失败：{self.ip}:{self.port}"}
            snapshot = None
            try:
                snapshot = client.read_snapshot()
            except Exception:
                snapshot = None
            self._client = client
            self.connected = True
            self.twin.update_device("PLC", status="ONLINE", task=f"CONNECTED {self.ip}:{self.port}")
            return {
                "success": True,
                "ip": self.ip,
                "port": self.port,
                "snapshot": None if snapshot is None else str(snapshot),
                "message": f"已连接 PLC {self.ip}:{self.port}",
            }
        except Exception as exc:
            self.twin.update_device("PLC", status="FAILED", task="CONNECT_ERROR")
            return {"success": False, "message": f"PLC 连接异常：{exc}"}

    def send_command(self, command: str, payload: dict) -> dict:
        if not self.connected:
            linked = self.connect()
            if not linked.get("success"):
                return linked
        cid = f"CMD-{uuid4().hex[:10].upper()}"
        self.commands[cid] = {
            "command": command,
            "payload": deepcopy(payload),
            "status": "SENT",
        }
        self.twin.update_device("PLC", task=f"{command} / SENT")
        self.twin.add_message("PLC", "SENT", f"{command}", {"command_id": cid, "payload": payload})
        return {
            "success": True,
            "command_id": cid,
            "command": command,
            "payload": deepcopy(payload),
        }

    def wait_ack(self, command_id: str, timeout_ms: int = 5000) -> dict:
        if command_id not in self.commands:
            return {"success": False, "message": "未知 PLC command_id"}
        # Book-keeping ACK for flow orchestration. Real motion completion is
        # verified inside RealGantryRobotAdapter when mapping is enabled.
        self.commands[command_id]["status"] = "ACK"
        self.twin.update_device("PLC", task="ACK")
        return {"success": True, "command_id": command_id, "ack": True, "timeout_ms": timeout_ms}

    def send_message(self, module: str, status: str, message: str, data: dict | None = None) -> dict:
        self.twin.add_message(module, status, message, data)
        return {"success": True, "module": module, "status": status, "message": message, "data": data}

    def close(self):
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None
        self.connected = False
