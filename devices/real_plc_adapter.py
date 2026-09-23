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
from devices.gantry_modbus_motion import GantryModbusMotion

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PLC_APP_DIR = PROJECT_ROOT / "third_party" / "plc_finished_app"


def _ensure_plc_import_path() -> None:
    path = str(PLC_APP_DIR.resolve())
    if path not in sys.path:
        sys.path.insert(0, path)


class RealPlcAdapter(PLCAdapter):
    """连接 PLC，并提供与 finished_app console 相同的绝对定位写入。"""

    def __init__(self, twin: DigitalTwinState, config: Mapping[str, Any] | None = None):
        self.twin = twin
        # TODO: 与PLC交互 — 配置来自 config/external_devices_config.py → PLC
        self.config = dict(config or {})
        self.ip = str(self.config.get("ip") or self.config.get("plc_ip") or "192.168.6.6")
        self.port = int(self.config.get("port") or self.config.get("plc_port") or 502)
        self.connected = False
        self.commands: dict[str, dict] = {}
        self._client = None
        self._motion: GantryModbusMotion | None = None
        # 运动前以 PLC 实际当前位置为准，避免软件目标位姿中的 R 带动旋转。
        self.hold_r_axis = bool(self.config.get("hold_r_axis", True))
        self.locked_r_deg: float | None = None

    def connect(self):
        # TODO: 与PLC交互 — 按配置 IP/端口连现场 PLC（Modbus TCP），失败则整机真机流程起不来
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
            # TCP 通了还不够：必须读到寄存器，避免非实验室环境误报 ONLINE
            try:
                snapshot = client.read_snapshot()
            except Exception as exc:
                try:
                    client.close()
                except Exception:
                    pass
                self._client = None
                self.twin.update_device("PLC", status="OFFLINE", task="READ_FAILED")
                return {
                    "success": False,
                    "message": f"PLC {self.ip}:{self.port} TCP 可达但寄存器读取失败：{exc}",
                }
            if snapshot is None:
                try:
                    client.close()
                except Exception:
                    pass
                self._client = None
                self.twin.update_device("PLC", status="OFFLINE", task="READ_EMPTY")
                return {
                    "success": False,
                    "message": f"PLC {self.ip}:{self.port} 未读到有效快照，判定未连接",
                }
            self._client = client
            self._motion = GantryModbusMotion(self.ip, self.port)
            if not self._motion.open():
                try:
                    client.close()
                except Exception:
                    pass
                self._client = None
                self._motion = None
                self.twin.update_device("PLC", status="OFFLINE", task="MOTION_CONNECT_FAILED")
                return {"success": False, "message": f"PLC 运动通道连接失败：{self.ip}:{self.port}"}
            self.connected = True
            self.twin.update_device("PLC", status="ONLINE", task=f"CONNECTED {self.ip}:{self.port}")
            return {
                "success": True,
                "ip": self.ip,
                "port": self.port,
                "snapshot": str(snapshot),
                "message": f"已连接 PLC {self.ip}:{self.port}（寄存器可读）",
            }
        except Exception as exc:
            self.twin.update_device("PLC", status="OFFLINE", task="CONNECT_ERROR")
            return {"success": False, "message": f"PLC 连接异常：{exc}"}

    def send_command(self, command: str, payload: dict) -> dict:
        # TODO: 与PLC交互 — 下发业务指令；MOVE 类由 move_absolute_xyzr 真写入
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
        # TODO: 与PLC交互 — 编排层 ACK；真机到位在 move_absolute_xyzr 内等待
        if command_id not in self.commands:
            return {"success": False, "message": "未知 PLC command_id"}
        self.commands[command_id]["status"] = "ACK"
        self.twin.update_device("PLC", task="ACK")
        return {"success": True, "command_id": command_id, "ack": True, "timeout_ms": timeout_ms}

    def read_positions(self) -> dict[str, float]:
        if not self.connected or self._motion is None:
            linked = self.connect()
            if not linked.get("success"):
                raise RuntimeError(str(linked.get("message") or "PLC 未连接"))
        assert self._motion is not None
        return self._motion.read_positions()

    def move_absolute_xyzr(
        self,
        targets: Mapping[str, float],
        speed: float = 30.0,
        timeout_s: float = 60.0,
        soft_limits: Mapping[str, Any] | None = None,
        axes: tuple[str, ...] = ("X", "Y", "Z", "R"),
    ) -> dict:
        """按 finished_app 时序写指定轴目标并等待到位。"""
        if not self.connected or self._motion is None:
            linked = self.connect()
            if not linked.get("success"):
                return linked
        assert self._motion is not None
        try:
            selected_axes = tuple(str(axis).upper() for axis in axes)
            if not selected_axes or any(axis not in {"X", "Y", "Z", "R"} for axis in selected_axes):
                raise RuntimeError(f"无效 PLC 运动轴：{selected_axes}")
            actual_targets = {key: float(value) for key, value in dict(targets).items() if key in selected_axes}
            if "R" in selected_axes and self.hold_r_axis:
                # 软件启动后的首次运动读取 PLC 实际 R，作为本次软件会话的锁定基准。
                if self.locked_r_deg is None:
                    current = self._motion.read_positions()
                    if not isinstance(current, Mapping) or current.get("R") is None:
                        raise RuntimeError("无法读取 PLC 当前 R 轴位置，已拒绝运动")
                    self.locked_r_deg = float(current["R"])
                if self.locked_r_deg is None:
                    raise RuntimeError("无法读取 PLC 当前 R 轴位置，已拒绝运动")
                actual_targets["R"] = float(self.locked_r_deg)
            result = self._motion.move_absolute(
                actual_targets,
                speed=speed,
                timeout_s=timeout_s,
                soft_limits=soft_limits,
                axes=selected_axes,
            )
            result = {
                **result,
                "targets": actual_targets,
                "axes": list(selected_axes),
                "r_axis_held": self.hold_r_axis,
                "r_axis_untouched": "R" not in selected_axes,
                "locked_r_deg": self.locked_r_deg,
            }
            self.twin.update_device("PLC", task="ABS_MOVE_DONE")
            self.twin.add_message("PLC", "SUCCESS", result.get("message", "绝对定位完成"), result)
            return result
        except Exception as exc:
            self.twin.update_device("PLC", status="FAILED", task="ABS_MOVE_FAILED")
            self.twin.add_message("PLC", "FAILED", str(exc), {"targets": dict(targets)})
            return {
                "success": False,
                "message": str(exc),
                "targets": dict(targets),
                "axes": list(axes),
                "r_axis_held": self.hold_r_axis,
                "r_axis_untouched": "R" not in axes,
                "locked_r_deg": self.locked_r_deg,
            }

    def send_message(self, module: str, status: str, message: str, data: dict | None = None) -> dict:
        self.twin.add_message(module, status, message, data)
        return {"success": True, "module": module, "status": status, "message": message, "data": data}

    def close(self):
        if self._motion is not None:
            try:
                self._motion.close()
            except Exception:
                pass
            self._motion = None
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None
        self.connected = False
