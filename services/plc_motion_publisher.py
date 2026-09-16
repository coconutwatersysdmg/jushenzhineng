# -*- coding: utf-8 -*-
"""主系统侧：运动指令落盘 + 实时推送到 plc_console。"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Mapping

from PySide6.QtCore import QByteArray
from PySide6.QtNetwork import QLocalSocket

from contracts.plc_motion_cmd import (
    PLC_CONSOLE_SERVER_NAME,
    build_motion_command,
    dumps_command,
    loads_command,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT_DIR = PROJECT_ROOT / "workdir" / "plc_commands"


class PlcMotionPublisher:
    def __init__(
        self,
        out_dir: Path | None = None,
        server_name: str = PLC_CONSOLE_SERVER_NAME,
        push_timeout_ms: int = 800,
    ):
        self.out_dir = Path(out_dir or DEFAULT_OUT_DIR)
        self.server_name = str(server_name)
        self.push_timeout_ms = int(push_timeout_ms)
        self._listener: Callable[[dict], None] | None = None
        self.last_command: dict[str, Any] | None = None
        self.last_push: dict[str, Any] | None = None

    def set_listener(self, listener: Callable[[dict], None] | None) -> None:
        self._listener = listener if callable(listener) else None

    def build(
        self,
        *,
        robot_id: str,
        world_pose: Mapping[str, Any],
        task: str = "",
        step: str = "",
        round_index: int = 0,
        gantry_xyzr: Mapping[str, float] | None = None,
        speed: float = 30.0,
        extra: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        return build_motion_command(
            robot_id=robot_id,
            world_pose=world_pose,
            task=task,
            step=step,
            round_index=round_index,
            gantry_xyzr=gantry_xyzr,
            speed=speed,
            extra=extra,
        )

    def publish_local(self, cmd: Mapping[str, Any]) -> dict[str, Any]:
        """落盘 + 通知 UI；不推送 PLC 模块。"""
        data = dict(cmd)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        day = datetime.now().strftime("%Y%m%d")
        path = self.out_dir / f"motion_{day}.jsonl"
        with path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(data, ensure_ascii=False) + "\n")
        single = self.out_dir / "latest_motion_cmd.json"
        single.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        data = {**data, "_saved_path": str(path), "_latest_path": str(single)}
        self.last_command = data
        if self._listener:
            self._listener(data)
        return data

    def push(self, cmd: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """实时推送到 plc_console；成功仅表示对方已接收（方案 B）。"""
        payload = dict(cmd or self.last_command or {})
        if not payload:
            result = {"success": False, "message": "没有可下发的运动指令"}
            self.last_push = result
            return result
        socket = QLocalSocket()
        socket.connectToServer(self.server_name)
        if not socket.waitForConnected(self.push_timeout_ms):
            result = {
                "success": False,
                "message": f"PLC 控制台未连接（{self.server_name}）。请先启动 plc_console。",
                "cmd_id": payload.get("cmd_id"),
            }
            self.last_push = result
            return result
        line = dumps_command(payload) + "\n"
        socket.write(QByteArray(line.encode("utf-8")))
        socket.flush()
        socket.waitForBytesWritten(self.push_timeout_ms)
        if not socket.waitForReadyRead(self.push_timeout_ms):
            socket.disconnectFromServer()
            result = {
                "success": False,
                "message": "已连接控制台但未收到接收确认",
                "cmd_id": payload.get("cmd_id"),
            }
            self.last_push = result
            return result
        raw = bytes(socket.readAll()).decode("utf-8", errors="replace").strip()
        ack: dict[str, Any] = {"success": True, "accepted": True, "message": "已推送"}
        if raw:
            try:
                parsed = json.loads(raw.splitlines()[-1])
                if isinstance(parsed, dict):
                    ack = parsed
            except Exception:
                ack = {"success": True, "accepted": True, "message": raw[:200]}
        socket.disconnectFromServer()
        result = {
            "success": bool(ack.get("success", ack.get("accepted", False))),
            "accepted": bool(ack.get("accepted", ack.get("success", False))),
            "message": str(ack.get("message") or ("推送成功" if ack.get("success") else "推送失败")),
            "cmd_id": payload.get("cmd_id"),
            "ack": ack,
        }
        self.last_push = result
        return result

    def publish_and_maybe_push(self, cmd: Mapping[str, Any], *, auto_push: bool) -> dict[str, Any]:
        local = self.publish_local(cmd)
        if not auto_push:
            return {"command": local, "pushed": False, "push_result": None}
        push_result = self.push(local)
        return {"command": local, "pushed": True, "push_result": push_result}

    @staticmethod
    def ping(server_name: str = PLC_CONSOLE_SERVER_NAME, timeout_ms: int = 400) -> dict[str, Any]:
        socket = QLocalSocket()
        socket.connectToServer(server_name)
        if not socket.waitForConnected(timeout_ms):
            return {"success": False, "message": "PLC 控制台离线"}
        socket.write(QByteArray(b'{"schema":"plc_motion_cmd_v1","type":"PING"}\n'))
        socket.flush()
        socket.waitForBytesWritten(timeout_ms)
        ok = socket.waitForReadyRead(timeout_ms)
        raw = bytes(socket.readAll()).decode("utf-8", errors="replace") if ok else ""
        socket.disconnectFromServer()
        return {"success": bool(ok), "message": raw.strip() or ("在线" if ok else "无响应")}
