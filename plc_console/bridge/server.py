# -*- coding: utf-8 -*-
"""接收主系统 QLocalSocket JSON 推送。接收成功即 ACK（方案 B），执行由 UI 侧异步处理。"""
from __future__ import annotations

import json

from PySide6.QtCore import QObject, Signal
from PySide6.QtNetwork import QLocalServer, QLocalSocket

from contracts.plc_motion_cmd import PLC_CONSOLE_SERVER_NAME, SCHEMA_ID, build_ack, validate_motion_command


class PlcBridgeServer(QObject):
    commandReceived = Signal(dict)
    clientMessage = Signal(str)

    def __init__(self, server_name: str = PLC_CONSOLE_SERVER_NAME, parent=None):
        super().__init__(parent)
        self.server_name = str(server_name)
        self._server = QLocalServer(self)
        self._server.newConnection.connect(self._on_new_connection)

    def start(self) -> dict:
        QLocalServer.removeServer(self.server_name)
        if not self._server.listen(self.server_name):
            return {"success": False, "message": self._server.errorString()}
        return {"success": True, "message": f"监听 {self.server_name}"}

    def stop(self) -> None:
        self._server.close()

    def _on_new_connection(self) -> None:
        while self._server.hasPendingConnections():
            socket = self._server.nextPendingConnection()
            socket.readyRead.connect(lambda s=socket: self._on_ready_read(s))
            # 对端可能在信号连接前已写完数据
            if socket.bytesAvailable() > 0:
                self._on_ready_read(socket)

    def _on_ready_read(self, socket: QLocalSocket) -> None:
        raw = bytes(socket.readAll()).decode("utf-8", errors="replace")
        for line in raw.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except Exception as exc:
                self._write_ack(
                    socket,
                    build_ack(cmd_id="", success=False, accepted=False, message=f"JSON 解析失败: {exc}"),
                )
                continue
            msg_type = str(payload.get("type") or "MOTION_CMD")
            if msg_type == "PING":
                self._write_ack(socket, {"schema": SCHEMA_ID, "type": "PONG", "success": True, "message": "PONG"})
                self.clientMessage.emit("PING")
                continue
            try:
                cmd = validate_motion_command(payload)
            except Exception as exc:
                self._write_ack(
                    socket,
                    build_ack(
                        cmd_id=str(payload.get("cmd_id") or ""),
                        success=False,
                        accepted=False,
                        message=str(exc),
                    ),
                )
                continue
            # 方案 B：接收成功即可；真机到位不阻塞主系统
            self._write_ack(
                socket,
                build_ack(cmd_id=cmd["cmd_id"], success=True, accepted=True, message="指令已接收"),
            )
            self.commandReceived.emit(cmd)

    @staticmethod
    def _write_ack(socket: QLocalSocket, ack: dict) -> None:
        data = (json.dumps(ack, ensure_ascii=False) + "\n").encode("utf-8")
        socket.write(data)
        socket.flush()
        socket.waitForBytesWritten(500)
        socket.disconnectFromServer()
