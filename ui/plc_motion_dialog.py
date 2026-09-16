# -*- coding: utf-8 -*-
"""主系统 PLC 运动指令弹出窗口：坐标显示 / 自动下发 / 确认下发。"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]

_DIALOG_QSS = """
QDialog {
    background: #111111;
    color: #f2f2f2;
}
QLabel {
    background: transparent;
    color: #f2f2f2;
    font-size: 14px;
}
QLabel#tipLabel {
    color: #c8c8c8;
    font-size: 13px;
}
QLabel#summaryLabel {
    color: #ffffff;
    font-size: 15px;
    font-weight: 700;
    padding: 8px 0;
}
QLabel#statusLabel {
    color: #d0d0d0;
    font-size: 14px;
    padding: 4px 0;
}
QTextEdit {
    background: #000000;
    color: #ffffff;
    border: 1px solid #444444;
    border-radius: 4px;
    font-size: 14px;
    padding: 8px;
    selection-background-color: #3a3a3a;
}
QPushButton {
    background: #2a2a2a;
    color: #ffffff;
    border: 1px solid #555555;
    border-radius: 4px;
    padding: 8px 14px;
    font-size: 14px;
    min-height: 28px;
}
QPushButton:hover {
    background: #3a3a3a;
    border: 1px solid #777777;
}
QPushButton:pressed {
    background: #1a1a1a;
}
QPushButton:disabled {
    color: #777777;
    background: #1a1a1a;
    border: 1px solid #333333;
}
QPushButton#primaryBtn {
    background: #ffffff;
    color: #111111;
    border: 1px solid #ffffff;
    font-weight: 700;
}
QPushButton#primaryBtn:hover {
    background: #e8e8e8;
}
QPushButton#primaryBtn:disabled {
    background: #444444;
    color: #999999;
    border: 1px solid #444444;
}
"""


class PlcMotionDialog(QDialog):
    """非模态：流程继续时仍可查看/下发坐标。"""

    def __init__(
        self,
        parent=None,
        *,
        on_confirm_push: Callable[[], dict] | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("PLC 运动坐标 / 下发")
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.resize(600, 520)
        self.setStyleSheet(_DIALOG_QSS)
        font = QFont("Microsoft YaHei UI", 11)
        self.setFont(font)
        self._on_confirm_push = on_confirm_push
        self._latest_cmd: dict[str, Any] | None = None
        self._build()

    def _build(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)

        tip = QLabel(
            "主系统只发布坐标；真机写入请在独立「PLC 控制台」执行。\n"
            "不下发也可继续下一步。自动下发开关在主界面工具栏。"
        )
        tip.setObjectName("tipLabel")
        tip.setWordWrap(True)
        layout.addWidget(tip)

        self.summary = QLabel("等待流程产生运动目标…")
        self.summary.setObjectName("summaryLabel")
        self.summary.setWordWrap(True)
        self.summary.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.summary)

        self.cmd_view = QTextEdit()
        self.cmd_view.setReadOnly(True)
        self.cmd_view.setPlaceholderText("WORLD / XYZR JSON 将显示在这里")
        self.cmd_view.setFont(QFont("Consolas", 12))
        layout.addWidget(self.cmd_view, 1)

        self.push_status = QLabel("下发状态：未下发")
        self.push_status.setObjectName("statusLabel")
        layout.addWidget(self.push_status)

        row = QHBoxLayout()
        row.setSpacing(10)
        self.confirm_btn = QPushButton("确认下发到 PLC")
        self.confirm_btn.setObjectName("primaryBtn")
        self.confirm_btn.setEnabled(False)
        self.confirm_btn.clicked.connect(self._confirm)
        self.open_console_btn = QPushButton("打开 PLC 控制台")
        self.open_console_btn.clicked.connect(self._open_console)
        self.close_btn = QPushButton("关闭")
        self.close_btn.clicked.connect(self.hide)
        row.addWidget(self.confirm_btn)
        row.addWidget(self.open_console_btn)
        row.addStretch(1)
        row.addWidget(self.close_btn)
        layout.addLayout(row)

    def clear_command(self) -> None:
        self._latest_cmd = None
        self.summary.setText("等待流程产生运动目标…")
        self.cmd_view.clear()
        self.push_status.setText("下发状态：未下发")
        self.confirm_btn.setEnabled(False)

    def apply_command(self, cmd: dict, *, auto_pushed: bool = False, push_result: dict | None = None) -> None:
        self._latest_cmd = dict(cmd or {})
        world = self._latest_cmd.get("world_pose") or {}
        xyzr = self._latest_cmd.get("gantry_xyzr")
        task = self._latest_cmd.get("task") or "-"
        step = self._latest_cmd.get("step") or "-"
        summary = (
            f"step = {step}    task = {task}\n"
            f"WORLD  x={float(world.get('x_mm', 0)):.1f}  y={float(world.get('y_mm', 0)):.1f}  "
            f"z={float(world.get('z_mm', 0)):.1f}  yaw={float(world.get('yaw_deg', 0)):.1f}"
        )
        if isinstance(xyzr, dict):
            summary += (
                f"\nXYZR   X={float(xyzr.get('X', 0)):.1f}  Y={float(xyzr.get('Y', 0)):.1f}  "
                f"Z={float(xyzr.get('Z', 0)):.1f}  R={float(xyzr.get('R', 0)):.1f}"
            )
        else:
            summary += "\nXYZR   未换算"
        self.summary.setText(summary)
        self.cmd_view.setPlainText(json.dumps(self._latest_cmd, ensure_ascii=False, indent=2))
        self.confirm_btn.setEnabled(True)
        if auto_pushed:
            push = dict(push_result or {})
            if push.get("success"):
                self.push_status.setText(f"下发状态：已自动推送 · {push.get('message', 'OK')}")
            else:
                self.push_status.setText(f"下发状态：自动推送失败 · {push.get('message', '')}")
        else:
            self.push_status.setText("下发状态：已生成（未下发，可确认或直接下一步）")
        if not self.isVisible():
            self.show()
        self.raise_()
        self.activateWindow()

    def set_push_status(self, text: str) -> None:
        self.push_status.setText(text)

    def _confirm(self) -> None:
        if not self._latest_cmd:
            QMessageBox.information(self, "提示", "当前没有可下发的运动坐标")
            return
        if not callable(self._on_confirm_push):
            return
        result = self._on_confirm_push() or {}
        if result.get("success"):
            self.push_status.setText(f"下发状态：已确认推送 · {result.get('message', 'OK')}")
        else:
            self.push_status.setText(f"下发状态：推送失败 · {result.get('message', '')}")
            QMessageBox.warning(self, "推送失败", result.get("message") or "PLC 控制台未连接")

    def _open_console(self) -> None:
        python = PROJECT_ROOT / "runtime" / "python.exe"
        exe = str(python if python.is_file() else sys.executable)
        try:
            subprocess.Popen(
                [exe, "-m", "plc_console"],
                cwd=str(PROJECT_ROOT),
                env={**os.environ, "PYTHONUTF8": "1"},
            )
            self.push_status.setText("下发状态：已尝试启动 plc_console")
        except Exception as exc:
            QMessageBox.warning(self, "启动失败", str(exc))

    def closeEvent(self, event):
        event.ignore()
        self.hide()
