# -*- coding: utf-8 -*-
"""主系统 PLC 运动指令弹出窗口：坐标显示 / 确认下发。"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

from PySide6.QtCore import Qt, Signal
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

# 流程步骤代码 → 界面文案（与 FlowController 一致）
STEP_TITLES = {
    "DEVICE_CHECK": "1. 外接设备连接检查",
    "PRE_PICK_OFFSET": "2. 货物-托盘偏差分析",
    "PARALLEL_LOCATE": "3. 并行：3.1 插取 ∥ 3.2 雷达",
    "PICK_ONLY": "3.1 机械臂找插孔并插取",
    "RADAR_TO_CAMERA": "4. 雷达粗点 → 相机目标",
    "CAPTURE_CORNERS": "5. 角点 RGB-D 拍摄",
    "CORNER_RECOGNITION": "6. 角点识别 / 人工审核",
    "CAMERA_TO_WORLD": "7. 转换 WORLD",
    "INITIAL_SPACE_PLAN": "8.1 车板规划",
    "NEIGHBOR_POSE": "8.2 临近托盘姿态",
    "TARGET_CONFIRM": "8.3 锁定本轮目标",
    "PRE_PLACE_MONITOR": "8.4 放置前动态监测",
    "PLACE": "9. 计算放置点 / 放货",
    "POST_PLACE_BOTTOM": "10.0 放置后底托检测",
    "POST_REGION": "10.1 区域偏差 + 两面观测",
    "POST_CARGO_OFFSET": "10.2 托盘-货物偏差",
    "FEEDBACK": "11. 偏差回传 / 更新空间",
    "RETURN": "12. 机械臂返回 / 下一轮",
}

# 运动 task → 简短目的（前缀匹配，长的在前）
_TASK_PURPOSE = (
    ("MOVE_TO_TAIL_STAGED_CARGO", "移到车尾待装货物上方，准备找插孔"),
    ("FIND_PALLET_HOLE", "到插孔识别位，便于相机定位插孔"),
    ("LIFT_STAGED_CARGO", "插取完成后抬升带货，离开货位"),
    ("CAPTURE_TAIL", "到车尾左侧外侧，拍摄车尾角点 RGB-D"),
    ("CAPTURE_HEAD", "到车头左侧外侧，拍摄车头角点 RGB-D"),
    ("NEIGHBOR_PALLET_POSE", "到目标侧附近，拍摄临近已放托盘姿态"),
    ("PRE_PLACE_DYNAMIC_MONITOR", "到放置侧观测位，做放货前动态监测"),
    ("POST_PLACE_BOTTOM_PALLET_DETECT", "到放置侧观测位，检测底层托盘"),
    ("PALLET_BOARD_REGION_DEVIATION", "到放置侧，测托盘相对车板区域偏差"),
    ("PALLET_CARGO_POST_PLACE_OFFSET", "到放置侧，测放货后货托偏差"),
    ("CARRY_TO_PLACEMENT_SIDE", "携货换到目标作业侧（安全高度）"),
    ("EXTEND_CARGO_FROM_OUTSIDE_TO_TARGET", "从车外侧伸出，把货物送到目标上方"),
    ("LOWER_CARGO_TO_TARGET", "下降到最终放置高度，准备放货"),
    ("OBSERVE_FACE_A", "到货物外侧，拍摄第一个相邻面"),
    ("EXTEND_INWARD_OBSERVE_FACE_B", "向车板内侧伸入，拍摄第二个相邻面"),
    ("RETURN_TO_LEFT_TAIL", "返回车尾左侧初始位，准备下一轮"),
    ("RETRACT", "缩回/回初始姿态"),
    ("FORK_PALLET", "执行托盘插取动作"),
    ("PLACE", "执行放货动作"),
    ("LIFT_CLEARANCE", "先升到安全高度，避免撞货"),
    ("LONGITUDINAL", "沿轨道纵向移动到目标 Y"),
    ("CROSSBEAM_TO_", "横梁换侧到目标作业侧"),
    ("LOWER_", "在目标侧下降到作业高度"),
)


def describe_motion(cmd: dict[str, Any]) -> tuple[str, str, str]:
    """返回 (步骤标题, 动作名, 目的说明)。"""
    step = str(cmd.get("step") or "").strip()
    task = str(cmd.get("task") or "").strip() or "-"
    step_title = STEP_TITLES.get(step, step or "（未知步骤）")
    purpose = "移动机械臂到目标位姿"
    for key, text in _TASK_PURPOSE:
        if task == key or task.startswith(key):
            purpose = text
            break
    return step_title, task, purpose


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
QLabel#stepLabel {
    color: #ffffff;
    font-size: 16px;
    font-weight: 700;
}
QLabel#purposeLabel {
    color: #e8e8e8;
    font-size: 14px;
}
QLabel#summaryLabel {
    color: #ffffff;
    font-size: 15px;
    font-weight: 700;
    padding: 4px 0;
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
    """待处理指令时阻塞主界面下一步；确认下发成功或关闭后才可继续。"""

    resolved = Signal()

    def __init__(
        self,
        parent=None,
        *,
        on_confirm_push: Callable[[], dict] | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("PLC 运动坐标 / 下发")
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.resize(620, 560)
        self.setStyleSheet(_DIALOG_QSS)
        font = QFont("Microsoft YaHei UI", 11)
        self.setFont(font)
        self._on_confirm_push = on_confirm_push
        self._latest_cmd: dict[str, Any] | None = None
        self._blocking = False
        self._build()

    def is_blocking(self) -> bool:
        return bool(self._blocking and self.isVisible())

    def _build(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)

        tip = QLabel(
            "主系统只发布坐标；真机由独立「PLC 控制台」执行。\n"
            "弹窗打开期间请先「确认下发」或「跳过并关闭」，才能继续执行下一步。"
        )
        tip.setObjectName("tipLabel")
        tip.setWordWrap(True)
        layout.addWidget(tip)

        self.step_label = QLabel("流程步骤：—")
        self.step_label.setObjectName("stepLabel")
        self.step_label.setWordWrap(True)
        self.step_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.step_label)

        self.purpose_label = QLabel("移动目的：—")
        self.purpose_label.setObjectName("purposeLabel")
        self.purpose_label.setWordWrap(True)
        self.purpose_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.purpose_label)

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
        self.close_btn = QPushButton("跳过下发并关闭")
        self.close_btn.clicked.connect(self.dismiss)
        row.addWidget(self.confirm_btn)
        row.addWidget(self.open_console_btn)
        row.addStretch(1)
        row.addWidget(self.close_btn)
        layout.addLayout(row)

    def clear_command(self) -> None:
        self._latest_cmd = None
        self.step_label.setText("流程步骤：—")
        self.purpose_label.setText("移动目的：—")
        self.summary.setText("等待流程产生运动目标…")
        self.cmd_view.clear()
        self.push_status.setText("下发状态：未下发")
        self.confirm_btn.setEnabled(False)
        self._end_blocking(emit_resolved=False)

    def apply_command(self, cmd: dict, *, auto_pushed: bool = False, push_result: dict | None = None) -> None:
        self._latest_cmd = dict(cmd or {})
        step_title, task, purpose = describe_motion(self._latest_cmd)
        world = self._latest_cmd.get("world_pose") or {}
        xyzr = self._latest_cmd.get("gantry_xyzr")

        self.step_label.setText(f"流程步骤：{step_title}")
        self.purpose_label.setText(f"移动动作：{task}\n移动目的：{purpose}")
        summary = (
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
                self._end_blocking(emit_resolved=True)
                return
            self.push_status.setText(f"下发状态：自动推送失败 · {push.get('message', '')}")
        else:
            self.push_status.setText("下发状态：待处理（请确认下发，或跳过并关闭）")

        self._begin_blocking()

    def _begin_blocking(self) -> None:
        self._blocking = True
        self.setWindowModality(Qt.WindowModality.ApplicationModal)
        if not self.isVisible():
            self.show()
        self.raise_()
        self.activateWindow()

    def _end_blocking(self, *, emit_resolved: bool) -> None:
        was = self._blocking
        self._blocking = False
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.hide()
        if emit_resolved and was:
            self.resolved.emit()

    def dismiss(self) -> None:
        """跳过下发并关闭，允许主流程继续。"""
        self.push_status.setText("下发状态：已跳过")
        self._end_blocking(emit_resolved=True)

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
            self._end_blocking(emit_resolved=True)
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
            self.push_status.setText("下发状态：已尝试启动 plc_console（请在本窗确认或跳过）")
        except Exception as exc:
            QMessageBox.warning(self, "启动失败", str(exc))

    def closeEvent(self, event):
        event.ignore()
        self.dismiss()
