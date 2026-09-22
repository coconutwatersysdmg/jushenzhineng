# -*- coding: utf-8 -*-
"""主系统 PLC 运动指令弹出窗口：坐标显示 / 确认下发。"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Mapping

from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QHeaderView,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]

# 流程步骤代码 → 界面文案（与 FlowController 一致）
STEP_TITLES = {
    "DEVICE_CHECK": "1. 外接设备连接检查",
    "LAB_SENSE": "2. 相机插孔识别 ∥ 雷达四角",
    "LAB_CORNER_SHELL": "3. 相机精定位（拍照+YOLO）",
    "LAB_RETURN_ORIGIN": "返回实验室原点（R保持）",
    "LAB_PRE_PLACE_MONITOR": "上一 B 区放货前监测",
    "LAB_PLACE_VERIFY": "当前 B 区手动放货后检测",
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

# 运动 task → 中文目的（前缀匹配，长的在前）
_TASK_PURPOSE = (
    ("LAB_RETURN_ORIGIN", "返回实验室原点，R轴保持启动角度"),
    ("LAB_PRE_PLACE_", "到上一 B 区几何中心做放货前监测"),
    ("LAB_POST_PLACE_", "到当前 B 区几何中心做放货后检测"),
    ("PALLET_HOLE_LEFT", "左插孔目标坐标（可改后下发）"),
    ("PALLET_HOLE_RIGHT", "右插孔目标坐标（可改后下发）"),
    ("MOVE_TO_TAIL_STAGED_CARGO", "移到车尾待装货物上方，准备找插孔"),
    ("FIND_PALLET_HOLE", "到插孔识别位，便于相机定位插孔"),
    ("LIFT_STAGED_CARGO", "插取完成后抬升带货，离开货位"),
    ("CAPTURE_TAIL", "到车尾左侧外侧，拍摄车尾角点"),
    ("CAPTURE_HEAD", "到车头左侧外侧，拍摄车头角点"),
    ("EXTEND_CARGO_FROM_OUTSIDE_TO_TARGET", "从车外侧伸出，把货物送到目标上方"),
    ("LOWER_CARGO_TO_TARGET", "下降到最终放置高度，准备放货"),
    ("EXTEND_INWARD_OBSERVE_FACE_B", "向车板内侧伸入，拍摄第二个相邻面"),
    ("POST_PLACE_BOTTOM_PALLET_DETECT", "到放置侧观测位，检测底层托盘"),
    ("PRE_PLACE_DYNAMIC_MONITOR", "到放置侧观测位，做放货前动态监测"),
    ("PALLET_BOARD_REGION_DEVIATION", "到放置侧，测托盘相对车板区域偏差"),
    ("PALLET_CARGO_POST_PLACE_OFFSET", "到放置侧，测放货后货托偏差"),
    ("CARRY_TO_PLACEMENT_SIDE", "携货换到目标作业侧（安全高度）"),
    ("NEIGHBOR_PALLET_POSE", "到目标侧附近，拍摄临近已放托盘姿态"),
    ("RETURN_TO_LEFT_TAIL", "返回车尾左侧初始位，准备下一轮"),
    ("OBSERVE_FACE_A", "到货物外侧，拍摄第一个相邻面"),
    ("FORK_PALLET", "执行托盘插取动作"),
    ("RETRACT", "缩回，回到初始姿态"),
    ("PLACE", "执行放货动作"),
)

_PHASE_PURPOSE = (
    ("LIFT_CLEARANCE", "先升到安全高度，避免撞货"),
    ("LONGITUDINAL", "沿轨道纵向移动到目标位置"),
    ("CROSSBEAM_TO_RIGHT", "横梁换到右侧作业"),
    ("CROSSBEAM_TO_LEFT", "横梁换到左侧作业"),
    ("LOWER_RIGHT", "在右侧下降到作业高度"),
    ("LOWER_LEFT", "在左侧下降到作业高度"),
)


def format_waypoint_line(index: int, total: int, cmd: Mapping[str, Any]) -> str:
    _, purpose = describe_motion(cmd)
    xyzr = cmd.get("gantry_xyzr")
    world = cmd.get("world_pose") or {}
    if isinstance(xyzr, Mapping):
        pose = (
            f"XYZR=({float(xyzr.get('X' ,0)):.1f}, {float(xyzr.get('Y', 0)):.1f}, "
            f"{float(xyzr.get('Z', 0)):.1f}, {float(xyzr.get('R', 0)):.1f})"
        )
    else:
        pose = (
            f"WORLD=({float(world.get('x_mm', 0)):.1f}, {float(world.get('y_mm', 0)):.1f}, "
            f"{float(world.get('z_mm', 0)):.1f})"
        )
    return f"{index}/{total}  {purpose}\n    {pose}"


def describe_motion(cmd: Mapping[str, Any]) -> tuple[str, str]:
    """返回 (步骤标题, 中文目的)。"""
    step = str(cmd.get("step") or "").strip()
    task = str(cmd.get("task") or "").strip()
    step_title = STEP_TITLES.get(step, step or "（未知步骤）")
    purpose = "移动机械臂到目标位姿"
    for key, text in _PHASE_PURPOSE:
        if task.endswith(key) or key in task:
            purpose = text
            return step_title, purpose
    for key, text in _TASK_PURPOSE:
        if task == key or task.startswith(key):
            purpose = text
            break
    return step_title, purpose


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

    resolved = Signal(bool)  # skip_remaining：关闭/跳过则丢掉本步剩余坐标，不再立刻再弹

    def __init__(
        self,
        parent=None,
        *,
        on_confirm_push: Callable[[], dict] | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("PLC 运动坐标 / 下发")
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, True)
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.resize(620, 560)
        self.setStyleSheet(_DIALOG_QSS)
        font = QFont("Microsoft YaHei UI", 11)
        self.setFont(font)
        self._on_confirm_push = on_confirm_push
        self._latest_cmd: dict[str, Any] | None = None
        self._latest_cmds: list[dict[str, Any]] = []
        self._blocking = False
        self._closing = False
        self._build()

    def is_blocking(self) -> bool:
        return bool(self._blocking and self.isVisible())

    def _build(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)

        self.step_label = QLabel("流程步骤：—")
        self.step_label.setObjectName("stepLabel")
        self.step_label.setWordWrap(True)
        self.step_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.step_label)

        self.purpose_label = QLabel("本步移动段：—")
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
        self.cmd_view.setMaximumHeight(140)
        layout.addWidget(self.cmd_view, 1)

        edit_box = QGroupBox("可编辑下发坐标 XYZR（修改后点确认生效）")
        edit_layout = QVBoxLayout(edit_box)
        tip = QLabel("每段一行；确认下发前可直接改表中数值。")
        tip.setStyleSheet("color:#aaaaaa;")
        edit_layout.addWidget(tip)
        self.xyzr_table = QTableWidget(0, 5)
        self.xyzr_table.setHorizontalHeaderLabels(["段/任务", "X", "Y", "Z", "R"])
        self.xyzr_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for col in (1, 2, 3, 4):
            self.xyzr_table.horizontalHeader().setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)
        self.xyzr_table.setMinimumHeight(110)
        edit_layout.addWidget(self.xyzr_table)
        layout.addWidget(edit_box)

        self.push_status = QLabel("下发状态：未下发")
        self.push_status.setObjectName("statusLabel")
        layout.addWidget(self.push_status)

        row = QHBoxLayout()
        row.setSpacing(10)
        self.confirm_btn = QPushButton("确认下发到 PLC")
        self.confirm_btn.setObjectName("primaryBtn")
        self.confirm_btn.setEnabled(False)
        self.confirm_btn.clicked.connect(self._confirm)
        self.open_console_btn = QPushButton("备用：打开控制台")
        self.open_console_btn.setToolTip("主系统 PLC 已连接时可直接下发，无需打开。仅在本地通道不可用时作备用。")
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
        self._latest_cmds = []
        self.step_label.setText("流程步骤：—")
        self.purpose_label.setText("本步移动段：—")
        self.summary.setText("等待流程产生运动目标…")
        self.cmd_view.clear()
        self.xyzr_table.setRowCount(0)
        self.push_status.setText("下发状态：未下发")
        self.confirm_btn.setText("确认下发到 PLC")
        self.confirm_btn.setEnabled(False)
        self._end_blocking(emit_resolved=False, skip_remaining=False)

    def apply_command(self, cmd: dict, *, auto_pushed: bool = False, push_result: dict | None = None) -> None:
        self.apply_batch([cmd], auto_pushed=auto_pushed, push_result=push_result)

    def _fill_xyzr_table(self, batch: list) -> None:
        self.xyzr_table.setRowCount(0)
        for i, cmd in enumerate(batch, 1):
            _, purpose = describe_motion(cmd)
            xyzr = cmd.get("gantry_xyzr") if isinstance(cmd.get("gantry_xyzr"), Mapping) else {}
            world = cmd.get("world_pose") or {}
            values = {
                "X": float(xyzr.get("X", world.get("x_mm", 0.0)) or 0.0),
                "Y": float(xyzr.get("Y", world.get("y_mm", 0.0)) or 0.0),
                "Z": float(xyzr.get("Z", world.get("z_mm", 0.0)) or 0.0),
                "R": float(xyzr.get("R", world.get("yaw_deg", 0.0)) or 0.0),
            }
            row = self.xyzr_table.rowCount()
            self.xyzr_table.insertRow(row)
            label = QTableWidgetItem(f"{i}/{len(batch)} {purpose}")
            label.setFlags(label.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.xyzr_table.setItem(row, 0, label)
            for col, axis in enumerate(("X", "Y", "Z", "R"), 1):
                item = QTableWidgetItem(f"{values[axis]:.3f}")
                self.xyzr_table.setItem(row, col, item)

    def _read_xyzr_edits(self) -> list[dict]:
        """把表中编辑写回 _latest_cmds 的 gantry_xyzr，并返回副本。"""
        out = []
        for row, cmd in enumerate(self._latest_cmds):
            data = dict(cmd or {})
            if row >= self.xyzr_table.rowCount():
                out.append(data)
                continue
            try:
                xyzr = {
                    "X": float(self.xyzr_table.item(row, 1).text()),
                    "Y": float(self.xyzr_table.item(row, 2).text()),
                    "Z": float(self.xyzr_table.item(row, 3).text()),
                    "R": float(self.xyzr_table.item(row, 4).text()),
                }
            except Exception:
                xyzr = dict(data.get("gantry_xyzr") or {})
            data["gantry_xyzr"] = xyzr
            # 同步粗略回写 world，避免只改了表但 JSON 仍显示旧值
            world = dict(data.get("world_pose") or {})
            world["x_mm"] = float(xyzr.get("X", 0.0))
            world["y_mm"] = float(xyzr.get("Y", 0.0))
            world["z_mm"] = float(xyzr.get("Z", 0.0))
            world["yaw_deg"] = float(xyzr.get("R", 0.0))
            data["world_pose"] = world
            out.append(data)
        self._latest_cmds = out
        self._latest_cmd = out[0] if out else None
        return out

    def apply_batch(self, cmds: list, *, auto_pushed: bool = False, push_result: dict | None = None) -> None:
        batch = [dict(item or {}) for item in (cmds or []) if item]
        if not batch:
            return
        self._latest_cmds = batch
        self._latest_cmd = batch[0]
        step_title, _ = describe_motion(batch[0])
        total = len(batch)
        lines = [format_waypoint_line(i, total, cmd) for i, cmd in enumerate(batch, 1)]
        self.step_label.setText(f"流程步骤：{step_title}")
        self.purpose_label.setText(f"本步共 {total} 段，按顺序执行（可先改 XYZR）：")
        self.summary.setText("\n\n".join(lines))
        self.cmd_view.setPlainText(json.dumps(batch if total > 1 else batch[0], ensure_ascii=False, indent=2))
        self._fill_xyzr_table(batch)
        self.confirm_btn.setEnabled(True)
        self.confirm_btn.setText("确认下发到 PLC" if total == 1 else f"确认按顺序下发全部 {total} 段")

        if auto_pushed:
            push = dict(push_result or {})
            if push.get("success"):
                self.push_status.setText(f"下发状态：已自动推送 · {push.get('message', 'OK')}")
                self._end_blocking(emit_resolved=True, skip_remaining=False)
                return
            self.push_status.setText(f"下发状态：自动推送失败 · {push.get('message', '')}")
        else:
            self.push_status.setText("下发状态：待处理（可改坐标后确认，按 1→N 顺序下发）")

        self._begin_blocking()

    def _begin_blocking(self) -> None:
        self._blocking = True
        self._closing = False
        if not self.isVisible():
            self.show()
        self.raise_()
        self.activateWindow()

    def _end_blocking(self, *, emit_resolved: bool, skip_remaining: bool = False) -> None:
        if self._closing:
            return
        was = self._blocking
        self._blocking = False
        self._closing = True
        try:
            if self.isVisible():
                self.hide()
        finally:
            # 保持 _closing 直到本轮事件结束，避免 hide 触发的 closeEvent 再走一遍
            QTimer.singleShot(0, self._clear_closing)
        if emit_resolved and was:
            QTimer.singleShot(0, lambda: self.resolved.emit(bool(skip_remaining)))

    def _clear_closing(self) -> None:
        self._closing = False

    def dismiss(self) -> None:
        """跳过本条及本步剩余坐标并关闭。"""
        if self._closing:
            return
        self.push_status.setText("下发状态：已跳过")
        self._end_blocking(emit_resolved=True, skip_remaining=True)

    def set_push_status(self, text: str) -> None:
        self.push_status.setText(text)

    def _confirm(self) -> None:
        if not self._latest_cmd:
            QMessageBox.information(self, "提示", "当前没有可下发的运动坐标")
            return
        if not callable(self._on_confirm_push):
            return
        edited = self._read_xyzr_edits()
        self.cmd_view.setPlainText(json.dumps(edited if len(edited) > 1 else edited[0], ensure_ascii=False, indent=2))
        result = self._on_confirm_push() or {}
        if result.get("success"):
            self.push_status.setText(f"下发状态：已确认推送 · {result.get('message', 'OK')}")
            self._end_blocking(emit_resolved=True, skip_remaining=False)
        else:
            self.push_status.setText(f"下发状态：下发失败 · {result.get('message', '')}")
            QMessageBox.warning(self, "下发失败", result.get("message") or "PLC 未连接或写轴失败")

    def _open_console(self) -> None:
        python = PROJECT_ROOT / "runtime" / "python.exe"
        exe = str(python if python.is_file() else sys.executable)
        try:
            subprocess.Popen(
                [exe, "-m", "plc_console"],
                cwd=str(PROJECT_ROOT),
                env={**os.environ, "PYTHONUTF8": "1"},
            )
            self.push_status.setText("下发状态：已尝试启动备用控制台（主系统 PLC 已连时可直接点确认下发）")
        except Exception as exc:
            QMessageBox.warning(self, "启动失败", str(exc))

    def closeEvent(self, event):
        # 必须 accept，否则第一次关会被 Qt 留住，要点第二次才消失
        event.accept()
        if self._closing:
            return
        if self._blocking:
            self.push_status.setText("下发状态：已跳过")
            was = self._blocking
            self._blocking = False
            self._closing = True
            QTimer.singleShot(0, self._clear_closing)
            if was:
                QTimer.singleShot(0, lambda: self.resolved.emit(True))
