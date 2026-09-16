# -*- coding: utf-8 -*-
"""独立 PLC 控制台窗口。"""
from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
)

from config.external_devices_config import GANTRY, PLC
from plc_console.bridge import PlcBridgeServer
from plc_console.motion import PlcMotionRunner


class _MotionWorker(QThread):
    finished_ok = Signal(dict)
    finished_err = Signal(str)

    def __init__(self, runner: PlcMotionRunner, cmd: dict, parent=None):
        super().__init__(parent)
        self.runner = runner
        self.cmd = dict(cmd)

    def run(self):
        try:
            result = self.runner.execute_command(self.cmd)
            if result.get("success"):
                self.finished_ok.emit(result)
            else:
                self.finished_err.emit(str(result.get("message") or "执行失败"))
        except Exception as exc:
            self.finished_err.emit(str(exc))


class PlcConsoleWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PLC 运动控制台 · 接收主系统坐标并驱动龙门架")
        self.resize(820, 640)
        self.runner = PlcMotionRunner({"plc": PLC, "gantry": GANTRY})
        self.bridge = PlcBridgeServer(parent=self)
        self.bridge.commandReceived.connect(self._on_command)
        self.bridge.clientMessage.connect(lambda m: self._log(f"[bridge] {m}"))
        self._pending: dict | None = None
        self._exec_queue: list[dict] = []
        self._worker: _MotionWorker | None = None
        self._build()
        started = self.bridge.start()
        self._log(started.get("message", ""))
        self.bridge_status.setText("桥接：" + ("在线" if started.get("success") else "失败"))

    def _build(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        top = QHBoxLayout()
        self.conn_btn = QPushButton("连接 PLC")
        self.conn_btn.clicked.connect(self._toggle_plc)
        self.bridge_status = QLabel("桥接：-")
        self.plc_status = QLabel("PLC：未连接")
        self.auto_exec = QCheckBox("收到指令后自动执行")
        self.auto_exec.setChecked(True)
        top.addWidget(self.conn_btn)
        top.addWidget(self.plc_status)
        top.addWidget(self.bridge_status)
        top.addStretch(1)
        top.addWidget(self.auto_exec)
        layout.addLayout(top)

        form_box = QGroupBox("当前 / 手动目标 XYZR")
        form = QFormLayout(form_box)
        self.spin = {}
        for axis, lo, hi, val in (
            ("X", -10000.0, 10000.0, 0.0),
            ("Y", -10000.0, 10000.0, 0.0),
            ("Z", -10000.0, 10000.0, 0.0),
            ("R", -180.0, 180.0, 0.0),
        ):
            sp = QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setDecimals(2)
            sp.setValue(val)
            self.spin[axis] = sp
            form.addRow(axis, sp)
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(1.0, 200.0)
        self.speed_spin.setValue(float(GANTRY.get("default_speed", 30.0)))
        form.addRow("速度", self.speed_spin)
        layout.addWidget(form_box)

        self.cmd_view = QTextEdit()
        self.cmd_view.setReadOnly(True)
        self.cmd_view.setPlaceholderText("等待主系统推送运动指令…")
        self.cmd_view.setMaximumHeight(180)
        layout.addWidget(self.cmd_view)

        btns = QHBoxLayout()
        self.exec_btn = QPushButton("执行当前指令")
        self.exec_btn.clicked.connect(self._execute_pending)
        self.manual_btn = QPushButton("按上方 XYZR 绝对定位")
        self.manual_btn.clicked.connect(self._execute_manual)
        self.import_btn = QPushButton("导入 JSON")
        self.import_btn.clicked.connect(self._import_json)
        self.refresh_btn = QPushButton("读当前位置")
        self.refresh_btn.clicked.connect(self._read_positions)
        for b in (self.exec_btn, self.manual_btn, self.import_btn, self.refresh_btn):
            btns.addWidget(b)
        layout.addLayout(btns)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log, 1)

        tip = QLabel(
            f"主系统推送通道：本地 IPC「jushenzhineng-plc-console」。"
            f"默认 PLC {PLC.get('ip')}:{PLC.get('port')}。"
            "主流程推送成功即可下一步；到位结果只在本窗口显示。"
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#666;")
        layout.addWidget(tip)

    def _log(self, text: str) -> None:
        self.log.append(str(text))

    def _toggle_plc(self) -> None:
        if self.runner.connected:
            self.runner.disconnect()
            self.plc_status.setText("PLC：未连接")
            self.conn_btn.setText("连接 PLC")
            self._log("已断开 PLC")
            return
        result = self.runner.connect()
        if result.get("success"):
            self.plc_status.setText(f"PLC：已连接 {self.runner.host}:{self.runner.port}")
            self.conn_btn.setText("断开 PLC")
            self._log(result.get("message", "已连接"))
            self._read_positions()
        else:
            QMessageBox.warning(self, "连接失败", result.get("message", "未知错误"))

    def _read_positions(self) -> None:
        try:
            pos = self.runner.read_positions()
            for axis, value in pos.items():
                if axis in self.spin:
                    self.spin[axis].setValue(float(value))
            self._log(f"当前位置：{pos}")
        except Exception as exc:
            QMessageBox.warning(self, "读取失败", str(exc))

    def _on_command(self, cmd: dict) -> None:
        data = dict(cmd)
        self._show_cmd(data)
        self._log(f"收到指令 {data.get('cmd_id')} task={data.get('task')}")
        if self.auto_exec.isChecked():
            self._enqueue_or_run(data)
        else:
            self._pending = data

    def _show_cmd(self, cmd: dict) -> None:
        self.cmd_view.setPlainText(json.dumps(cmd, ensure_ascii=False, indent=2))
        xyzr = cmd.get("gantry_xyzr")
        if isinstance(xyzr, dict):
            for axis in ("X", "Y", "Z", "R"):
                if axis in xyzr and axis in self.spin:
                    self.spin[axis].setValue(float(xyzr[axis]))

    def _enqueue_or_run(self, cmd: dict) -> None:
        if self._worker and self._worker.isRunning():
            self._exec_queue.append(dict(cmd))
            self._log(f"上一段尚未到位，已排队（队列 {len(self._exec_queue)}）")
            return
        self._pending = dict(cmd)
        self._start_worker(self._pending)

    def _execute_pending(self) -> None:
        if not self._pending:
            QMessageBox.information(self, "提示", "当前没有待执行指令")
            return
        self._enqueue_or_run(self._pending)

    def _execute_manual(self) -> None:
        cmd = {
            "schema": "plc_motion_cmd_v1",
            "type": "MOTION_CMD",
            "cmd_id": "MANUAL",
            "task": "MANUAL_ABS",
            "gantry_xyzr": {axis: float(self.spin[axis].value()) for axis in ("X", "Y", "Z", "R")},
            "speed": float(self.speed_spin.value()),
        }
        self._pending = cmd
        self.cmd_view.setPlainText(json.dumps(cmd, ensure_ascii=False, indent=2))
        self._start_worker(cmd)

    def _import_json(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "导入运动指令 JSON",
            str(Path(__file__).resolve().parents[2] / "workdir" / "plc_commands"),
            "JSON (*.json *.jsonl)",
        )
        if not path:
            return
        text = Path(path).read_text(encoding="utf-8")
        if path.endswith(".jsonl"):
            lines = [ln for ln in text.splitlines() if ln.strip()]
            text = lines[-1] if lines else "{}"
        try:
            cmd = json.loads(text)
        except Exception as exc:
            QMessageBox.warning(self, "导入失败", str(exc))
            return
        self._on_command(cmd)

    def _start_worker(self, cmd: dict) -> None:
        if self._worker and self._worker.isRunning():
            self._exec_queue.append(dict(cmd))
            self._log(f"运动中，新目标已排队（队列 {len(self._exec_queue)}）")
            return
        if not self.runner.connected:
            linked = self.runner.connect()
            if not linked.get("success"):
                QMessageBox.warning(self, "未连接", linked.get("message", "请先连接 PLC"))
                return
            self.plc_status.setText(f"PLC：已连接 {self.runner.host}:{self.runner.port}")
            self.conn_btn.setText("断开 PLC")
        payload = dict(cmd)
        payload["speed"] = float(self.speed_spin.value())
        self.exec_btn.setEnabled(False)
        self.manual_btn.setEnabled(False)
        self._log(f"开始执行 {payload.get('cmd_id')} …")
        self._worker = _MotionWorker(self.runner, payload, self)
        self._worker.finished_ok.connect(self._on_exec_ok)
        self._worker.finished_err.connect(self._on_exec_err)
        self._worker.finished.connect(self._on_worker_done)
        self._worker.start()

    def _on_exec_ok(self, result: dict) -> None:
        self._log(f"执行成功：{result.get('message')} targets={result.get('targets')}")
        try:
            self._read_positions()
        except Exception:
            pass

    def _on_exec_err(self, message: str) -> None:
        self._log(f"执行失败：{message}")
        QMessageBox.warning(self, "运动失败", message)

    def _on_worker_done(self) -> None:
        self.exec_btn.setEnabled(True)
        self.manual_btn.setEnabled(True)
        if self._exec_queue:
            nxt = self._exec_queue.pop(0)
            self._pending = nxt
            self._show_cmd(nxt)
            self._log(f"执行队列下一段，剩余 {len(self._exec_queue)}")
            self._start_worker(nxt)

    def closeEvent(self, event):
        self.bridge.stop()
        self.runner.disconnect()
        super().closeEvent(event)
