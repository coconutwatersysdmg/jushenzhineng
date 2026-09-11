# -*- coding: utf-8 -*-
"""龙门架绝对定位：复用 plc_finished_app 已验证的寄存器地址与写入时序。

对应 console「坐标控制 → 移动到目标坐标」：
写各轴目标/速度 → 触发脉冲 10→0 → 轮询到位。
"""
from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PLC_APP_DIR = PROJECT_ROOT / "third_party" / "plc_finished_app"

# 与 plc_finished_console.AXIS_CONFIG / 默认软限位一致
DEFAULT_SOFT_LIMITS = {
    "X": (0.0, 500.0),
    "Y": (0.0, 1300.0),
    "Z": (0.0, 380.0),
    "R": (-180.0, 180.0),
}

ABS_TARGET = {"X": 41208, "Y": 41308, "Z": 41408, "R": 41508}
ABS_SPEED = {"X": 41210, "Y": 41310, "Z": 41410, "R": 41510}
ABS_TRIGGER = {"X": 18, "Y": 28, "Z": 38, "R": 48}
ABS_DONE = {"X": 117, "Y": 217, "Z": 317, "R": 417}
POSITION = {"X": 47234, "Y": 47238, "Z": 47246, "R": 47250}
ACTUAL_SPEED = {"X": 29680, "Y": 29700, "Z": 29740, "R": 29760}
MODE_ADDRESS = 5
ESTOP_ADDRESS = 7
DEVICE_ID = 1


def _ensure_import_path() -> None:
    path = str(PLC_APP_DIR.resolve())
    if path not in sys.path:
        sys.path.insert(0, path)


def _encode_dint(value: int) -> list[int]:
    raw = int(value) & 0xFFFFFFFF
    return [raw & 0xFFFF, (raw >> 16) & 0xFFFF]


def _decode_dint(words) -> int:
    low = int(words[0]) & 0xFFFF
    high = int(words[1]) & 0xFFFF
    value = (high << 16) | low
    if value >= 0x80000000:
        value -= 0x100000000
    return int(value)


class GantryModbusMotion:
    """对现场 PLC 执行四轴绝对定位（需已连接且可写）。"""

    def __init__(self, host: str, port: int = 502, timeout: float = 1.0):
        _ensure_import_path()
        self.host = str(host)
        self.port = int(port)
        self.timeout = float(timeout)
        self._client = None

    def open(self) -> bool:
        try:
            from pymodbus.client import ModbusTcpClient
        except ImportError:
            try:
                from pymodbus.client.sync import ModbusTcpClient
            except ImportError as exc:
                raise RuntimeError("缺少 pymodbus，请 pip install pymodbus") from exc

        client = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout, retries=0)
        if not client.connect():
            self._client = None
            return False
        self._client = client
        return True

    def close(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None

    @property
    def connected(self) -> bool:
        return self._client is not None

    def _write_register(self, address: int, value: int) -> None:
        if self._client is None:
            raise RuntimeError("PLC 未连接")
        try:
            result = self._client.write_register(address, int(value), device_id=DEVICE_ID)
        except TypeError:
            result = self._client.write_register(address, int(value), slave=DEVICE_ID)
        if hasattr(result, "isError") and result.isError():
            raise RuntimeError(f"写寄存器 {address} 失败：{result}")

    def _write_dint(self, address: int, value: int) -> None:
        if self._client is None:
            raise RuntimeError("PLC 未连接")
        words = _encode_dint(int(round(value)))
        try:
            result = self._client.write_registers(address, words, device_id=DEVICE_ID)
        except TypeError:
            result = self._client.write_registers(address, words, slave=DEVICE_ID)
        if hasattr(result, "isError") and result.isError():
            raise RuntimeError(f"写 DINT {address} 失败：{result}")

    def _read_registers(self, address: int, count: int = 1):
        if self._client is None:
            raise RuntimeError("PLC 未连接")
        try:
            result = self._client.read_holding_registers(address, count=count, device_id=DEVICE_ID)
        except TypeError:
            result = self._client.read_holding_registers(address, count=count, slave=DEVICE_ID)
        if hasattr(result, "isError") and result.isError():
            raise RuntimeError(f"读寄存器 {address} 失败：{result}")
        return list(result.registers)

    def _read_dint(self, address: int) -> int:
        return _decode_dint(self._read_registers(address, 2))

    def read_mode_estop(self) -> tuple[int, int]:
        regs = self._read_registers(MODE_ADDRESS, 3)
        return int(regs[0]), int(regs[2])

    def read_positions(self) -> dict[str, float]:
        return {axis: float(self._read_dint(addr)) for axis, addr in POSITION.items()}

    def ensure_upper_mode(self) -> None:
        mode, estop = self.read_mode_estop()
        if estop != 0:
            raise RuntimeError("上位机急停未解除，拒绝运动")
        if mode == 2:
            return
        self._write_register(MODE_ADDRESS, 2)
        mode, _ = self.read_mode_estop()
        if mode != 2:
            raise RuntimeError(f"切换上位机失败，当前模式={mode}")

    def validate_targets(
        self,
        targets: Mapping[str, float],
        soft_limits: Mapping[str, Any] | None = None,
    ) -> None:
        limits = dict(DEFAULT_SOFT_LIMITS)
        for axis, pair in dict(soft_limits or {}).items():
            if isinstance(pair, (list, tuple)) and len(pair) >= 2:
                limits[str(axis).upper()] = (float(pair[0]), float(pair[1]))
        for axis in ("X", "Y", "Z", "R"):
            if axis not in targets:
                raise RuntimeError(f"缺少轴目标 {axis}")
            value = float(targets[axis])
            lo, hi = limits[axis]
            if value < lo or value > hi:
                raise RuntimeError(f"{axis} 目标 {value:.2f} 超出软限位 [{lo}, {hi}]")

    def move_absolute(
        self,
        targets: Mapping[str, float],
        speed: float = 30.0,
        timeout_s: float = 60.0,
        soft_limits: Mapping[str, Any] | None = None,
        axes: tuple[str, ...] = ("X", "Y", "Z", "R"),
        position_tol: float = 1.0,
    ) -> dict[str, Any]:
        """写目标并等待各轴到位（与 console run_targets 时序一致）。"""
        if not self.connected and not self.open():
            raise RuntimeError(f"无法连接 PLC {self.host}:{self.port}")

        wanted = {axis: float(targets[axis]) for axis in axes}
        self.validate_targets(wanted, soft_limits)
        self.ensure_upper_mode()

        speed_i = max(1, int(round(float(speed))))
        for axis, target in wanted.items():
            self._write_dint(ABS_TARGET[axis], int(round(target)))
            self._write_dint(ABS_SPEED[axis], speed_i)

        for axis in wanted:
            self._write_register(ABS_TRIGGER[axis], 10)
        time.sleep(0.08)
        for axis in wanted:
            self._write_register(ABS_TRIGGER[axis], 0)

        deadline = time.time() + max(1.0, float(timeout_s))
        while time.time() < deadline:
            _, estop = self.read_mode_estop()
            if estop != 0:
                raise RuntimeError("运动过程中急停触发")
            positions = self.read_positions()
            all_ok = True
            for axis, target in wanted.items():
                near = abs(positions[axis] - target) <= position_tol
                try:
                    done = self._read_registers(ABS_DONE[axis], 1)[0] == 1
                except Exception:
                    done = False
                try:
                    spd = self._read_dint(ACTUAL_SPEED[axis])
                except Exception:
                    spd = None
                stopped = spd in (None, 0)
                if not (near and (done or stopped)):
                    all_ok = False
                    break
            if all_ok:
                return {
                    "success": True,
                    "targets": wanted,
                    "positions": positions,
                    "speed": speed_i,
                    "message": "四轴绝对定位完成",
                }
            time.sleep(0.1)

        positions = self.read_positions()
        raise RuntimeError(
            f"绝对定位超时（{timeout_s:.0f}s），目标={wanted}，当前={positions}"
        )
