"""PLC Modbus TCP监控与受限模式控制核心。

除PlcModeClient允许D5写入0或2外，不提供任何PLC写入能力。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Iterable, Optional


DEFAULT_IP = "192.168.6.6"
DEFAULT_PORT = 502
DEFAULT_DEVICE_ID = 1
MODE_ADDRESS = 5
ALLOWED_CONTROL_MODES = (0, 2)

SYSTEM_ADDRESS = 5
SYSTEM_COUNT = 3
ALARM_ADDRESSES = {
    "X": 119,
    "Y": 219,
    "Z": 319,
    "R": 419,
}
ALARM_COUNT = 5
POSITION_ADDRESS = 47234
POSITION_COUNT = 18
POSITION_OFFSETS = {
    "X": 0,
    "Y": 4,
    "Z": 12,
    "R": 16,
}
SPEED_ADDRESSES = {
    "X": 29680,
    "Y": 29700,
    "Z": 29740,
    "R": 29760,
}


class PlcReadError(RuntimeError):
    """PLC连接或读取失败。"""


class ModeSwitchError(RuntimeError):
    """受限的D5控制模式切换失败。"""


@dataclass(frozen=True)
class AxisAlarm:
    positive_limit: bool
    negative_limit: bool
    driver_alarm: bool

    @property
    def any_alarm(self) -> bool:
        return (
            self.positive_limit
            or self.negative_limit
            or self.driver_alarm
        )


@dataclass(frozen=True)
class PlcSnapshot:
    mode: int
    estop: int
    positions: Dict[str, int]
    speeds: Dict[str, int]
    alarms: Dict[str, AxisAlarm]


def _mode_text(value: int) -> str:
    return {0: "本地模式", 2: "上位机模式"}.get(
        value,
        f"未知模式({value})",
    )


def _estop_text(value: int) -> str:
    return "正常" if value == 0 else "已触发"


def _alarm_text(alarm: AxisAlarm) -> str:
    details = []
    if alarm.positive_limit:
        details.append("正限位")
    if alarm.negative_limit:
        details.append("负限位")
    if alarm.driver_alarm:
        details.append("驱动器报警")
    return " / ".join(details) if details else "正常"


def describe_snapshot_changes(
    previous: Optional[PlcSnapshot],
    current: PlcSnapshot,
) -> list[str]:
    """返回两个有效快照之间值得记录的状态变化。"""
    if previous is None:
        return []

    messages = []
    if previous.mode != current.mode:
        messages.append(
            "控制模式变化："
            f"{_mode_text(previous.mode)} -> {_mode_text(current.mode)}"
        )

    if previous.estop != current.estop:
        messages.append(
            "上位机急停变化："
            f"{_estop_text(previous.estop)} -> {_estop_text(current.estop)}"
        )

    for axis_name in ("X", "Y", "Z", "R"):
        old_alarm = previous.alarms[axis_name]
        new_alarm = current.alarms[axis_name]
        if old_alarm == new_alarm:
            continue

        old_text = _alarm_text(old_alarm)
        new_text = _alarm_text(new_alarm)
        if not old_alarm.any_alarm and new_alarm.any_alarm:
            messages.append(f"{axis_name}轴报警：{new_text}")
        elif old_alarm.any_alarm and not new_alarm.any_alarm:
            messages.append(
                f"{axis_name}轴报警解除（原状态：{old_text}）"
            )
        else:
            messages.append(
                f"{axis_name}轴报警变化：{old_text} -> {new_text}"
            )

    return messages


def signed_32(value: int) -> int:
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def decode_dint_low_word_first(registers: Iterable[int]) -> int:
    words = list(registers)
    if len(words) != 2:
        raise ValueError("DINT解码必须提供两个16位寄存器")

    low_word = words[0] & 0xFFFF
    high_word = words[1] & 0xFFFF
    return signed_32((high_word << 16) | low_word)


class PlcReadonlyClient:
    """仅提供连接、读取快照和关闭操作的PLC客户端。"""

    def __init__(
        self,
        host: str = DEFAULT_IP,
        port: int = DEFAULT_PORT,
        device_id: int = DEFAULT_DEVICE_ID,
        timeout: float = 0.5,
        client_factory: Optional[Callable[..., object]] = None,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.device_id = int(device_id)
        self.timeout = float(timeout)
        self._client_factory = client_factory
        self._client = None

    def open(self) -> bool:
        factory = self._client_factory
        if factory is None:
            try:
                from pymodbus.client import ModbusTcpClient
            except ImportError as exc:
                raise PlcReadError(
                    "缺少pymodbus，请在qt5环境中安装pymodbus。"
                ) from exc
            factory = ModbusTcpClient

        try:
            self._client = factory(
                self.host,
                port=self.port,
                timeout=self.timeout,
                retries=0,
            )
            return bool(self._client.connect())
        except Exception as exc:
            self._client = None
            raise PlcReadError(
                f"连接{self.host}:{self.port}失败：{exc}"
            ) from exc

    def close(self) -> None:
        client = self._client
        self._client = None
        if client is not None:
            try:
                client.close()
            except Exception:
                pass

    def _read(self, address: int, count: int) -> list[int]:
        if self._client is None:
            raise PlcReadError("PLC尚未连接")

        try:
            try:
                response = self._client.read_holding_registers(
                    address,
                    count=count,
                    device_id=self.device_id,
                )
            except TypeError as exc:
                if "device_id" not in str(exc):
                    raise
                response = self._client.read_holding_registers(
                    address,
                    count=count,
                    slave=self.device_id,
                )
        except Exception as exc:
            raise PlcReadError(f"读取地址{address}失败：{exc}") from exc

        if hasattr(response, "isError") and response.isError():
            raise PlcReadError(f"读取地址{address}失败：{response}")

        registers = list(getattr(response, "registers", []))
        if len(registers) != count:
            raise PlcReadError(
                f"读取地址{address}失败：期望{count}个寄存器，"
                f"实际收到{len(registers)}个"
            )
        return registers

    def read_snapshot(self) -> PlcSnapshot:
        system = self._read(SYSTEM_ADDRESS, SYSTEM_COUNT)

        alarms = {}
        for axis_name, address in ALARM_ADDRESSES.items():
            values = self._read(address, ALARM_COUNT)
            alarms[axis_name] = AxisAlarm(
                positive_limit=values[0] != 0,
                negative_limit=values[2] != 0,
                driver_alarm=values[4] != 0,
            )

        raw_positions = self._read(POSITION_ADDRESS, POSITION_COUNT)
        positions = {
            axis_name: decode_dint_low_word_first(
                raw_positions[offset : offset + 2]
            )
            for axis_name, offset in POSITION_OFFSETS.items()
        }

        speeds = {}
        for axis_name, address in SPEED_ADDRESSES.items():
            speeds[axis_name] = decode_dint_low_word_first(
                self._read(address, 2)
            )

        return PlcSnapshot(
            mode=system[0],
            estop=system[2],
            positions=positions,
            speeds=speeds,
            alarms=alarms,
        )


class PlcModeClient(PlcReadonlyClient):
    """在只读监控基础上，仅允许切换D5控制模式。"""

    def read_control_mode(self) -> int:
        return self._read(MODE_ADDRESS, 1)[0]

    def _write_control_mode(self, value: int) -> None:
        if self._client is None:
            raise ModeSwitchError("PLC尚未连接")

        try:
            try:
                response = self._client.write_register(
                    MODE_ADDRESS,
                    value,
                    device_id=self.device_id,
                )
            except TypeError as exc:
                if "device_id" not in str(exc):
                    raise
                response = self._client.write_register(
                    MODE_ADDRESS,
                    value,
                    slave=self.device_id,
                )
        except Exception as exc:
            raise ModeSwitchError(f"写入D5失败：{exc}") from exc

        if hasattr(response, "isError") and response.isError():
            raise ModeSwitchError(f"写入D5失败：{response}")

    def _check_upper_mode_safety(self) -> int:
        system = self._read(SYSTEM_ADDRESS, SYSTEM_COUNT)
        current_mode = system[0]
        if current_mode not in ALLOWED_CONTROL_MODES:
            raise ModeSwitchError(f"当前D5模式值异常：{current_mode}")
        if system[2] != 0:
            raise ModeSwitchError(f"上位机急停未解除，D7={system[2]}")

        for axis_name, address in ALARM_ADDRESSES.items():
            values = self._read(address, ALARM_COUNT)
            if values[0] != 0 or values[2] != 0 or values[4] != 0:
                raise ModeSwitchError(f"{axis_name}轴存在限位或驱动器报警")
        return current_mode

    def set_control_mode(self, target_mode: int) -> int:
        if target_mode not in ALLOWED_CONTROL_MODES:
            raise ValueError("D5控制模式只允许写入0或2")

        if target_mode == 2:
            current_mode = self._check_upper_mode_safety()
        else:
            current_mode = self.read_control_mode()

        if current_mode == target_mode:
            return current_mode

        self._write_control_mode(target_mode)
        verified_mode = self.read_control_mode()
        if verified_mode != target_mode:
            if target_mode == 2:
                try:
                    self._write_control_mode(0)
                except ModeSwitchError:
                    pass
            raise ModeSwitchError(
                f"D5回读验证失败：期望{target_mode}，实际{verified_mode}"
            )
        return verified_mode
