"""PLC统一工程调试核心。

所有真实Modbus写入都集中在本模块，并受地址和值双重白名单约束。
"""

from __future__ import annotations

import time
import threading
import math
from dataclasses import dataclass
from typing import Callable

from plc_readonly_core import (
    PlcReadError,
    PlcReadonlyClient,
    decode_dint_low_word_first,
)


MODE_ADDRESS = 5
ESTOP_ADDRESS = 7
JOG_ADDRESSES = {"X": 12, "Y": 22, "Z": 32, "R": 42}
JOG_SPEED_ADDRESSES = {
    "X": 41188,
    "Y": 41288,
    "Z": 41388,
    "R": 41488,
}
RESET_ADDRESSES = {"X": 113, "Y": 213, "Z": 313, "R": 413}
HOME_ADDRESSES = {"X": 10, "Y": 20, "Z": 30, "R": 40}
HOME_DONE_ADDRESSES = {
    "X": 37868,
    "Y": 37888,
    "Z": 37928,
    "R": 37948,
}
HOME_STATIONARY_CONFIRM_SECONDS = 1.0
ABS_TRIGGER_ADDRESSES = {"X": 18, "Y": 28, "Z": 38, "R": 48}
ABS_DONE_ADDRESSES = {"X": 117, "Y": 217, "Z": 317, "R": 417}
ABS_TARGET_ADDRESSES = {
    "X": 41208,
    "Y": 41308,
    "Z": 41408,
    "R": 41508,
}
ABS_SPEED_ADDRESSES = {
    "X": 41210,
    "Y": 41310,
    "Z": 41410,
    "R": 41510,
}
REL_TRIGGER_ADDRESSES = {"X": 16, "Y": 26, "Z": 36, "R": 46}
REL_DONE_ADDRESSES = {"X": 115, "Y": 215, "Z": 315, "R": 415}
REL_DISTANCE_ADDRESSES = {
    "X": 41198,
    "Y": 41298,
    "Z": 41398,
    "R": 41498,
}
REL_SPEED_ADDRESSES = {
    "X": 41200,
    "Y": 41300,
    "Z": 41400,
    "R": 41500,
}
PAUSE_ADDRESSES = {"X": 129, "Y": 229, "Z": 329, "R": 429}
EXTENDED_TRIGGER_ADDRESSES = (
    *JOG_ADDRESSES.values(),
    *HOME_ADDRESSES.values(),
    *ABS_TRIGGER_ADDRESSES.values(),
    *REL_TRIGGER_ADDRESSES.values(),
    *PAUSE_ADDRESSES.values(),
)
EXTENDED_COMMAND_ADDRESSES = (
    *RESET_ADDRESSES.values(),
    *EXTENDED_TRIGGER_ADDRESSES,
)
COMMAND_ADDRESSES = {
    *RESET_ADDRESSES.values(),
    *HOME_ADDRESSES.values(),
    *ABS_TRIGGER_ADDRESSES.values(),
    *REL_TRIGGER_ADDRESSES.values(),
}
WRITABLE_DINT_ADDRESSES = {
    *JOG_SPEED_ADDRESSES.values(),
    *ABS_TARGET_ADDRESSES.values(),
    *ABS_SPEED_ADDRESSES.values(),
    *REL_DISTANCE_ADDRESSES.values(),
    *REL_SPEED_ADDRESSES.values(),
}
MOTION_TRIGGER_ADDRESSES = (
    10,
    12,
    16,
    18,
    20,
    22,
    26,
    28,
    30,
    32,
    36,
    38,
    40,
    42,
    46,
    48,
    129,
    229,
    329,
    429,
)
ACTUAL_SPEED_ADDRESSES = {
    "X": 29680,
    "Y": 29700,
    "Z": 29740,
    "R": 29760,
}
POSITION_ADDRESSES = {
    "X": 47234,
    "Y": 47238,
    "Z": 47246,
    "R": 47250,
}
RELATIVE_TEST_DISTANCE = 10
ALARM_ADDRESSES = {
    "X": 119,
    "Y": 219,
    "Z": 319,
    "R": 419,
}
SINGLE_WRITE_RULES = {
    MODE_ADDRESS: frozenset((0, 2)),
    ESTOP_ADDRESS: frozenset((0, 1)),
    **{address: frozenset((0, 10, 20)) for address in JOG_ADDRESSES.values()},
    **{address: frozenset((0, 10)) for address in COMMAND_ADDRESSES},
    **{address: frozenset((0, 1)) for address in PAUSE_ADDRESSES.values()},
}


class EngineeringSafetyError(RuntimeError):
    """工程测试的状态或写入不满足安全约束。"""


@dataclass(frozen=True)
class D7VerificationResult:
    before: int
    triggered: int
    restored: int

    @property
    def values(self) -> tuple[int, int, int]:
        return self.before, self.triggered, self.restored


@dataclass(frozen=True)
class JogRequest:
    axis: str
    direction_value: int
    speed: int
    duration: float

    def validate(self) -> None:
        if self.axis not in JOG_ADDRESSES:
            raise ValueError("轴只允许X/Y/Z/R")
        if self.direction_value not in (10, 20):
            raise ValueError("方向值只允许10或20")
        if not 1 <= int(self.speed) <= 5:
            raise ValueError("速度只允许1到5 mm/s")
        if not 0.2 <= float(self.duration) <= 2.0:
            raise ValueError("持续时间只允许0.2到2.0秒")
        if float(self.speed) * float(self.duration) > 10.0:
            raise ValueError("最大计划位移为10 mm")


@dataclass(frozen=True)
class CleanupReport:
    mode: int
    estop: int
    triggers: tuple[int, int, int, int]
    restored_speed: int
    errors: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return (
            not self.errors
            and self.mode == 0
            and self.estop == 0
            and self.triggers == (0, 0, 0, 0)
        )


@dataclass(frozen=True)
class JogResult:
    axis: str
    direction_value: int
    requested_speed: int
    duration: float
    start_position: int
    end_position: int
    speed_samples: tuple[int, ...]
    cleanup: CleanupReport

    @property
    def delta(self) -> int:
        return self.end_position - self.start_position

    @property
    def speed_range(self) -> tuple[int, int]:
        if not self.speed_samples:
            return 0, 0
        return min(self.speed_samples), max(self.speed_samples)


@dataclass(frozen=True)
class MotionSample:
    elapsed: float
    positions: tuple[int, int, int, int]
    speeds: tuple[int, int, int, int]


@dataclass(frozen=True)
class ExtendedCleanupReport:
    mode: int
    estop: int
    trigger_values: tuple[int, ...]
    parameter_values: tuple[tuple[int, int], ...]
    latched: bool
    errors: tuple[str, ...]


@dataclass(frozen=True)
class ResetResult:
    axis: str
    start_position: int
    end_position: int
    speed_samples: tuple[int, ...]
    command_values: tuple[int, int]
    cleanup: ExtendedCleanupReport


@dataclass(frozen=True)
class HomeResult:
    axis: str
    start_position: int
    end_position: int
    speed_samples: tuple[int, ...]
    done_samples: tuple[int, ...]
    cleanup: ExtendedCleanupReport


@dataclass(frozen=True)
class AbsoluteLegResult:
    axis: str
    target: int
    start_position: int
    end_position: int
    speed_samples: tuple[int, ...]
    done_samples: tuple[int, ...]


@dataclass(frozen=True)
class AbsoluteResult:
    axis: str
    outbound: AbsoluteLegResult
    returned: AbsoluteLegResult
    cleanup: ExtendedCleanupReport


@dataclass(frozen=True)
class RelativeResult:
    axis: str
    start_position: int
    outbound_position: int
    returned_position: int
    speed_samples: tuple[int, ...]
    done_samples: tuple[int, ...]
    cleanup: ExtendedCleanupReport


@dataclass(frozen=True)
class PauseResult:
    axis: str
    start_position: int
    paused_position: int
    resumed_position: int
    resumed: bool
    cleanup: ExtendedCleanupReport


@dataclass(frozen=True)
class DynamicD7Result:
    axis: str
    start_position: int
    stopped_position: int
    final_speed: int
    cleanup: ExtendedCleanupReport


@dataclass(frozen=True)
class MultiAxisLegResult:
    target: int
    samples: tuple[MotionSample, ...]
    trigger_skew_seconds: float
    cleanup: ExtendedCleanupReport


@dataclass(frozen=True)
class MultiAxisTargetLegResult:
    targets: tuple[int, int, int, int]
    final_positions: tuple[int, int, int, int]
    trigger_skew_seconds: float
    max_changed_axes: int


@dataclass(frozen=True)
class MultiAxisRoundTripResult:
    start_positions: tuple[int, int, int, int]
    outbound: MultiAxisTargetLegResult
    returned: MultiAxisTargetLegResult
    cleanup: ExtendedCleanupReport


def encode_dint_low_word_first(value: int) -> tuple[int, int]:
    """将有符号32位整数编码为低字在前的两个寄存器。"""
    raw = int(value) & 0xFFFFFFFF
    return raw & 0xFFFF, (raw >> 16) & 0xFFFF


class PlcEngineeringClient(PlcReadonlyClient):
    """带严格写白名单的工程验证客户端。"""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._operation_lock = threading.Lock()

    def _read_dint(self, address: int) -> int:
        return decode_dint_low_word_first(self._read(address, 2))

    def _write_single(self, address: int, value: int) -> None:
        allowed = SINGLE_WRITE_RULES.get(address)
        if allowed is None or value not in allowed:
            raise EngineeringSafetyError(
                f"禁止写入地址{address}或值{value}"
            )
        if self._client is None:
            raise EngineeringSafetyError("PLC尚未连接")

        try:
            try:
                response = self._client.write_register(
                    address,
                    value,
                    device_id=self.device_id,
                )
            except TypeError as exc:
                if "device_id" not in str(exc):
                    raise
                response = self._client.write_register(
                    address,
                    value,
                    slave=self.device_id,
                )
        except Exception as exc:
            raise EngineeringSafetyError(
                f"写入地址{address}失败：{exc}"
            ) from exc

        if hasattr(response, "isError") and response.isError():
            raise EngineeringSafetyError(
                f"写入地址{address}失败：{response}"
            )

    def _write_dint(self, address: int, value: int) -> None:
        if address not in WRITABLE_DINT_ADDRESSES:
            raise EngineeringSafetyError(f"禁止写入DINT地址{address}")
        if self._client is None:
            raise EngineeringSafetyError("PLC尚未连接")

        words = list(encode_dint_low_word_first(value))
        try:
            try:
                response = self._client.write_registers(
                    address,
                    words,
                    device_id=self.device_id,
                )
            except TypeError as exc:
                if "device_id" not in str(exc):
                    raise
                response = self._client.write_registers(
                    address,
                    words,
                    slave=self.device_id,
                )
        except Exception as exc:
            raise EngineeringSafetyError(
                f"写入DINT地址{address}失败：{exc}"
            ) from exc

        if hasattr(response, "isError") and response.isError():
            raise EngineeringSafetyError(
                f"写入DINT地址{address}失败：{response}"
            )

    def _verify_motion_inactive(self) -> None:
        active = []
        for address in MOTION_TRIGGER_ADDRESSES:
            value = self._read(address, 1)[0]
            if value != 0:
                active.append(f"地址{address}={value}")
        if active:
            raise EngineeringSafetyError(
                "存在未清零的运动触发位：" + "，".join(active)
            )

        moving = []
        for axis, address in ACTUAL_SPEED_ADDRESSES.items():
            speed = self._read_dint(address)
            if speed != 0:
                moving.append(f"{axis}轴速度={speed} mm/s")
        if moving:
            raise EngineeringSafetyError(
                "检测到轴仍在运动：" + "，".join(moving)
            )

    def _verify_stationary_preflight(self) -> None:
        system = self._read(MODE_ADDRESS, 3)
        if system[0] != 0:
            raise EngineeringSafetyError(
                f"D5当前为{system[0]}，必须先处于本地模式0"
            )
        if system[2] != 0:
            raise EngineeringSafetyError(
                f"D7当前为{system[2]}，必须先恢复正常值0"
            )
        self._verify_motion_inactive()

    def _prepare_upper_mode(self) -> int:
        """Validate a stationary PLC and enter D5=2 automatically.

        D5=0 is the local/manual mode and D5=2 is the upper-computer mode.
        Motion commands must not require the operator to switch to local mode;
        they only require the gantry to be stopped, alarm-free and not in D7
        software emergency stop.  A local-mode PLC is switched to D5=2 here
        and the value is read back before any target or trigger is written.
        """
        system = self._read(MODE_ADDRESS, 3)
        current_mode = system[0]
        if current_mode not in (0, 2):
            raise EngineeringSafetyError(f"D5当前值异常：{current_mode}")
        if system[2] != 0:
            raise EngineeringSafetyError(f"D7当前为{system[2]}，必须先恢复正常值0")
        self._verify_motion_inactive()
        self._verify_all_axes_alarm_free()
        if current_mode == 0:
            self._write_single(MODE_ADDRESS, 2)
            self._verify_single_readback(MODE_ADDRESS, 2, "D5上位机模式")
        return 2

    def restore_local_mode(self) -> int:
        """在确认无运动后，仅写D5=0并回读验证。"""
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")
        try:
            system = self._read(MODE_ADDRESS, 3)
            current_mode = system[0]
            if current_mode == 0:
                return 0
            if current_mode != 2:
                raise EngineeringSafetyError(
                    f"D5当前值异常：{current_mode}"
                )
            if system[2] != 0:
                raise EngineeringSafetyError(
                    f"D7当前为{system[2]}，必须先为正常值0"
                )

            self._verify_motion_inactive()
            self._write_single(MODE_ADDRESS, 0)
            restored = self._read(MODE_ADDRESS, 1)[0]
            if restored != 0:
                raise EngineeringSafetyError(
                    f"D5写入0后回读{restored}，恢复本地模式失败"
                )
            return restored
        finally:
            self._operation_lock.release()

    def verify_d7_round_trip(
        self,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> D7VerificationResult:
        """设备静止时验证D7=0→1→0，并始终尝试恢复为0。"""
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")
        try:
            self._verify_stationary_preflight()
            before = self._read(ESTOP_ADDRESS, 1)[0]
            attempted = False
            triggered = -1
            restored = -1

            try:
                attempted = True
                self._write_single(ESTOP_ADDRESS, 1)
                triggered = self._read(ESTOP_ADDRESS, 1)[0]
                if triggered != 1:
                    raise EngineeringSafetyError(
                        f"D7写入1后回读{triggered}，验证失败"
                    )
                sleep_fn(1.0)
            finally:
                if attempted:
                    self._write_single(ESTOP_ADDRESS, 0)
                    restored = self._read(ESTOP_ADDRESS, 1)[0]
                    if restored != 0:
                        raise EngineeringSafetyError(
                            f"D7恢复0失败，回读{restored}"
                        )

            return D7VerificationResult(before, triggered, restored)
        finally:
            self._operation_lock.release()

    def _verify_selected_axis_alarm_free(self, axis: str) -> None:
        values = self._read(ALARM_ADDRESSES[axis], 5)
        if values[0] != 0 or values[2] != 0 or values[4] != 0:
            raise EngineeringSafetyError(
                f"{axis}轴存在正限位、负限位或驱动器报警"
            )

    def _verify_all_axes_alarm_free(self) -> None:
        for axis in ALARM_ADDRESSES:
            self._verify_selected_axis_alarm_free(axis)

    def _verify_single_readback(
        self,
        address: int,
        expected: int,
        label: str,
    ) -> None:
        actual = self._read(address, 1)[0]
        if actual != expected:
            raise EngineeringSafetyError(
                f"{label}回读失败：期望{expected}，实际{actual}"
            )

    def _start_self_clearing_trigger(
        self,
        address: int,
        label: str,
    ) -> int:
        self._write_single(address, 10)
        observed = self._read(address, 1)[0]
        if observed not in (0, 10):
            raise EngineeringSafetyError(
                f"{label}首次回读异常："
                f"期望PLC保持10或自动清零0，实际{observed}"
            )
        return observed

    def _pulse_command(
        self,
        address: int,
        label: str,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> tuple[int, int]:
        self._write_single(address, 10)
        self._verify_single_readback(address, 10, label)
        sleep_fn(0.1)
        self._write_single(address, 0)
        self._verify_single_readback(address, 0, label)
        return 10, 0

    def _cleanup_extended_motion(
        self,
        normal: bool,
        parameter_backups: tuple[tuple[int, int], ...] = (),
    ) -> ExtendedCleanupReport:
        """Best-effort cleanup; abnormal cleanup deliberately latches D7."""
        backups = tuple(parameter_backups)
        errors: list[str] = []

        def attempt(label: str, action: Callable[[], None]) -> None:
            try:
                action()
            except Exception as exc:
                errors.append(f"{label}失败：{exc}")

        cleanup_addresses = (
            EXTENDED_TRIGGER_ADDRESSES
            if normal
            else EXTENDED_COMMAND_ADDRESSES
        )
        for address in cleanup_addresses:
            attempt(
                f"清零运动触发D{address}",
                lambda address=address: self._write_single(address, 0),
            )

        if not normal:
            attempt(
                "锁存D7急停",
                lambda: self._write_single(ESTOP_ADDRESS, 1),
            )
        attempt(
            "恢复D5本地模式",
            lambda: self._write_single(MODE_ADDRESS, 0),
        )

        for address, original_value in backups:
            attempt(
                f"恢复DINT参数{address}",
                lambda address=address, original_value=original_value:
                    self._write_dint(address, original_value),
            )

        mode = -1
        estop = -1
        trigger_values = (-1,) * len(EXTENDED_COMMAND_ADDRESSES)
        parameter_values: tuple[tuple[int, int], ...] = ()

        try:
            mode = self._read(MODE_ADDRESS, 1)[0]
            if mode != 0:
                errors.append(f"D5清理回读为{mode}")
        except Exception as exc:
            errors.append(f"读取D5清理结果失败：{exc}")

        expected_estop = 0 if normal else 1
        try:
            estop = self._read(ESTOP_ADDRESS, 1)[0]
            if estop != expected_estop:
                errors.append(
                    f"D7清理回读为{estop}，期望{expected_estop}"
                )
        except Exception as exc:
            errors.append(f"读取D7清理结果失败：{exc}")

        try:
            trigger_values = tuple(
                self._read(address, 1)[0]
                for address in EXTENDED_COMMAND_ADDRESSES
            )
            if any(trigger_values):
                errors.append(f"运动触发清理回读为{trigger_values}")
        except Exception as exc:
            errors.append(f"读取运动触发清理结果失败：{exc}")

        restored_parameters: list[tuple[int, int]] = []
        for address, original_value in backups:
            try:
                actual = self._read_dint(address)
                restored_parameters.append((address, actual))
                if actual != original_value:
                    errors.append(
                        f"DINT参数{address}恢复回读为{actual}，"
                        f"期望{original_value}"
                    )
            except Exception as exc:
                errors.append(f"读取DINT参数{address}清理结果失败：{exc}")
        parameter_values = tuple(restored_parameters)

        return ExtendedCleanupReport(
            mode=mode,
            estop=estop,
            trigger_values=trigger_values,
            parameter_values=parameter_values,
            latched=estop == 1,
            errors=tuple(errors),
        )

    def recover_stationary_estop(self) -> int:
        """Only release a latched D7 after proving the PLC is stationary."""
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")
        try:
            system = self._read(MODE_ADDRESS, 3)
            if system[0] != 0:
                raise EngineeringSafetyError("D5必须为本地模式0")
            if system[2] != 1:
                raise EngineeringSafetyError("D7必须为锁存值1")
            self._verify_motion_inactive()
            self._write_single(ESTOP_ADDRESS, 0)
            self._verify_single_readback(
                ESTOP_ADDRESS,
                0,
                "D7静止恢复",
            )
            return 0
        finally:
            self._operation_lock.release()

    def run_reset_test(
        self,
        axis: str,
        d7_verified: bool,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[str, int, int], None] | None = None,
    ) -> ResetResult:
        """Verify the reset signal path while requiring zero axis motion."""
        if axis not in RESET_ADDRESSES:
            raise ValueError("轴只允许X/Y/Z/R")
        if not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")

        try:
            self._verify_stationary_preflight()
            self._verify_all_axes_alarm_free()

            position_address = POSITION_ADDRESSES[axis]
            speed_address = ACTUAL_SPEED_ADDRESSES[axis]
            start_position = self._read_dint(position_address)
            end_position = start_position
            position_samples: list[int] = []
            speed_samples: list[int] = []
            command_values = (-1, -1)
            failure: Exception | None = None

            try:
                self._write_single(MODE_ADDRESS, 2)
                self._verify_single_readback(
                    MODE_ADDRESS,
                    2,
                    "D5上位机模式",
                )
                command_values = self._pulse_command(
                    RESET_ADDRESSES[axis],
                    f"{axis}轴复位命令",
                    sleep_fn,
                )

                for _ in range(10):
                    sleep_fn(0.1)
                    end_position = self._read_dint(position_address)
                    position_samples.append(end_position)
                    actual_speed = self._read_dint(speed_address)
                    speed_samples.append(actual_speed)
                    self._verify_all_axes_alarm_free()
                    if sample_callback is not None:
                        sample_callback(axis, end_position, actual_speed)

                if (
                    any(
                        position != start_position
                        for position in position_samples
                    )
                    or any(speed_samples)
                ):
                    raise EngineeringSafetyError(
                        "复位信号路径检测到位置或速度变化"
                    )
            except Exception as exc:
                failure = exc

            if failure is not None:
                cleanup = self._cleanup_extended_motion(normal=False)
                cleanup_detail = "；".join(cleanup.errors)
                suffix = f"；清理异常：{cleanup_detail}" if cleanup_detail else ""
                raise EngineeringSafetyError(
                    f"复位信号路径异常并已执行锁存清理：{failure}{suffix}"
                ) from failure

            cleanup = self._cleanup_extended_motion(normal=True)
            if cleanup.errors:
                latched_cleanup = self._cleanup_extended_motion(normal=False)
                details = "；".join(
                    (*cleanup.errors, *latched_cleanup.errors)
                )
                raise EngineeringSafetyError(
                    f"复位信号路径清理异常并已锁存D7：{details}"
                )

            return ResetResult(
                axis=axis,
                start_position=start_position,
                end_position=end_position,
                speed_samples=tuple(speed_samples),
                command_values=command_values,
                cleanup=cleanup,
            )
        finally:
            self._operation_lock.release()

    def run_home_test(
        self,
        axis: str,
        d7_verified: bool,
        timeout: float = 60.0,
        poll_interval: float = 0.1,
        clock: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[str, int, int, int], None] | None = None,
    ) -> HomeResult:
        """Run one selected-axis home cycle and verify its live feedback."""
        if axis not in HOME_ADDRESSES:
            raise ValueError("轴只允许X/Y/Z/R")
        if not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        if float(timeout) <= 0:
            raise ValueError("回零超时时间必须大于0秒")
        if float(poll_interval) <= 0:
            raise ValueError("回零轮询间隔必须大于0秒")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")

        try:
            self._verify_stationary_preflight()
            self._verify_all_axes_alarm_free()

            position_address = POSITION_ADDRESSES[axis]
            speed_address = ACTUAL_SPEED_ADDRESSES[axis]
            done_address = HOME_DONE_ADDRESSES[axis]
            start_position = self._read_dint(position_address)
            if abs(start_position) < 2:
                raise EngineeringSafetyError(
                    f"{axis}轴回零前必须距离零点至少2 mm，"
                    f"当前位置为{start_position} mm"
                )

            end_position = start_position
            speed_samples: list[int] = []
            done_samples: list[int] = []
            failure: Exception | None = None

            try:
                self._write_single(MODE_ADDRESS, 2)
                self._verify_single_readback(
                    MODE_ADDRESS,
                    2,
                    "D5上位机模式",
                )
                label = f"{axis}轴回零命令"
                observed = self._start_self_clearing_trigger(
                    HOME_ADDRESSES[axis],
                    label,
                )
                sleep_fn(0.1)
                if observed == 10:
                    self._write_single(HOME_ADDRESSES[axis], 0)
                self._verify_single_readback(
                    HOME_ADDRESSES[axis],
                    0,
                    label,
                )

                deadline = clock() + float(timeout)
                done_seen = False
                fallback_stationary_since: float | None = None
                motion_seen = False
                while True:
                    remaining = deadline - clock()
                    if remaining <= 0:
                        raise EngineeringSafetyError(
                            f"{axis}轴回零超时，未同时满足完成信号和速度归零"
                        )
                    sleep_fn(min(float(poll_interval), remaining))
                    if clock() > deadline:
                        raise EngineeringSafetyError(
                            f"{axis}轴回零超时，未同时满足完成信号和速度归零"
                        )

                    end_position = self._read_dint(position_address)
                    actual_speed = self._read_dint(speed_address)
                    done = self._read(done_address, 1)[0]
                    self._verify_selected_axis_alarm_free(axis)
                    if clock() > deadline:
                        raise EngineeringSafetyError(
                            f"{axis}轴回零超时，未同时满足完成信号和速度归零"
                        )

                    speed_samples.append(actual_speed)
                    done_samples.append(done)
                    if sample_callback is not None:
                        sample_callback(
                            axis,
                            end_position,
                            actual_speed,
                            done,
                        )

                    done_seen = done_seen or done == 1
                    motion_seen = (
                        motion_seen
                        or end_position != start_position
                        or actual_speed != 0
                    )
                    if clock() > deadline:
                        raise EngineeringSafetyError(
                            f"{axis}轴回零超时，未同时满足完成信号和速度归零"
                        )
                    if done_seen and actual_speed == 0:
                        break
                    fallback_ready = (
                        motion_seen
                        and actual_speed == 0
                        and -1 <= end_position <= 1
                    )
                    if fallback_ready:
                        if fallback_stationary_since is None:
                            fallback_stationary_since = clock()
                        elif (
                            clock() - fallback_stationary_since
                            >= HOME_STATIONARY_CONFIRM_SECONDS
                        ):
                            break
                    else:
                        fallback_stationary_since = None
                    if clock() >= deadline:
                        raise EngineeringSafetyError(
                            f"{axis}轴回零超时，未同时满足完成信号和速度归零"
                        )

                observed_motion = (
                    end_position != start_position
                    or any(speed != 0 for speed in speed_samples)
                )
                if not observed_motion:
                    raise EngineeringSafetyError(
                        f"{axis}轴回零未检测到位置变化或非零速度"
                    )
                if speed_samples[-1] != 0:
                    raise EngineeringSafetyError(
                        f"{axis}轴回零最终速度不是0 mm/s："
                        f"{speed_samples[-1]} mm/s"
                    )
                if not -1 <= end_position <= 1:
                    raise EngineeringSafetyError(
                        f"{axis}轴回零最终位置超出±1 mm："
                        f"{end_position} mm"
                    )
            except Exception as exc:
                failure = exc

            if failure is not None:
                cleanup = self._cleanup_extended_motion(normal=False)
                cleanup_detail = "；".join(cleanup.errors)
                suffix = (
                    f"；清理异常：{cleanup_detail}"
                    if cleanup_detail
                    else ""
                )
                raise EngineeringSafetyError(
                    f"回零测试异常并已执行锁存清理：{failure}{suffix}"
                ) from failure

            cleanup = self._cleanup_extended_motion(normal=True)
            if cleanup.errors:
                latched_cleanup = self._cleanup_extended_motion(normal=False)
                details = "；".join(
                    (*cleanup.errors, *latched_cleanup.errors)
                )
                raise EngineeringSafetyError(
                    f"回零测试清理异常并已锁存D7：{details}"
                )

            return HomeResult(
                axis=axis,
                start_position=start_position,
                end_position=end_position,
                speed_samples=tuple(speed_samples),
                done_samples=tuple(done_samples),
                cleanup=cleanup,
            )
        finally:
            self._operation_lock.release()

    def _run_absolute_leg(
        self,
        axis: str,
        target: int,
        speed: int,
        timeout: float,
        poll_interval: float,
        clock: Callable[[], float],
        sleep_fn: Callable[[float], None],
        sample_callback: Callable[[str, int, int, int], None] | None,
        allowed_targets: frozenset[int] | None = None,
        require_nonzero_speed: bool = True,
    ) -> AbsoluteLegResult:
        """Run one absolute leg while the caller holds the operation lock.

        默认仍服务于0~10 mm工程绝对定位验证。相对移动通过绝对定位
        实现时，可由调用者显式授权本次的两个目标（当前位置与当前位置+位移），
        避免把工程测试区间误当成设备的绝对坐标范围。
        """
        if axis not in ABS_TRIGGER_ADDRESSES:
            raise ValueError("轴只允许X/Y/Z/R")
        target_value = int(target)
        if allowed_targets is None:
            if not 0 <= target_value <= 10:
                raise ValueError("绝对定位分段目标只允许0到10 mm")
        elif target_value not in allowed_targets:
            raise ValueError(
                f"绝对定位目标{target_value} mm不在本次相对移动授权目标中"
            )
        if not 1 <= int(speed) <= 5:
            raise ValueError("绝对定位速度只允许1到5 mm/s")
        if float(timeout) <= 0:
            raise ValueError("绝对定位超时时间必须大于0秒")
        if float(poll_interval) <= 0:
            raise ValueError("绝对定位轮询间隔必须大于0秒")

        target_address = ABS_TARGET_ADDRESSES[axis]
        speed_address = ABS_SPEED_ADDRESSES[axis]
        trigger_address = ABS_TRIGGER_ADDRESSES[axis]
        position_address = POSITION_ADDRESSES[axis]
        actual_speed_address = ACTUAL_SPEED_ADDRESSES[axis]
        done_address = ABS_DONE_ADDRESSES[axis]

        start_position = self._read_dint(position_address)
        end_position = start_position
        position_samples: list[int] = []
        speed_samples: list[int] = []
        done_samples: list[int] = []

        self._write_dint(target_address, int(target))
        target_readback = self._read_dint(target_address)
        if target_readback != int(target):
            raise EngineeringSafetyError(
                f"{axis}轴绝对定位目标参数回读失败："
                f"期望{int(target)}，实际{target_readback}"
            )

        self._write_dint(speed_address, int(speed))
        speed_readback = self._read_dint(speed_address)
        if speed_readback != int(speed):
            raise EngineeringSafetyError(
                f"{axis}轴绝对定位速度参数回读失败："
                f"期望{int(speed)}，实际{speed_readback}"
            )

        done_seen = False
        label = f"{axis}轴绝对定位命令"
        observed = self._start_self_clearing_trigger(
            trigger_address,
            label,
        )
        try:
            sleep_fn(0.1)
            end_position = self._read_dint(position_address)
            actual_speed = self._read_dint(actual_speed_address)
            done = self._read(done_address, 1)[0]
            self._verify_selected_axis_alarm_free(axis)
            position_samples.append(end_position)
            speed_samples.append(actual_speed)
            done_samples.append(done)
            done_seen = done == 1
            if sample_callback is not None:
                sample_callback(axis, end_position, actual_speed, done)
        finally:
            if observed == 10:
                self._write_single(trigger_address, 0)
            self._verify_single_readback(
                trigger_address,
                0,
                label,
            )

        deadline = clock() + float(timeout)
        while True:
            remaining = deadline - clock()
            if remaining <= 0:
                raise EngineeringSafetyError(
                    f"{axis}轴绝对定位超时，未同时满足完成信号和速度归零"
                )
            sleep_fn(min(float(poll_interval), remaining))
            if clock() > deadline:
                raise EngineeringSafetyError(
                    f"{axis}轴绝对定位超时，未同时满足完成信号和速度归零"
                )

            end_position = self._read_dint(position_address)
            actual_speed = self._read_dint(actual_speed_address)
            done = self._read(done_address, 1)[0]
            self._verify_selected_axis_alarm_free(axis)
            if clock() > deadline:
                raise EngineeringSafetyError(
                    f"{axis}轴绝对定位超时，未同时满足完成信号和速度归零"
                )

            speed_samples.append(actual_speed)
            done_samples.append(done)
            position_samples.append(end_position)
            if sample_callback is not None:
                sample_callback(axis, end_position, actual_speed, done)

            done_seen = done_seen or done == 1
            if clock() > deadline:
                raise EngineeringSafetyError(
                    f"{axis}轴绝对定位超时，未同时满足完成信号和速度归零"
                )
            if done_seen and actual_speed == 0:
                break
            if clock() >= deadline:
                raise EngineeringSafetyError(
                    f"{axis}轴绝对定位超时，未同时满足完成信号和速度归零"
                )

        if not any(
            position != start_position for position in position_samples
        ):
            raise EngineeringSafetyError(
                f"{axis}轴绝对定位未检测到位置变化"
            )
        if (
            require_nonzero_speed
            and not any(actual_speed != 0 for actual_speed in speed_samples)
        ):
            raise EngineeringSafetyError(
                f"{axis}轴绝对定位未检测到非零速度"
            )
        if not done_seen:
            raise EngineeringSafetyError(
                f"{axis}轴绝对定位完成信号不是1"
            )
        if speed_samples[-1] != 0:
            raise EngineeringSafetyError(
                f"{axis}轴绝对定位最终速度不是0 mm/s："
                f"{speed_samples[-1]} mm/s"
            )
        if not int(target) - 1 <= end_position <= int(target) + 1:
            raise EngineeringSafetyError(
                f"{axis}轴绝对定位最终位置超出目标±1 mm："
                f"目标{int(target)} mm，实际{end_position} mm"
            )

        return AbsoluteLegResult(
            axis=axis,
            target=int(target),
            start_position=start_position,
            end_position=end_position,
            speed_samples=tuple(speed_samples),
            done_samples=tuple(done_samples),
        )

    def run_absolute_test(
        self,
        axis: str,
        d7_verified: bool,
        speed: int = 2,
        target: int = 10,
        timeout: float = 20.0,
        poll_interval: float = 0.1,
        clock: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[str, int, int, int], None] | None = None,
    ) -> AbsoluteResult:
        """Run a selected-axis absolute 0-to-target-to-0 verification."""
        if axis not in ABS_TRIGGER_ADDRESSES:
            raise ValueError("轴只允许X/Y/Z/R")
        if not isinstance(d7_verified, bool) or not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        if (
            isinstance(speed, bool)
            or not isinstance(speed, (int, float))
            or not float(speed).is_integer()
            or not 1 <= int(speed) <= 5
        ):
            raise ValueError("绝对定位速度只允许1到5 mm/s")
        if (
            isinstance(target, bool)
            or not isinstance(target, (int, float))
            or not float(target).is_integer()
            or not 1 <= int(target) <= 10
        ):
            raise ValueError("绝对定位目标只允许1到10 mm")
        if (
            isinstance(timeout, bool)
            or not isinstance(timeout, (int, float))
            or float(timeout) <= 0
        ):
            raise ValueError("绝对定位超时时间必须大于0秒")
        if (
            isinstance(poll_interval, bool)
            or not isinstance(poll_interval, (int, float))
            or float(poll_interval) <= 0
        ):
            raise ValueError("绝对定位轮询间隔必须大于0秒")
        if not callable(clock):
            raise ValueError("绝对定位时钟必须可调用")
        if not callable(sleep_fn):
            raise ValueError("绝对定位等待函数必须可调用")
        if sample_callback is not None and not callable(sample_callback):
            raise ValueError("绝对定位采样回调必须可调用")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")

        try:
            self._verify_stationary_preflight()
            self._verify_all_axes_alarm_free()

            start_position = self._read_dint(POSITION_ADDRESSES[axis])
            if not -1 <= start_position <= 1:
                raise EngineeringSafetyError(
                    f"{axis}轴绝对定位前必须位于0±1 mm，"
                    f"当前位置为{start_position} mm"
                )

            target_address = ABS_TARGET_ADDRESSES[axis]
            speed_address = ABS_SPEED_ADDRESSES[axis]
            parameter_backups = (
                (target_address, self._read_dint(target_address)),
                (speed_address, self._read_dint(speed_address)),
            )
            failure: Exception | None = None
            outbound: AbsoluteLegResult | None = None
            returned: AbsoluteLegResult | None = None

            try:
                self._write_single(MODE_ADDRESS, 2)
                self._verify_single_readback(
                    MODE_ADDRESS,
                    2,
                    "D5上位机模式",
                )
                outbound = self._run_absolute_leg(
                    axis,
                    int(target),
                    int(speed),
                    float(timeout),
                    float(poll_interval),
                    clock,
                    sleep_fn,
                    sample_callback,
                )
                returned = self._run_absolute_leg(
                    axis,
                    0,
                    int(speed),
                    float(timeout),
                    float(poll_interval),
                    clock,
                    sleep_fn,
                    sample_callback,
                )
            except Exception as exc:
                failure = exc

            if failure is not None:
                cleanup = self._cleanup_extended_motion(
                    normal=False,
                    parameter_backups=parameter_backups,
                )
                cleanup_detail = "；".join(cleanup.errors)
                suffix = (
                    f"；清理异常：{cleanup_detail}"
                    if cleanup_detail
                    else ""
                )
                raise EngineeringSafetyError(
                    f"绝对定位测试异常并已执行锁存清理：{failure}{suffix}"
                ) from failure

            cleanup = self._cleanup_extended_motion(
                normal=True,
                parameter_backups=parameter_backups,
            )
            if cleanup.errors:
                latched_cleanup = self._cleanup_extended_motion(
                    normal=False,
                    parameter_backups=parameter_backups,
                )
                details = "；".join(
                    (*cleanup.errors, *latched_cleanup.errors)
                )
                raise EngineeringSafetyError(
                    f"绝对定位测试参数恢复或清理异常并已锁存D7：{details}"
                )

            if outbound is None or returned is None:
                raise EngineeringSafetyError("绝对定位测试结果不完整")
            return AbsoluteResult(
                axis=axis,
                outbound=outbound,
                returned=returned,
                cleanup=cleanup,
            )
        finally:
            self._operation_lock.release()

    def run_relative_test(
        self,
        axis: str,
        d7_verified: bool,
        distance: int = RELATIVE_TEST_DISTANCE,
        speed: int = 1,
        timeout: float = 30.0,
        poll_interval: float = 0.1,
        clock: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[str, int, int, int], None] | None = None,
    ) -> RelativeResult:
        """用已验证的绝对定位链路实现“当前位置+位移，再返回”。

        当前PLC的原生相对定位寄存器虽然会改变内部状态，但现场没有带动
        机械轴。因此本测试不再写D16/HD110/HD112，而是把相对位移换算为
        绝对目标：目标=当前坐标+位移，再调用已经真机验证通过的绝对定位。
        """
        if axis not in ABS_TRIGGER_ADDRESSES:
            raise ValueError("轴只允许X/Y/Z/R")
        if not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        if type(distance) is not int or not 1 <= distance <= RELATIVE_TEST_DISTANCE:
            unit = "°" if axis == "R" else "mm"
            raise ValueError(
                f"相对移动距离只允许整数1到{RELATIVE_TEST_DISTANCE} {unit}"
            )
        if type(speed) is not int or not 1 <= speed <= 2:
            raise ValueError("相对定位速度只允许整数1到2 mm/s")
        if float(timeout) <= 0:
            raise ValueError("相对定位超时时间必须大于0秒")
        if float(poll_interval) <= 0:
            raise ValueError("相对定位轮询间隔必须大于0秒")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")

        position_address = POSITION_ADDRESSES[axis]
        target_address = ABS_TARGET_ADDRESSES[axis]
        speed_address = ABS_SPEED_ADDRESSES[axis]
        backups: tuple[tuple[int, int], ...] = ()

        try:
            self._verify_stationary_preflight()
            self._verify_all_axes_alarm_free()

            start_position = self._read_dint(position_address)
            outbound_target = start_position + distance

            # 相对移动可从任意已知当前位置开始。工程验证仍只允许
            # 1~10的小位移，并在执行前检查静止、报警和限位状态。
            # 具体正方向剩余空间由操作者在确认窗口中人工确认。

            backups = (
                (target_address, self._read_dint(target_address)),
                (speed_address, self._read_dint(speed_address)),
            )

            self._write_single(MODE_ADDRESS, 2)
            self._verify_single_readback(MODE_ADDRESS, 2, "D5上位机模式")

            authorized_targets = frozenset((start_position, outbound_target))
            outbound = self._run_absolute_leg(
                axis,
                outbound_target,
                speed,
                timeout,
                poll_interval,
                clock,
                sleep_fn,
                sample_callback,
                allowed_targets=authorized_targets,
                require_nonzero_speed=False,
            )
            returned = self._run_absolute_leg(
                axis,
                start_position,
                speed,
                timeout,
                poll_interval,
                clock,
                sleep_fn,
                sample_callback,
                allowed_targets=authorized_targets,
                require_nonzero_speed=False,
            )

        except Exception as exc:
            cleanup = self._cleanup_extended_motion(
                normal=False,
                parameter_backups=backups,
            )
            cleanup_detail = "；".join(cleanup.errors)
            suffix = f"；清理异常：{cleanup_detail}" if cleanup_detail else ""
            raise EngineeringSafetyError(
                "相对移动（绝对定位链路实现）异常并已执行锁存清理："
                f"{exc}{suffix}"
            ) from exc
        else:
            cleanup = self._cleanup_extended_motion(
                normal=True,
                parameter_backups=backups,
            )
            if cleanup.errors:
                latched_cleanup = self._cleanup_extended_motion(
                    normal=False,
                    parameter_backups=backups,
                )
                details = "；".join((*cleanup.errors, *latched_cleanup.errors))
                raise EngineeringSafetyError(
                    "相对移动清理异常并已锁存D7：" + details
                )

            return RelativeResult(
                axis=axis,
                start_position=start_position,
                outbound_position=outbound.end_position,
                returned_position=returned.end_position,
                speed_samples=tuple(
                    (*outbound.speed_samples, *returned.speed_samples)
                ),
                done_samples=tuple(
                    (*outbound.done_samples, *returned.done_samples)
                ),
                cleanup=cleanup,
            )
        finally:
            self._operation_lock.release()

    def run_pause_test(
        self,
        axis: str,
        d7_verified: bool,
        clock: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[str, int, int], None] | None = None,
    ) -> PauseResult:
        """用已验证点动链路启动轴，再按位置变化验证暂停与恢复。"""
        if axis not in PAUSE_ADDRESSES:
            raise ValueError("轴只允许X/Y/Z/R")
        if not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")

        jog_address = JOG_ADDRESSES[axis]
        pause_address = PAUSE_ADDRESSES[axis]
        speed_parameter = JOG_SPEED_ADDRESSES[axis]
        position_address = POSITION_ADDRESSES[axis]
        actual_speed_address = ACTUAL_SPEED_ADDRESSES[axis]
        backups: tuple[tuple[int, int], ...] = ()
        test_speed = 5
        movement_threshold = 2
        max_test_displacement = 10
        try:
            self._verify_stationary_preflight()
            self._verify_all_axes_alarm_free()
            if self._read(pause_address, 1)[0] != 0:
                raise EngineeringSafetyError(f"{axis}轴暂停位必须先为0")

            start_position = self._read_dint(position_address)
            backups = ((speed_parameter, self._read_dint(speed_parameter)),)

            # 使用已经验证过的低速点动链路，速度提高到5，便于肉眼观察。
            self._write_dint(speed_parameter, test_speed)
            if self._read_dint(speed_parameter) != test_speed:
                raise EngineeringSafetyError("点动速度回读失败")
            self._write_single(MODE_ADDRESS, 2)
            self._verify_single_readback(MODE_ADDRESS, 2, "D5上位机模式")
            self._write_single(jog_address, 10)
            self._verify_single_readback(jog_address, 10, f"{axis}轴正向点动")

            # 速度反馈在低速下可能始终为0，所以按位置至少变化2 mm判断真实启动。
            moving_position = start_position
            for _ in range(50):
                sleep_fn(0.1)
                moving_position = self._read_dint(position_address)
                moving_speed = self._read_dint(actual_speed_address)
                self._verify_all_axes_alarm_free()
                if sample_callback is not None:
                    sample_callback(axis, moving_position, moving_speed)
                if abs(moving_position - start_position) > max_test_displacement:
                    raise EngineeringSafetyError("暂停测试启动阶段位移超过10 mm")
                if abs(moving_position - start_position) >= movement_threshold:
                    break
            else:
                raise EngineeringSafetyError("暂停测试开始前未检测到位置变化")

            # 写入暂停并留出短暂减速时间，再以位置稳定性验证暂停。
            self._write_single(pause_address, 1)
            self._verify_single_readback(pause_address, 1, f"{axis}轴暂停")
            sleep_fn(0.3)
            paused_position = self._read_dint(position_address)

            for _ in range(20):
                sleep_fn(0.1)
                position = self._read_dint(position_address)
                actual_speed = self._read_dint(actual_speed_address)
                self._verify_all_axes_alarm_free()
                if sample_callback is not None:
                    sample_callback(axis, position, actual_speed)
                if abs(position - paused_position) > 1:
                    raise EngineeringSafetyError("暂停保持期间轴仍在移动")

            # 现场真机验证表明：仅把暂停位从1清为0不会自动恢复点动。
            # 暂停动作会结束本次点动状态，因此解除暂停后需要把原点动命令
            # 重新形成一次0→10触发，才能继续沿原方向运动。
            self._write_single(pause_address, 0)
            self._verify_single_readback(pause_address, 0, f"{axis}轴解除暂停")
            self._write_single(jog_address, 0)
            self._verify_single_readback(jog_address, 0, f"{axis}轴恢复前清除点动")
            sleep_fn(0.1)
            self._write_single(jog_address, 10)
            self._verify_single_readback(jog_address, 10, f"{axis}轴恢复点动")
            resumed_position = paused_position
            for _ in range(50):
                sleep_fn(0.1)
                resumed_position = self._read_dint(position_address)
                resumed_speed = self._read_dint(actual_speed_address)
                self._verify_all_axes_alarm_free()
                if sample_callback is not None:
                    sample_callback(axis, resumed_position, resumed_speed)
                if abs(resumed_position - start_position) > max_test_displacement:
                    raise EngineeringSafetyError("暂停测试恢复阶段总位移超过10 mm")
                if abs(resumed_position - paused_position) >= movement_threshold:
                    break
            else:
                raise EngineeringSafetyError("解除暂停后未检测到位置恢复变化")

            resumed = True
        except Exception as exc:
            cleanup = self._cleanup_extended_motion(
                normal=False,
                parameter_backups=backups,
            )
            raise EngineeringSafetyError(
                f"暂停测试异常并已执行锁存清理：{exc}"
            ) from exc
        else:
            cleanup = self._cleanup_extended_motion(
                normal=True,
                parameter_backups=backups,
            )
            if cleanup.errors:
                self._cleanup_extended_motion(
                    normal=False,
                    parameter_backups=backups,
                )
                raise EngineeringSafetyError("暂停测试清理异常并已锁存D7")
            return PauseResult(
                axis=axis,
                start_position=start_position,
                paused_position=paused_position,
                resumed_position=resumed_position,
                resumed=resumed,
                cleanup=cleanup,
            )
        finally:
            self._operation_lock.release()

    def run_dynamic_d7_test(
        self,
        d7_verified: bool,
        clock: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[str, int, int], None] | None = None,
    ) -> DynamicD7Result:
        """Verify D7 stops one low-speed X jog and deliberately stays latched."""
        axis = "X"
        if not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")
        speed_parameter = JOG_SPEED_ADDRESSES[axis]
        position_address = POSITION_ADDRESSES[axis]
        actual_speed_address = ACTUAL_SPEED_ADDRESSES[axis]
        backups: tuple[tuple[int, int], ...] = ()
        d7_triggered = False
        movement_threshold = 10
        max_start_displacement = 15
        stop_drift_limit = 2
        try:
            self._verify_stationary_preflight()
            self._verify_all_axes_alarm_free()
            start_position = self._read_dint(position_address)
            backups = ((speed_parameter, self._read_dint(speed_parameter)),)
            self._write_dint(speed_parameter, 5)
            self._write_single(MODE_ADDRESS, 2)
            self._verify_single_readback(MODE_ADDRESS, 2, "D5上位机模式")
            self._write_single(JOG_ADDRESSES[axis], 10)
            self._verify_single_readback(JOG_ADDRESSES[axis], 10, "X轴正向点动")

            # 低速测试时速度反馈可能一直为0，不能把速度非零作为
            # “已经运动”的硬条件。这里只按真实位置变化确认X轴已启动。
            moving_position = start_position
            for _ in range(50):
                sleep_fn(0.1)
                moving_position = self._read_dint(position_address)
                moving_speed = self._read_dint(actual_speed_address)
                self._verify_all_axes_alarm_free()
                if sample_callback is not None:
                    sample_callback(axis, moving_position, moving_speed)
                if abs(moving_position - start_position) > max_start_displacement:
                    raise EngineeringSafetyError("D7动态测试触发前位移超过15 mm")
                if abs(moving_position - start_position) >= movement_threshold:
                    break
            else:
                raise EngineeringSafetyError("D7动态测试开始前未检测到真实运动")

            self._write_single(ESTOP_ADDRESS, 1)
            d7_triggered = True
            self._verify_single_readback(ESTOP_ADDRESS, 1, "运动中D7急停")
            sleep_fn(0.2)
            stopped_position = self._read_dint(position_address)
            final_speed = self._read_dint(actual_speed_address)
            self._verify_all_axes_alarm_free()
            if sample_callback is not None:
                sample_callback(axis, stopped_position, final_speed)

            # D7触发后的制动效果按位置稳定性确认；速度反馈仅记录。
            for _ in range(10):
                sleep_fn(0.1)
                position = self._read_dint(position_address)
                final_speed = self._read_dint(actual_speed_address)
                self._verify_all_axes_alarm_free()
                if sample_callback is not None:
                    sample_callback(axis, position, final_speed)
                if abs(position - stopped_position) > stop_drift_limit:
                    raise EngineeringSafetyError("D7写入1后X轴仍在继续移动")
        except Exception as exc:
            cleanup = self._cleanup_extended_motion(
                normal=False,
                parameter_backups=backups,
            )
            raise EngineeringSafetyError(
                f"动态D7测试异常并已执行锁存清理：{exc}"
            ) from exc
        else:
            cleanup = self._cleanup_extended_motion(
                normal=False,
                parameter_backups=backups,
            )
            if cleanup.errors or not cleanup.latched or not d7_triggered:
                raise EngineeringSafetyError("动态D7测试清理或锁存验证失败")
            return DynamicD7Result(
                axis=axis,
                start_position=start_position,
                stopped_position=stopped_position,
                final_speed=final_speed,
                cleanup=cleanup,
            )
        finally:
            self._operation_lock.release()

    def _run_multi_axis_targets_leg(
        self,
        targets: tuple[int, int, int, int],
        speed: int,
        timeout: float,
        poll_interval: float,
        clock: Callable[[], float],
        sleep_fn: Callable[[float], None],
        sample_callback: Callable[[MotionSample], None] | None,
    ) -> MultiAxisTargetLegResult:
        """Run one near-simultaneous absolute leg to four independent targets.

        The caller owns the operation lock, mode switch, parameter backup, and cleanup.
        Actual-speed feedback is recorded but is not required to become nonzero because
        the low-speed feedback has been observed to remain zero during real movement.
        """
        axes = tuple(ABS_TRIGGER_ADDRESSES)
        if len(targets) != len(axes) or any(type(value) is not int for value in targets):
            raise ValueError("四轴目标必须是四个整数")

        leg_start_positions = tuple(
            self._read_dint(POSITION_ADDRESSES[axis])
            for axis in axes
        )
        done_seen = {axis: False for axis in axes}
        samples: list[MotionSample] = []
        trigger_times: list[float] = []
        max_changed_axes = 0

        for axis, target in zip(axes, targets):
            target_address = ABS_TARGET_ADDRESSES[axis]
            speed_address = ABS_SPEED_ADDRESSES[axis]
            self._write_dint(target_address, target)
            actual_target = self._read_dint(target_address)
            if actual_target != target:
                raise EngineeringSafetyError(
                    f"{axis}轴四轴定位目标参数回读失败：期望{target}，实际{actual_target}"
                )
            self._write_dint(speed_address, speed)
            actual_speed_parameter = self._read_dint(speed_address)
            if actual_speed_parameter != speed:
                raise EngineeringSafetyError(
                    f"{axis}轴四轴定位速度参数回读失败："
                    f"期望{speed}，实际{actual_speed_parameter}"
                )

        observed: dict[str, int] = {}
        for axis in axes:
            trigger_times.append(clock())
            observed[axis] = self._start_self_clearing_trigger(
                ABS_TRIGGER_ADDRESSES[axis],
                f"{axis}轴四轴绝对定位命令",
            )

        for axis in axes:
            if observed[axis] == 10:
                self._write_single(ABS_TRIGGER_ADDRESSES[axis], 0)
        for axis in axes:
            self._verify_single_readback(
                ABS_TRIGGER_ADDRESSES[axis],
                0,
                f"{axis}轴四轴绝对定位命令",
            )

        started_at = trigger_times[0]
        deadline = clock() + float(timeout)
        while True:
            remaining = deadline - clock()
            if remaining <= 0:
                raise EngineeringSafetyError(
                    "四轴近同时定位超时，未全部到达目标"
                )
            sleep_fn(min(float(poll_interval), remaining))
            positions = tuple(
                self._read_dint(POSITION_ADDRESSES[axis])
                for axis in axes
            )
            speeds = tuple(
                self._read_dint(ACTUAL_SPEED_ADDRESSES[axis])
                for axis in axes
            )
            dones = tuple(
                self._read(ABS_DONE_ADDRESSES[axis], 1)[0]
                for axis in axes
            )
            self._verify_all_axes_alarm_free()
            for axis, done in zip(axes, dones):
                done_seen[axis] = done_seen[axis] or done == 1

            sample = MotionSample(
                elapsed=max(0.0, clock() - started_at),
                positions=positions,
                speeds=speeds,
            )
            samples.append(sample)
            if sample_callback is not None:
                sample_callback(sample)

            changed_axes = sum(
                position != start
                for position, start in zip(positions, leg_start_positions)
            )
            max_changed_axes = max(max_changed_axes, changed_axes)
            reached = all(
                target - 1 <= position <= target + 1
                for target, position in zip(targets, positions)
            )
            if reached and all(done_seen.values()) and not any(speeds):
                break

        final_positions = samples[-1].positions
        for index, axis in enumerate(axes):
            if not any(
                sample.positions[index] != leg_start_positions[index]
                for sample in samples
            ):
                raise EngineeringSafetyError(
                    f"{axis}轴四轴近同时定位未检测到位置变化"
                )
            if not done_seen[axis]:
                raise EngineeringSafetyError(
                    f"{axis}轴四轴近同时定位完成信号不是1"
                )
            target = targets[index]
            final_position = final_positions[index]
            if not target - 1 <= final_position <= target + 1:
                raise EngineeringSafetyError(
                    f"{axis}轴四轴近同时定位最终位置超出目标±1："
                    f"目标{target}，实际{final_position}"
                )

        trigger_skew = (
            max(trigger_times) - min(trigger_times)
            if trigger_times
            else 0.0
        )
        return MultiAxisTargetLegResult(
            targets=targets,
            final_positions=final_positions,
            trigger_skew_seconds=trigger_skew,
            max_changed_axes=max_changed_axes,
        )

    def run_multi_axis_round_trip(
        self,
        d7_verified: bool,
        distance: int = 20,
        speed: int = 5,
        dwell_seconds: float = 1.0,
        timeout: float = 30.0,
        poll_interval: float = 0.1,
        clock: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[MotionSample], None] | None = None,
    ) -> MultiAxisRoundTripResult:
        """Move X/Y/Z by +20 mm and R by +20 degrees, then return."""
        if type(distance) is not int or distance != 20:
            raise ValueError("四轴近同时往返测试距离固定为20 mm/20°")
        if type(speed) is not int or speed != 5:
            raise ValueError("四轴近同时往返测试速度固定为5 mm/s或5°/s")
        if not isinstance(d7_verified, bool) or not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        for label, value in (
            ("停留时间", dwell_seconds),
            ("超时时间", timeout),
            ("轮询间隔", poll_interval),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) <= 0
            ):
                raise ValueError(f"四轴近同时往返{label}必须为有限正数")
        if not callable(clock) or not callable(sleep_fn):
            raise ValueError("四轴近同时往返时钟和等待函数必须可调用")
        if sample_callback is not None and not callable(sample_callback):
            raise ValueError("四轴近同时往返采样回调必须可调用")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")

        axes = tuple(ABS_TRIGGER_ADDRESSES)
        parameter_backups: tuple[tuple[int, int], ...] = ()
        try:
            self._verify_stationary_preflight()
            self._verify_all_axes_alarm_free()
            start_positions = tuple(
                self._read_dint(POSITION_ADDRESSES[axis])
                for axis in axes
            )
            outbound_targets = tuple(
                position + distance for position in start_positions
            )
            parameter_backups = tuple(
                (address, self._read_dint(address))
                for axis in axes
                for address in (
                    ABS_TARGET_ADDRESSES[axis],
                    ABS_SPEED_ADDRESSES[axis],
                )
            )

            self._write_single(MODE_ADDRESS, 2)
            self._verify_single_readback(MODE_ADDRESS, 2, "D5上位机模式")

            outbound = self._run_multi_axis_targets_leg(
                outbound_targets,
                speed,
                float(timeout),
                float(poll_interval),
                clock,
                sleep_fn,
                sample_callback,
            )
            sleep_fn(float(dwell_seconds))
            returned = self._run_multi_axis_targets_leg(
                start_positions,
                speed,
                float(timeout),
                float(poll_interval),
                clock,
                sleep_fn,
                sample_callback,
            )

            cleanup = self._cleanup_extended_motion(
                normal=True,
                parameter_backups=parameter_backups,
            )
            if cleanup.errors:
                latched_cleanup = self._cleanup_extended_motion(
                    normal=False,
                    parameter_backups=parameter_backups,
                )
                details = "；".join((*cleanup.errors, *latched_cleanup.errors))
                raise EngineeringSafetyError(
                    "四轴近同时往返参数恢复或清理异常并已锁存D7：" + details
                )
            return MultiAxisRoundTripResult(
                start_positions=start_positions,
                outbound=outbound,
                returned=returned,
                cleanup=cleanup,
            )
        except Exception as exc:
            cleanup = self._cleanup_extended_motion(
                normal=False,
                parameter_backups=parameter_backups,
            )
            details = "；".join(cleanup.errors)
            suffix = f"；清理异常：{details}" if details else ""
            raise EngineeringSafetyError(
                f"四轴近同时20 mm/20°往返异常并已执行锁存清理：{exc}{suffix}"
            ) from exc
        finally:
            self._operation_lock.release()

    def run_multi_axis_leg(
        self,
        target: int,
        d7_verified: bool,
        speed: int = 2,
        timeout: float = 20.0,
        poll_interval: float = 0.1,
        clock: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[MotionSample], None] | None = None,
    ) -> MultiAxisLegResult:
        """Run one four-axis near-simultaneous absolute-position leg."""
        if type(target) is not int or target not in (0, 10):
            raise ValueError("四轴目标只允许整数0或10 mm")
        if not isinstance(d7_verified, bool) or not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        if type(speed) is not int or not 1 <= speed <= 5:
            raise ValueError("四轴绝对定位速度只允许整数1到5 mm/s")
        if (
            isinstance(timeout, bool)
            or not isinstance(timeout, (int, float))
            or not math.isfinite(float(timeout))
            or float(timeout) <= 0
        ):
            raise ValueError("四轴绝对定位超时时间必须为有限正数")
        if (
            isinstance(poll_interval, bool)
            or not isinstance(poll_interval, (int, float))
            or not math.isfinite(float(poll_interval))
            or float(poll_interval) <= 0
        ):
            raise ValueError("四轴绝对定位轮询间隔必须为有限正数")
        if not callable(clock):
            raise ValueError("四轴绝对定位时钟必须可调用")
        if not callable(sleep_fn):
            raise ValueError("四轴绝对定位等待函数必须可调用")
        if sample_callback is not None and not callable(sample_callback):
            raise ValueError("四轴绝对定位采样回调必须可调用")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")

        axes = tuple(ABS_TRIGGER_ADDRESSES)
        timeout_seconds = float(timeout)
        poll_seconds = float(poll_interval)
        try:
            self._verify_stationary_preflight()
            self._verify_all_axes_alarm_free()

            start_positions = tuple(
                self._read_dint(POSITION_ADDRESSES[axis])
                for axis in axes
            )
            required_start = 0 if target == 10 else 10
            for axis, position in zip(axes, start_positions):
                if not required_start - 1 <= position <= required_start + 1:
                    raise EngineeringSafetyError(
                        f"{axis}轴四轴定位前必须位于"
                        f"{required_start}±1 mm，当前位置为{position} mm"
                    )

            parameter_backups = tuple(
                (address, self._read_dint(address))
                for axis in axes
                for address in (
                    ABS_TARGET_ADDRESSES[axis],
                    ABS_SPEED_ADDRESSES[axis],
                )
            )
            samples: list[MotionSample] = []
            done_seen = {axis: False for axis in axes}
            trigger_times: list[float] = []
            failure: Exception | None = None

            def capture_sample(started_at: float) -> MotionSample:
                positions = tuple(
                    self._read_dint(POSITION_ADDRESSES[axis])
                    for axis in axes
                )
                speeds = tuple(
                    self._read_dint(ACTUAL_SPEED_ADDRESSES[axis])
                    for axis in axes
                )
                dones = tuple(
                    self._read(ABS_DONE_ADDRESSES[axis], 1)[0]
                    for axis in axes
                )
                self._verify_all_axes_alarm_free()
                for axis, done in zip(axes, dones):
                    done_seen[axis] = done_seen[axis] or done == 1
                sample = MotionSample(
                    elapsed=max(0.0, clock() - started_at),
                    positions=positions,
                    speeds=speeds,
                )
                samples.append(sample)
                if sample_callback is not None:
                    sample_callback(sample)
                return sample

            try:
                self._write_single(MODE_ADDRESS, 2)
                self._verify_single_readback(
                    MODE_ADDRESS,
                    2,
                    "D5上位机模式",
                )

                for axis in axes:
                    target_address = ABS_TARGET_ADDRESSES[axis]
                    speed_address = ABS_SPEED_ADDRESSES[axis]
                    self._write_dint(target_address, target)
                    actual_target = self._read_dint(target_address)
                    if actual_target != target:
                        raise EngineeringSafetyError(
                            f"{axis}轴四轴定位目标参数回读失败："
                            f"期望{target}，实际{actual_target}"
                        )
                    self._write_dint(speed_address, speed)
                    actual_parameter_speed = self._read_dint(speed_address)
                    if actual_parameter_speed != speed:
                        raise EngineeringSafetyError(
                            f"{axis}轴四轴定位速度参数回读失败："
                            f"期望{speed}，实际{actual_parameter_speed}"
                        )

                observed: dict[str, int] = {}
                for axis in axes:
                    trigger_times.append(clock())
                    observed[axis] = self._start_self_clearing_trigger(
                        ABS_TRIGGER_ADDRESSES[axis],
                        f"{axis}轴四轴绝对定位命令",
                    )

                motion_started_at = trigger_times[0]
                sleep_fn(0.1)
                capture_sample(motion_started_at)

                for axis in axes:
                    if observed[axis] == 10:
                        self._write_single(ABS_TRIGGER_ADDRESSES[axis], 0)
                for axis in axes:
                    self._verify_single_readback(
                        ABS_TRIGGER_ADDRESSES[axis],
                        0,
                        f"{axis}轴四轴绝对定位命令",
                    )

                deadline = clock() + timeout_seconds
                while True:
                    remaining = deadline - clock()
                    if remaining <= 0:
                        raise EngineeringSafetyError(
                            "四轴绝对定位超时，未全部满足完成信号和速度归零"
                        )
                    sleep_fn(min(poll_seconds, remaining))
                    if clock() > deadline:
                        raise EngineeringSafetyError(
                            "四轴绝对定位超时，未全部满足完成信号和速度归零"
                        )
                    sample = capture_sample(motion_started_at)
                    if all(done_seen.values()) and not any(sample.speeds):
                        break
                    if clock() >= deadline:
                        raise EngineeringSafetyError(
                            "四轴绝对定位超时，未全部满足完成信号和速度归零"
                        )

                for index, axis in enumerate(axes):
                    if not any(
                        sample.positions[index] != start_positions[index]
                        for sample in samples
                    ):
                        raise EngineeringSafetyError(
                            f"{axis}轴四轴绝对定位未检测到位置变化"
                        )
                    if not any(sample.speeds[index] != 0 for sample in samples):
                        raise EngineeringSafetyError(
                            f"{axis}轴四轴绝对定位未检测到非零速度"
                        )
                    if not done_seen[axis]:
                        raise EngineeringSafetyError(
                            f"{axis}轴四轴绝对定位完成信号不是1"
                        )
                    if samples[-1].speeds[index] != 0:
                        raise EngineeringSafetyError(
                            f"{axis}轴四轴绝对定位最终速度不是0 mm/s"
                        )
                    final_position = samples[-1].positions[index]
                    if not target - 1 <= final_position <= target + 1:
                        raise EngineeringSafetyError(
                            f"{axis}轴四轴绝对定位最终位置超出目标±1 mm："
                            f"目标{target} mm，实际{final_position} mm"
                        )
                if not any(
                    sum(axis_speed != 0 for axis_speed in sample.speeds) >= 2
                    for sample in samples
                ):
                    raise EngineeringSafetyError(
                        "四轴绝对定位未检测到至少两轴速度重叠"
                    )
            except Exception as exc:
                failure = exc

            if failure is not None:
                cleanup = self._cleanup_extended_motion(
                    normal=False,
                    parameter_backups=parameter_backups,
                )
                cleanup_detail = "；".join(cleanup.errors)
                suffix = (
                    f"；清理异常：{cleanup_detail}"
                    if cleanup_detail
                    else ""
                )
                raise EngineeringSafetyError(
                    f"四轴绝对定位异常并已执行锁存清理：{failure}{suffix}"
                ) from failure

            cleanup = self._cleanup_extended_motion(
                normal=True,
                parameter_backups=parameter_backups,
            )
            if cleanup.errors:
                latched_cleanup = self._cleanup_extended_motion(
                    normal=False,
                    parameter_backups=parameter_backups,
                )
                details = "；".join(
                    (*cleanup.errors, *latched_cleanup.errors)
                )
                raise EngineeringSafetyError(
                    "四轴绝对定位参数恢复或清理异常并已锁存D7："
                    f"{details}"
                )

            trigger_skew = (
                max(trigger_times) - min(trigger_times)
                if trigger_times
                else 0.0
            )
            return MultiAxisLegResult(
                target=target,
                samples=tuple(samples),
                trigger_skew_seconds=trigger_skew,
                cleanup=cleanup,
            )
        finally:
            self._operation_lock.release()

    def _cleanup_after_jog(
        self,
        speed_address: int,
        original_speed: int,
        sleep_fn: Callable[[float], None],
    ) -> CleanupReport:
        errors: list[str] = []

        def attempt(label: str, action: Callable[[], None]) -> bool:
            try:
                action()
                return True
            except Exception as exc:
                errors.append(f"{label}失败：{exc}")
                return False

        for axis, address in JOG_ADDRESSES.items():
            attempt(
                f"清零{axis}轴点动触发",
                lambda address=address: self._write_single(address, 0),
            )

        d7_written = attempt(
            "写入D7急停",
            lambda: self._write_single(ESTOP_ADDRESS, 1),
        )
        if d7_written:
            attempt("等待D7清理保持时间", lambda: sleep_fn(1.0))
        attempt("恢复D5本地模式", lambda: self._write_single(MODE_ADDRESS, 0))
        attempt("解除D7急停", lambda: self._write_single(ESTOP_ADDRESS, 0))
        attempt(
            "恢复原点动速度",
            lambda: self._write_dint(speed_address, original_speed),
        )

        mode = -1
        estop = -1
        triggers = (-1, -1, -1, -1)
        restored_speed = -2147483648
        try:
            mode = self._read(MODE_ADDRESS, 1)[0]
            if mode != 0:
                errors.append(f"D5清理回读为{mode}")
        except Exception as exc:
            errors.append(f"读取D5清理结果失败：{exc}")
        try:
            estop = self._read(ESTOP_ADDRESS, 1)[0]
            if estop != 0:
                errors.append(f"D7清理回读为{estop}")
        except Exception as exc:
            errors.append(f"读取D7清理结果失败：{exc}")
        try:
            triggers = tuple(
                self._read(address, 1)[0]
                for address in JOG_ADDRESSES.values()
            )
            if triggers != (0, 0, 0, 0):
                errors.append(f"点动触发清理回读为{triggers}")
        except Exception as exc:
            errors.append(f"读取点动触发清理结果失败：{exc}")
        try:
            restored_speed = self._read_dint(speed_address)
            if restored_speed != original_speed:
                errors.append(
                    f"原点动速度恢复失败：期望{original_speed}，"
                    f"实际{restored_speed}"
                )
        except Exception as exc:
            errors.append(f"读取点动速度清理结果失败：{exc}")

        return CleanupReport(
            mode=mode,
            estop=estop,
            triggers=triggers,
            restored_speed=restored_speed,
            errors=tuple(errors),
        )

    def run_jog_test(
        self,
        request: JogRequest,
        d7_verified: bool,
        sleep_fn: Callable[[float], None] = time.sleep,
        sample_callback: Callable[[str, int, int], None] | None = None,
    ) -> JogResult:
        """执行一次受限的单轴低速点动，并在任何路径中清理。"""
        request.validate()
        if not d7_verified:
            raise EngineeringSafetyError("本连接会话尚未通过D7静止验证")
        if not self._operation_lock.acquire(blocking=False):
            raise EngineeringSafetyError("已有PLC测试正在运行")

        try:
            self._verify_stationary_preflight()
            self._verify_selected_axis_alarm_free(request.axis)

            speed_address = JOG_SPEED_ADDRESSES[request.axis]
            trigger_address = JOG_ADDRESSES[request.axis]
            position_address = POSITION_ADDRESSES[request.axis]
            actual_speed_address = ACTUAL_SPEED_ADDRESSES[request.axis]
            original_speed = self._read_dint(speed_address)
            start_position = self._read_dint(position_address)
            speed_samples: list[int] = []
            end_position = start_position
            failure: Exception | None = None

            try:
                self._write_dint(speed_address, int(request.speed))
                temporary_speed = self._read_dint(speed_address)
                if temporary_speed != int(request.speed):
                    raise EngineeringSafetyError(
                        f"临时点动速度回读失败：{temporary_speed}"
                    )

                self._write_single(MODE_ADDRESS, 2)
                self._verify_single_readback(MODE_ADDRESS, 2, "D5上位机模式")
                self._write_single(trigger_address, request.direction_value)
                self._verify_single_readback(
                    trigger_address,
                    request.direction_value,
                    f"{request.axis}轴点动触发",
                )

                sample_count = max(2, int(round(request.duration / 0.1)))
                interval = request.duration / sample_count
                for _ in range(sample_count):
                    sleep_fn(interval)
                    end_position = self._read_dint(position_address)
                    actual_speed = self._read_dint(actual_speed_address)
                    speed_samples.append(actual_speed)
                    if sample_callback is not None:
                        sample_callback(
                            request.axis,
                            end_position,
                            actual_speed,
                        )
            except Exception as exc:
                failure = exc
            finally:
                cleanup = self._cleanup_after_jog(
                    speed_address,
                    original_speed,
                    sleep_fn,
                )

            if not cleanup.ok:
                message = "；".join(cleanup.errors) or "清理结果异常"
                raise EngineeringSafetyError(
                    f"点动测试清理未全部成功：{message}"
                ) from failure
            if failure is not None:
                raise EngineeringSafetyError(
                    f"点动测试已中止并完成清理：{failure}"
                ) from failure

            return JogResult(
                axis=request.axis,
                direction_value=request.direction_value,
                requested_speed=int(request.speed),
                duration=float(request.duration),
                start_position=start_position,
                end_position=end_position,
                speed_samples=tuple(speed_samples),
                cleanup=cleanup,
            )
        finally:
            self._operation_lock.release()
