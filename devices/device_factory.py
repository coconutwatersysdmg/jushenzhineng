# -*- coding: utf-8 -*-
"""Create PLC / robot / radar / camera adapters from system_config.runtime.device_mode.

- mock (default): local digital-twin demo with synthetic / example inputs
- real: hardware-facing adapters (Livox + PLC gantry + RealSense D435i)
"""
from __future__ import annotations

from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from devices.mock_devices import (
    MockArmCameraAdapter,
    MockPLCAdapter,
    MockRadarAdapter,
    MockRobotAdapter,
)


def normalize_device_mode(value: Any) -> str:
    mode = str(value or "mock").strip().lower()
    if mode in {"real", "hardware", "live"}:
        return "real"
    return "mock"


def create_device_adapters(
    twin: DigitalTwinState,
    system_config: Mapping[str, Any] | None = None,
):
    """Return (mode, plc, robot, radar, camera)."""
    cfg = dict(system_config or {})
    runtime = dict(cfg.get("runtime") or {})
    mode = normalize_device_mode(runtime.get("device_mode", "mock"))
    device_cfg = dict(cfg.get("devices") or {})

    if mode == "real":
        from devices.real_arm_camera_adapter import RealArmCameraAdapter
        from devices.real_gantry_robot_adapter import RealGantryRobotAdapter
        from devices.real_livox_radar_adapter import RealLivoxRadarAdapter
        from devices.real_plc_adapter import RealPlcAdapter

        # TODO: 与PLC交互 — 读 config/external_devices_config.py 的 PLC（现场 IP/端口）创建真机适配器
        plc = RealPlcAdapter(twin, device_cfg.get("plc") or {})
        # TODO: 与PLC交互 — 读 GANTRY（world_to_gantry / allow_real_motion）后才向 PLC 发真运动
        robot = RealGantryRobotAdapter(twin, plc, device_cfg.get("gantry") or {})
        radar = RealLivoxRadarAdapter(twin, device_cfg.get("livox") or {})
        camera = RealArmCameraAdapter(twin, device_cfg.get("camera") or {})
        return mode, plc, robot, radar, camera

    # TODO: 与PLC交互 — mock 模式：本地假 PLC；现场请在 external_devices_config.py 把 DEVICE_MODE 改为 real
    plc = MockPLCAdapter(twin)
    robot = MockRobotAdapter(twin, plc)
    radar = MockRadarAdapter(twin)
    camera = MockArmCameraAdapter(twin)
    return mode, plc, robot, radar, camera
