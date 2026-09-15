# -*- coding: utf-8 -*-
"""系统总配置。

外接设备（PLC / 雷达 / 龙门架 / 相机 / 臂布局）已集中到：
  config/external_devices_config.py
现场改设备请改那个文件；本文件保留算法/数据库/流程类参数。
"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict

from config.external_devices_config import (
    DEVICE_MODE,
    PLC,
    get_devices_section_for_system_config,
    sync_livox_mid360_json,
    sync_plc_gantry_settings_json,
)
from config.feature_switches import (
    ALLOW_DEMO_DEVICE_DATA,
    ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO,
    CORNER_REVIEW_AUTO_ACCEPT_DEMO,
    CORNER_REVIEW_ENABLED,
)


SYSTEM_CONFIG: Dict[str, Any] = {
    "runtime": {
        # 设备模式 / Mock 开关来自 config/feature_switches.py
        "device_mode": DEVICE_MODE,
        "allow_demo_device_data": bool(ALLOW_DEMO_DEVICE_DATA),
        "allow_unimplemented_vision_measurement_zero": bool(ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO),
    },
    "devices": get_devices_section_for_system_config(),
    "database": {
        "enabled": True,
        "driver": "mysql",
        "host": "127.0.0.1",
        "port": 3306,
        "database": "jushenzhineng",
        "user": "root",
        "password": "123456",
        "password_env": "JUSHEN_MYSQL_PASSWORD",
        "auto_start": False,
        "initialize_schema": False,
        "path": "workdir/vehicle_loading.db",
        "center_entity": "truck",
    },
    "pallet_cargo_offset": {
        "max_overhang_percent": 5.0,
        "reference_pallet_width_mm": 1200.0,
    },
    "camera_corner_world": {
        "depth_mode": "raw",
        "depth_sample_window": 5,
        "coarse_radar_warning_mm": 800.0,
    },
    "corner_review": {
        "enabled": bool(CORNER_REVIEW_ENABLED),
        "auto_accept_demo": bool(CORNER_REVIEW_AUTO_ACCEPT_DEMO),
    },
    "camera_board_geometry": {
        "row_length_mm": 1200.0,
        "high_low_height_threshold_mm": 80.0,
        "max_borrow_mm": 650.0,
        "min_second_section_remaining_mm": 600.0,
    },
    "plc": {
        "ack_timeout_ms": int(PLC.get("ack_timeout_ms", 5000)),
    },
}


def get_system_config() -> Dict[str, Any]:
    """返回系统配置深拷贝；devices 每次从外接设备配置刷新。"""
    # 将 Python 配置同步到第三方工具使用的 JSON
    try:
        sync_livox_mid360_json()
        sync_plc_gantry_settings_json()
    except Exception:
        pass
    cfg = deepcopy(SYSTEM_CONFIG)
    cfg["runtime"]["device_mode"] = DEVICE_MODE
    cfg["runtime"]["allow_demo_device_data"] = bool(ALLOW_DEMO_DEVICE_DATA)
    cfg["runtime"]["allow_unimplemented_vision_measurement_zero"] = bool(
        ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO
    )
    cfg["corner_review"] = {
        "enabled": bool(CORNER_REVIEW_ENABLED),
        "auto_accept_demo": bool(CORNER_REVIEW_AUTO_ACCEPT_DEMO),
    }
    cfg["devices"] = get_devices_section_for_system_config()
    cfg["plc"] = {"ack_timeout_ms": int(PLC.get("ack_timeout_ms", 5000))}
    return cfg
