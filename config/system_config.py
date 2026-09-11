# -*- coding: utf-8 -*-
"""系统总配置（原 system_config.json）。

现场联调改这个文件即可，保存后重启程序。
"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict


# ---------------------------------------------------------------------------
# TODO: 与PLC交互 — 现场联调前改下面几项：
#   1) runtime["device_mode"] = "real"
#   2) devices["plc"] 填现场 PLC 的 Modbus TCP IP/端口
#   3) devices["gantry"] 配好 world_to_gantry，确认后再把 allow_real_motion=True
#   4) plc["ack_timeout_ms"] 按现场响应调
# ---------------------------------------------------------------------------

SYSTEM_CONFIG: Dict[str, Any] = {
    "runtime": {
        # "mock" = 本地假设备；现场真机改为 "real"
        "device_mode": "mock",
        "allow_demo_device_data": True,
        "allow_unimplemented_vision_measurement_zero": True,
    },
    "devices": {
        # TODO: 与PLC交互 — 填写现场 PLC 的 Modbus TCP 地址
        "plc": {
            "ip": "192.168.6.6",
            "port": 502,
        },
        "livox": {
            "config_file": "config/livox_config.ini",
        },
        # TODO: 与PLC交互 — 填 WORLD→龙门架 XYZR 映射；allow_real_motion 现场确认后再开
        "gantry": {
            "allow_real_motion": False,
            "world_to_gantry": {},
        },
        "camera": {
            "backend": "realsense_d435i",
        },
    },
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
        "path": "runtime/vehicle_loading.db",
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
        "enabled": True,
        "auto_accept_demo": False,
    },
    "camera_board_geometry": {
        "row_length_mm": 1200.0,
        "high_low_height_threshold_mm": 80.0,
        "max_borrow_mm": 650.0,
        "min_second_section_remaining_mm": 600.0,
    },
    # TODO: 与PLC交互 — 现场按 PLC 响应速度调整 ACK 超时（毫秒）
    "plc": {
        "ack_timeout_ms": 5000,
    },
}


def get_system_config() -> Dict[str, Any]:
    """返回系统配置深拷贝，避免运行时被意外改写。"""
    return deepcopy(SYSTEM_CONFIG)
