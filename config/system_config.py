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
        # TODO "mock" = 本地假设备；现场真机改为 "real"
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
        # TODO 与PLC交互 — 下面是瞎填的占位映射，方便跟机械同学对接时对照改数
        # TODO allow_real_motion 必须保持 False；乱开会按错坐标动真机
        # 真写入代码已接好（devices/gantry_modbus_motion.py）；缺的是现场标定后把 placeholder=False
        "gantry": {
            "allow_real_motion": False,
            "default_speed": 30.0,
            "move_timeout_s": 60.0,
            "soft_limits": {
                "X": [0.0, 500.0],
                "Y": [0.0, 1300.0],
                "Z": [0.0, 380.0],
                "R": [-180.0, 180.0],
            },
            # 占位示例：把软件 WORLD(mm) 换成 PLC 龙门架轴 XYZR
            # 真实公式/零点/方向以现场标定为准；计算机同学一般只负责填进这里
            "world_to_gantry": {
                "placeholder": True,  # True=假数据，现场标定后改 False 并替换数值
                # WORLD 坐标系里“龙门架零点”大概对应哪（软件侧）
                "world_origin_mm": {"x": -3500.0, "y": 0.0, "z": 0.0},
                # PLC 轴坐标系里工作原点（成品上位机默认：X0 Y0 Z120 R0）
                "gantry_origin_xyzr": {"X": 0.0, "Y": 0.0, "Z": 120.0, "R": 0.0},
                # 轴对应关系（示意）：软件哪个量驱动 PLC 哪根轴
                "axis_map": {
                    "X": "world_x_relative_mm",   # 左右横移
                    "Y": "world_y_relative_mm",   # 前后
                    "Z": "world_z_relative_mm",   # 升降
                    "R": "world_yaw_deg",         # 旋转角
                },
                # 比例/符号（示意）：1.0 表示同向同毫米；-1.0 表示方向相反
                "scale": {"X": 1.0, "Y": 1.0, "Z": 1.0, "R": 1.0},
                # 一对点对照例子（瞎编）：同一物理位置，两边各怎么写
                "example_point": {
                    "world": {"x_mm": -3500.0, "y_mm": 1000.0, "z_mm": 1950.0, "yaw_deg": -90.0},
                    "gantry_xyzr": {"X": 0.0, "Y": 250.0, "Z": 200.0, "R": -90.0},
                },
            },
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
