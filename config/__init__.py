# -*- coding: utf-8 -*-
"""项目配置包。

- 功能开关：config.feature_switches（Mock/真机/实验室算法等，优先改这里）
- 外接设备：config.external_devices_config（IP、外参路径、限位等）
- 系统/算法：config.system_config
"""

from config.external_devices_config import (
    DEVICE_MODE,
    get_device_layout_config,
    get_external_devices_snapshot,
    sync_livox_mid360_json,
    sync_plc_gantry_settings_json,
)
from config.system_config import SYSTEM_CONFIG, get_system_config
from config.feature_switches import (
    ALLOW_DEMO_DEVICE_DATA,
    ALLOW_REAL_MOTION,
    ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO,
    CORNER_REVIEW_AUTO_ACCEPT_DEMO,
    CORNER_REVIEW_ENABLED,
    USE_LAB_CAMERA_ALGO,
    USE_LAB_LIDAR_ALGO,
    USE_LIVE_LIDAR_CAPTURE,
)

__all__ = [
    "SYSTEM_CONFIG",
    "get_system_config",
    "DEVICE_MODE",
    "ALLOW_DEMO_DEVICE_DATA",
    "ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO",
    "ALLOW_REAL_MOTION",
    "USE_LIVE_LIDAR_CAPTURE",
    "CORNER_REVIEW_ENABLED",
    "CORNER_REVIEW_AUTO_ACCEPT_DEMO",
    "USE_LAB_LIDAR_ALGO",
    "USE_LAB_CAMERA_ALGO",
    "get_device_layout_config",
    "get_external_devices_snapshot",
    "sync_livox_mid360_json",
    "sync_plc_gantry_settings_json",
]
