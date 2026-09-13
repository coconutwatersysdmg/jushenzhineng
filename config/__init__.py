# -*- coding: utf-8 -*-
"""项目配置包。

- 外接设备：config.external_devices_config
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

__all__ = [
    "SYSTEM_CONFIG",
    "get_system_config",
    "DEVICE_MODE",
    "get_device_layout_config",
    "get_external_devices_snapshot",
    "sync_livox_mid360_json",
    "sync_plc_gantry_settings_json",
]
