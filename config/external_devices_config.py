# -*- coding: utf-8 -*-
"""
外接设备参数（IP、端口、轴限位、外参路径、布局等）。

开关类配置请改：config/feature_switches.py
  - DEVICE_MODE
  - ALLOW_REAL_MOTION
  - USE_LIVE_LIDAR_CAPTURE
  - ALLOW_DEMO_DEVICE_DATA
  - 实验室算法开关等
"""
from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict

PROJECT_ROOT = Path(__file__).resolve().parents[1]

from config.feature_switches import (
    ALLOW_REAL_MOTION,
    DEVICE_MODE,
    USE_LIVE_LIDAR_CAPTURE,
)



# DEVICE_MODE / ALLOW_REAL_MOTION / USE_LIVE_LIDAR_CAPTURE
# 权威定义在 config/feature_switches.py，此处仅引用。

# ---- PLC（Modbus TCP）---- 对应 gantry_settings.json
PLC = {
    "ip": "192.168.6.6",
    "port": 502,
    "ack_timeout_ms": 5000,
}

# TODO PLC---- 龙门架 / 真机运动 ---- 对应 gantry_settings.json
GANTRY = {
    "allow_real_motion": bool(ALLOW_REAL_MOTION),
    "default_speed": 30.0,
    "move_timeout_s": 60.0,
    "axis_default_speeds": {
        "X": 30.0,
        "Y": 50.0,
        "Z": 30.0,
        "R": 30.0,
    },
    "axis_step": {
        "XYZ": 50.0,
        "R": 10.0,
    },
    "soft_limits": {
        "X": [0.0, 500.0],
        "Y": [0.0, 1300.0],
        "Z": [0.0, 380.0],
        "R": [-180.0, 180.0],
    },
    "work_origin": {
        "X": 0.0,
        "Y": 0.0,
        "Z": 0.0,
        "R": 0.0,
    },
    "r_forbidden_zones_enabled": False,
    "r_forbidden_zones": [],
    # 叉车臂运动时锁定 R：启动/首次运动时的角度保持不变，只动 XYZ
    "hold_r_axis": True,
    # TODO 待确认方向，由于目前没确认车头在哪，先打开看看了
    "world_to_gantry": {
        "placeholder": False,
        "world_origin_mm": {"x": -3500.0, "y": 0.0, "z": 0.0},
        "gantry_origin_xyzr": {"X": 0.0, "Y": 0.0, "Z": 0.0, "R": 0.0},
        "axis_map": {
            "X": "world_x_relative_mm",
            "Y": "world_y_relative_mm",
            "Z": "world_z_relative_mm",
            "R": "world_yaw_deg",
        },
        "scale": {"X": 1.0, "Y": 1.0, "Z": 1.0, "R": 1.0},
        "example_point": {
            "world": {"x_mm": -3500.0, "y_mm": 0.0, "z_mm": 0.0, "yaw_deg": 0.0},
            "gantry_xyzr": {"X": 0.0, "Y": 0.0, "Z": 0.0, "R": 0.0},
        },
    },
}

# TODO 雷达，对应 livox_mid360s配置
LIVOX = {
    "use_live_capture": bool(USE_LIVE_LIDAR_CAPTURE),
    "exe_path": "third_party/livox_runtime/livox_realtime_select_and_move.exe",
    "mid360_json_path": "third_party/livox_runtime/mid360s_config.json",
    "save_dir": "data/lidar",
    "capture_ms": 3000,
    "max_points": 300000,
    "timeout_sec": 20,
    # 电脑「连雷达网卡」IPv4，必须与现场 ipconfig 一致
    "host_ip": "192.168.1.50",
    "lidar_ports": {
        "cmd_data_port": 56100,
        "push_msg_port": 56200,
        "point_data_port": 56300,
        "imu_data_port": 56400,
        "log_data_port": 56500,
    },
    "host_ports": {
        "cmd_data_port": 56101,
        "push_msg_port": 56201,
        "point_data_port": 56301,
        "imu_data_port": 56401,
        "log_data_port": 56501,
    },
}

# TODO 相机配置 D435i
CAMERA = {
    "enabled": True,
    "backend": "realsense",
    "serial": "",  # 空=自动选择第一台 D435i
    "color_width": 1280,
    "color_height": 720,
    "depth_width": 1280,
    "depth_height": 720,
    "fps": 30,
    "depth_unit_mm": 1.0,
    # 现场已标定外参（迁自 Automatic loading system）
    "extrinsic_file": "config/camera_extrinsic.json",
    "plc_reference_r_deg": -80.0,
    # 旧标定包路径（内参等，可选）
    "calibration_dir": "config/sensor_coordinate_config",
    "intrinsic_file": "config/sensor_coordinate_config/camera_intrinsic.json",
    "capture_dir": "workdir/camera_captures",
}

# ---- 设备布局：臂 / 相机挂载 / 雷达位姿 / 拍摄角色 ----
DEVICE_LAYOUT: Dict[str, Any] = {
    "coordinate_frame": "world",
    "unit": "mm",
    "axis": {"x": "right", "y": "vehicle_forward", "z": "up"},
    "calibration_source": "config/sensor_coordinate_config",
    "radar": {
        "calibration_name": "lidar",
        "initial_pose": {
            "x_mm": 0,
            "y_mm": 500,
            "z_mm": 1800,
            "roll_deg": 0,
            "pitch_deg": -15,
            "yaw_deg": 0,
        },
    },
    "robots": {
        "PICK_ARM": {
            "role": "车尾左外侧初始位；沿双侧龙门架的纵轨、横梁运动",
            "initial_pose": {
                "x_mm": -3500,
                "y_mm": 1000,
                "z_mm": 1800,
                "roll_deg": 0,
                "pitch_deg": 0,
                "yaw_deg": -90,
            },
            "cargo_mount_pose": {
                "x_mm": 0,
                "y_mm": 1100,
                "z_mm": -350,
                "roll_deg": 0,
                "pitch_deg": 0,
                "yaw_deg": 0,
            },
        }
    },
    "cameras": {
        "CAM_PICK": {
            "calibration_name": "hole_camera",
            "parent_robot_id": "PICK_ARM",
            "sensor": "RGB-D",
            "role": "插孔、携货角点识别、临近托盘、放置及两面复检",
            # 现场标定参考姿态 PLC R=-80°（见 camera_extrinsic.json）
            "mount_pose": {
                "x_mm": 250,
                "y_mm": 0,
                "z_mm": -80,
                "roll_deg": 0,
                "pitch_deg": 0,
                "yaw_deg": 180,
            },
            "plc_reference_r_deg": -80.0,
        }
    },
    "camera_roles": {
        "pallet_hole": "CAM_PICK",
        "pre_pick_offset": "CAM_PICK",
        "corner": "CAM_PICK",
        "observe": "CAM_PICK",
        "neighbor": "CAM_PICK",
        "post": "CAM_PICK",
    },
    "notes": [
        "外接设备参数迁自 Automatic loading system 现场可联机配置。",
        "改本文件顶部 PLC / LIVOX / GANTRY / CAMERA 即可；Livox mid360s_config.json 会自动同步。",
    ],
}


def get_device_layout_config() -> Dict[str, Any]:
    return deepcopy(DEVICE_LAYOUT)


def get_livox_runtime_settings() -> Dict[str, str]:
    return {
        "exe_path": str(LIVOX["exe_path"]),
        "config_path": str(LIVOX["mid360_json_path"]),
        "save_dir": str(LIVOX["save_dir"]),
        "capture_ms": str(LIVOX["capture_ms"]),
        "max_points": str(LIVOX["max_points"]),
        "timeout_sec": str(LIVOX["timeout_sec"]),
    }


def build_mid360_sdk_json() -> Dict[str, Any]:
    return {
        "Mid360s": {
            "lidar_net_info": dict(LIVOX["lidar_ports"]),
            "host_net_info": [
                {
                    "host_ip": str(LIVOX["host_ip"]),
                    **dict(LIVOX["host_ports"]),
                }
            ],
        }
    }


def sync_livox_mid360_json(project_root: Path | None = None) -> Path:
    root = Path(project_root) if project_root else PROJECT_ROOT
    rel = Path(str(LIVOX["mid360_json_path"]))
    path = rel if rel.is_absolute() else (root / rel)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(build_mid360_sdk_json(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def sync_plc_gantry_settings_json(project_root: Path | None = None) -> Path:
    """同步写出 third_party/plc_finished_app/gantry_settings.json，供 console 使用。"""
    root = Path(project_root) if project_root else PROJECT_ROOT
    path = root / "third_party" / "plc_finished_app" / "gantry_settings.json"
    payload = {
        "plc_ip": str(PLC["ip"]),
        "plc_port": str(PLC["port"]),
        "default_speed": float(GANTRY.get("default_speed", 30.0)),
        "axis_default_speeds": dict(GANTRY.get("axis_default_speeds") or {}),
        "axis_step": dict(GANTRY.get("axis_step") or {}),
        "soft_limits": deepcopy(GANTRY.get("soft_limits") or {}),
        "r_forbidden_zones": list(GANTRY.get("r_forbidden_zones") or []),
        "work_origin": dict(GANTRY.get("work_origin") or {"X": 0, "Y": 0, "Z": 0, "R": 0}),
        "r_forbidden_zones_enabled": bool(GANTRY.get("r_forbidden_zones_enabled", False)),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def get_devices_section_for_system_config() -> Dict[str, Any]:
    import config.feature_switches as fs

    gantry = deepcopy(GANTRY)
    gantry["allow_real_motion"] = bool(fs.ALLOW_REAL_MOTION)
    return {
        "plc": {
            "ip": PLC["ip"],
            "port": PLC["port"],
        },
        "livox": {
            "use_live_capture": bool(fs.USE_LIVE_LIDAR_CAPTURE),
            "exe_path": LIVOX["exe_path"],
            "config_path": LIVOX["mid360_json_path"],
            "save_dir": LIVOX["save_dir"],
            "capture_ms": LIVOX["capture_ms"],
            "max_points": LIVOX["max_points"],
            "timeout_sec": LIVOX["timeout_sec"],
            "host_ip": LIVOX["host_ip"],
        },
        "gantry": gantry,
        "camera": deepcopy(CAMERA),
    }


def get_external_devices_snapshot() -> Dict[str, Any]:
    import config.feature_switches as fs

    devices = get_devices_section_for_system_config()
    return {
        "device_mode": fs.DEVICE_MODE,
        "run_profile": fs.RUN_PROFILE,
        "plc": deepcopy(PLC),
        "gantry": devices["gantry"],
        "livox": devices["livox"],
        "camera": deepcopy(CAMERA),
        "layout": get_device_layout_config(),
    }
