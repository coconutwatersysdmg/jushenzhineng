# -*- coding: utf-8 -*-
"""功能开关（独立文件，后续新开关也写这里）。

推荐：在软件界面顶部切换「运行模式」，会自动套用下面各组开关。
也可直接改本文件后重启；界面切换优先于文件默认值（会写入 workdir/ui_run_profile.json）。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROFILE_STATE_FILE = PROJECT_ROOT / "workdir" / "ui_run_profile.json"

# ===========================================================================
# 运行模式（前端三选一；权威入口）
# ===========================================================================
# sim   = 完全模拟：不连外设，Mock + 演示数据，现场算法
# lab   = 实验室：真机适配器 + 实验室雷达/相机算法
# field = 完全真实：真机适配器 + 现场 PointNet / corner_service
RUN_PROFILE = "field"

RUN_PROFILES: Dict[str, Dict[str, Any]] = {
    "sim": {
        "id": "sim",
        "title": "完全模拟",
        "subtitle": "不连外设，本地 Mock 跑通全流程",
        "switches": {
            "DEVICE_MODE": "mock",
            "ALLOW_DEMO_DEVICE_DATA": True,
            "ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO": True,
            "ALLOW_REAL_MOTION": False,
            "USE_LIVE_LIDAR_CAPTURE": False,
            "CORNER_REVIEW_ENABLED": True,
            "CORNER_REVIEW_AUTO_ACCEPT_DEMO": True,
            "USE_LAB_LIDAR_ALGO": False,
            "USE_LAB_CAMERA_ALGO": False,
        },
    },
    "lab": {
        "id": "lab",
        "title": "实验室",
        "subtitle": "实验室四步：设备检查→插孔∥雷达→相机精定位→俯拍校验；真机+实验室算法",
        "switches": {
            "DEVICE_MODE": "real",
            "ALLOW_DEMO_DEVICE_DATA": False,
            "ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO": True,
            "ALLOW_REAL_MOTION": True,
            "USE_LIVE_LIDAR_CAPTURE": True,
            "CORNER_REVIEW_ENABLED": False,
            "CORNER_REVIEW_AUTO_ACCEPT_DEMO": False,
            "USE_LAB_LIDAR_ALGO": True,
            "USE_LAB_CAMERA_ALGO": True,
        },
    },
    "field": {
        "id": "field",
        "title": "完全真实",
        "subtitle": "现场算法 + 真机适配器（暂定命名）",
        "switches": {
            "DEVICE_MODE": "real",
            "ALLOW_DEMO_DEVICE_DATA": False,
            "ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO": True,
            "ALLOW_REAL_MOTION": True,
            "USE_LIVE_LIDAR_CAPTURE": True,
            "CORNER_REVIEW_ENABLED": True,
            "CORNER_REVIEW_AUTO_ACCEPT_DEMO": False,
            "USE_LAB_LIDAR_ALGO": False,
            "USE_LAB_CAMERA_ALGO": False,
        },
    },
}

# ===========================================================================
# 1) 设备模式 / Mock 数据（也可被 RUN_PROFILE 覆盖）
# ===========================================================================

DEVICE_MODE = "real"
ALLOW_DEMO_DEVICE_DATA = True
ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO = True

# ===========================================================================
# 2) 真机运动 / 雷达实采
# ===========================================================================

ALLOW_REAL_MOTION = True
USE_LIVE_LIDAR_CAPTURE = True

# ===========================================================================
# 3) 角点人工审核
# ===========================================================================

CORNER_REVIEW_ENABLED = True
CORNER_REVIEW_AUTO_ACCEPT_DEMO = False

# ===========================================================================
# 4) 实验室 / 现场 算法切换
# ===========================================================================

USE_LAB_LIDAR_ALGO = False
USE_LAB_CAMERA_ALGO = False


def list_run_profiles() -> list[dict[str, Any]]:
    return [
        {
            "id": p["id"],
            "title": p["title"],
            "subtitle": p["subtitle"],
        }
        for p in RUN_PROFILES.values()
    ]


def get_run_profile(profile_id: str | None = None) -> dict[str, Any]:
    pid = str(profile_id or RUN_PROFILE or "field").strip().lower()
    if pid not in RUN_PROFILES:
        pid = "field"
    return dict(RUN_PROFILES[pid])


def apply_run_profile(profile_id: str, *, persist: bool = True) -> dict[str, Any]:
    """套用三模式之一，并同步到本模块与 external_devices 可变字段。"""
    global RUN_PROFILE
    global DEVICE_MODE, ALLOW_DEMO_DEVICE_DATA, ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO
    global ALLOW_REAL_MOTION, USE_LIVE_LIDAR_CAPTURE
    global CORNER_REVIEW_ENABLED, CORNER_REVIEW_AUTO_ACCEPT_DEMO
    global USE_LAB_LIDAR_ALGO, USE_LAB_CAMERA_ALGO

    profile = get_run_profile(profile_id)
    switches = dict(profile.get("switches") or {})
    RUN_PROFILE = str(profile["id"])
    DEVICE_MODE = str(switches.get("DEVICE_MODE", DEVICE_MODE))
    ALLOW_DEMO_DEVICE_DATA = bool(switches.get("ALLOW_DEMO_DEVICE_DATA", ALLOW_DEMO_DEVICE_DATA))
    ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO = bool(
        switches.get("ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO", ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO)
    )
    ALLOW_REAL_MOTION = bool(switches.get("ALLOW_REAL_MOTION", ALLOW_REAL_MOTION))
    USE_LIVE_LIDAR_CAPTURE = bool(switches.get("USE_LIVE_LIDAR_CAPTURE", USE_LIVE_LIDAR_CAPTURE))
    CORNER_REVIEW_ENABLED = bool(switches.get("CORNER_REVIEW_ENABLED", CORNER_REVIEW_ENABLED))
    CORNER_REVIEW_AUTO_ACCEPT_DEMO = bool(
        switches.get("CORNER_REVIEW_AUTO_ACCEPT_DEMO", CORNER_REVIEW_AUTO_ACCEPT_DEMO)
    )
    USE_LAB_LIDAR_ALGO = bool(switches.get("USE_LAB_LIDAR_ALGO", USE_LAB_LIDAR_ALGO))
    USE_LAB_CAMERA_ALGO = bool(switches.get("USE_LAB_CAMERA_ALGO", USE_LAB_CAMERA_ALGO))

    # 延迟同步，避免与 external_devices_config 循环导入
    try:
        import sys

        edc = sys.modules.get("config.external_devices_config")
        if edc is not None:
            edc.DEVICE_MODE = DEVICE_MODE
            if isinstance(getattr(edc, "GANTRY", None), dict):
                edc.GANTRY["allow_real_motion"] = ALLOW_REAL_MOTION
            if isinstance(getattr(edc, "LIVOX", None), dict):
                edc.LIVOX["use_live_capture"] = USE_LIVE_LIDAR_CAPTURE
    except Exception:
        pass

    if persist:
        try:
            import json

            PROFILE_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
            PROFILE_STATE_FILE.write_text(
                json.dumps({"run_profile": RUN_PROFILE}, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception:
            pass

    return {
        "id": RUN_PROFILE,
        "title": profile["title"],
        "subtitle": profile["subtitle"],
        "switches": {
            "DEVICE_MODE": DEVICE_MODE,
            "ALLOW_DEMO_DEVICE_DATA": ALLOW_DEMO_DEVICE_DATA,
            "ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO": ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO,
            "ALLOW_REAL_MOTION": ALLOW_REAL_MOTION,
            "USE_LIVE_LIDAR_CAPTURE": USE_LIVE_LIDAR_CAPTURE,
            "CORNER_REVIEW_ENABLED": CORNER_REVIEW_ENABLED,
            "CORNER_REVIEW_AUTO_ACCEPT_DEMO": CORNER_REVIEW_AUTO_ACCEPT_DEMO,
            "USE_LAB_LIDAR_ALGO": USE_LAB_LIDAR_ALGO,
            "USE_LAB_CAMERA_ALGO": USE_LAB_CAMERA_ALGO,
        },
    }


def load_persisted_run_profile() -> str:
    """启动时读取上次界面选择的模式；没有则用本文件 RUN_PROFILE。"""
    try:
        if PROFILE_STATE_FILE.is_file():
            import json

            data = json.loads(PROFILE_STATE_FILE.read_text(encoding="utf-8"))
            pid = str((data or {}).get("run_profile") or "").strip().lower()
            if pid in RUN_PROFILES:
                return pid
    except Exception:
        pass
    return str(RUN_PROFILE or "field")


# 导入时套用默认/上次模式（不依赖 external_devices_config）
apply_run_profile(load_persisted_run_profile(), persist=False)
