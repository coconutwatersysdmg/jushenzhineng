# -*- coding: utf-8 -*-
"""便携 runtime / 项目资源环境检查。"""
from __future__ import annotations

import os
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _ok(msg: str) -> None:
    print(f"[OK]      {msg}")


def _missing(msg: str) -> None:
    print(f"[MISSING] {msg}")


def _info(msg: str) -> None:
    print(f"[INFO]    {msg}")


def _warn(msg: str) -> None:
    print(f"[WARN]    {msg}")


def check_python() -> bool:
    print("=== Python ===")
    exe = Path(sys.executable).resolve()
    _info(f"Python executable : {exe}")
    _info(f"Python version    : {sys.version.replace(chr(10), ' ')}")
    _info(f"Project root      : {PROJECT_ROOT}")
    _info(f"Working directory : {Path.cwd().resolve()}")

    expected = (PROJECT_ROOT / "runtime" / "python.exe").resolve()
    ok = True
    if exe == expected:
        _ok("sys.executable 指向项目 runtime\\python.exe")
    else:
        _warn(f"sys.executable 不是 runtime\\python.exe（期望: {expected}）")
        if "runtime" not in str(exe).replace("/", "\\").lower():
            ok = False
            _missing("当前解释器不是项目便携 runtime，启动脚本可能未使用 runtime\\python.exe")
    return ok


def check_packages() -> bool:
    print("\n=== 核心第三方包 ===")
    specs = [
        ("PySide6", "PySide6"),
        ("numpy", "numpy"),
        ("cv2", "opencv-python"),
        ("torch", "torch"),
        ("ultralytics", "ultralytics"),
        ("pyrealsense2", "pyrealsense2"),
        ("pymodbus", "pymodbus"),
        ("open3d", "open3d"),
        ("pymysql", "PyMySQL"),
        ("PIL", "Pillow"),
    ]
    all_ok = True
    for import_name, pip_name in specs:
        try:
            mod = __import__(import_name)
            ver = getattr(mod, "__version__", getattr(mod, "VERSION", "?"))
            _ok(f"{pip_name:16s} import={import_name} version={ver}")
        except Exception as exc:
            all_ok = False
            _missing(f"{pip_name:16s} ({type(exc).__name__}: {exc})")
    return all_ok


def check_cuda() -> None:
    print("\n=== CUDA / PyTorch ===")
    try:
        import torch
    except Exception as exc:
        _missing(f"无法 import torch: {exc}")
        return

    _info(f"torch version     : {torch.__version__}")
    try:
        available = bool(torch.cuda.is_available())
    except Exception as exc:
        _warn(f"检查 CUDA 时出错（已忽略崩溃）: {exc}")
        return

    if not available:
        _info("torch.cuda.is_available() = False")
        _info("当前可为 CPU 推理；目标机若要用 GPU，需安装兼容 NVIDIA 驱动，并使用匹配的 CUDA 版 PyTorch wheel。")
        return

    _ok("torch.cuda.is_available() = True")
    try:
        _info(f"torch.version.cuda : {torch.version.cuda}")
        _info(f"device 0           : {torch.cuda.get_device_name(0)}")
    except Exception as exc:
        _warn(f"读取 CUDA 设备信息失败: {exc}")


def check_resources() -> bool:
    print("\n=== 项目资源 ===")
    required = [
        "main.py",
        "requirements.txt",
        "config/external_devices_config.py",
        "config/system_config.py",
        "config/camera_extrinsic.json",
        "config/sensor_coordinate_config/camera_intrinsic.json",
        "config/sensor_coordinate_config/coordinate_config.json",
        "models/corner_service.pt",
        "models/pallet_hole_best.pt",
        "algorithm_modules/point_cloud_segment_module/checkpoints/best_model.pth",
        "ui/qml/TwinScene.qml",
        "data/loading_plan.json",
        "third_party/livox_runtime/livox_realtime_select_and_move.exe",
        "third_party/livox_runtime/mid360s_config.json",
        "third_party/plc_finished_app/gantry_settings.json",
    ]
    all_ok = True
    for rel in required:
        path = PROJECT_ROOT / rel
        if path.exists():
            _ok(rel)
        else:
            all_ok = False
            _missing(rel)
    return all_ok


def main() -> int:
    os.chdir(PROJECT_ROOT)
    print("巨神智能 / jushenzhineng_v3 环境检查\n")
    ok_py = check_python()
    ok_pkg = check_packages()
    check_cuda()
    ok_res = check_resources()

    print("\n=== 硬件驱动提醒（非 Python 包）===")
    _info("Intel RealSense D435i：需要 Windows RealSense 驱动/Runtime（与 pyrealsense2 包不同）")
    _info("NVIDIA GPU：仅在使用 CUDA 版 PyTorch 时需要兼容显卡驱动（通常不必装完整 CUDA Toolkit）")
    _info("Livox Mid360：依赖 third_party/livox_runtime 内 exe/dll，以及现场网卡/IP 配置")
    _info("PLC：现场 Modbus TCP 网口连通；IP/端口见 config/external_devices_config.py")

    print("\n=== 汇总 ===")
    failed = 0
    if not ok_py:
        failed += 1
        _missing("Python runtime 路径不符合便携要求")
    else:
        _ok("Python runtime")
    if not ok_pkg:
        failed += 1
        _missing("存在缺失的核心 Python 包")
    else:
        _ok("核心 Python 包")
    if not ok_res:
        failed += 1
        _missing("存在缺失的项目资源文件")
    else:
        _ok("项目资源")

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
