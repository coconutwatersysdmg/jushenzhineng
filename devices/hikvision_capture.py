# -*- coding: utf-8 -*-
"""海康 GigE 工业相机采图（不依赖 MVS 界面，依赖本机已装 MVS Runtime/SDK）。

现场：MVS 客户端能出图 → 通常本机已有 MvCameraControl；本模块直接 SDK 抓一帧存盘。
"""
from __future__ import annotations

import os
import sys
from ctypes import POINTER, byref, cast, memset, sizeof, c_ubyte
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAPTURE_DIR = PROJECT_ROOT / "workdir" / "camera_captures" / "hikvision"

# 常见 MVS 安装根目录（含自定义盘符 D:/E:）
_MVS_ROOT_CANDIDATES = [
    Path(r"C:\Program Files (x86)\MVS"),
    Path(r"C:\Program Files\MVS"),
    Path(r"D:\Program Files (x86)\MVS"),
    Path(r"D:\Program Files\MVS"),
    Path(r"E:\Program Files (x86)\MVS"),
    Path(r"E:\Program Files\MVS"),
]
_RUNTIME_DLL_DIRS = [
    Path(r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"),
    Path(r"C:\Program Files\Common Files\MVS\Runtime\Win64_x64"),
    Path(r"D:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"),
    Path(r"D:\Program Files\Common Files\MVS\Runtime\Win64_x64"),
]


def _discover_mvs_roots() -> list[Path]:
    roots: list[Path] = []
    for root in _MVS_ROOT_CANDIDATES:
        if root.is_dir() and root not in roots:
            roots.append(root)
    # 正在运行的 MVS.exe → 反推安装根目录
    try:
        import subprocess

        out = subprocess.check_output(
            ["wmic", "process", "where", "name='MVS.exe'", "get", "ExecutablePath"],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5,
        )
        for line in out.splitlines():
            line = line.strip()
            if not line.lower().endswith("mvs.exe"):
                continue
            # ...\Applications\Win64\MVS.exe → 上两级即安装根
            exe = Path(line)
            root = exe.parents[2] if len(exe.parents) >= 3 else exe.parent
            if root.is_dir() and root not in roots:
                roots.append(root)
    except Exception:
        pass
    env_root = os.environ.get("MVS_SDK_PATH") or os.environ.get("MVCAM_COMMON_RUNENV")
    if env_root:
        p = Path(env_root)
        # Runtime 环境变量时常指向 Common Files\MVS\Runtime\...
        for candidate in (p, p.parent, p.parent.parent if p.name.lower() == "runtime" else p):
            if candidate.is_dir() and (candidate / "Development").exists() and candidate not in roots:
                roots.append(candidate)
    return roots


def _ensure_mvs_paths() -> list[str]:
    added = []
    dll_dirs = list(_RUNTIME_DLL_DIRS)
    for root in _discover_mvs_roots():
        dll_dirs.append(root / "Runtime" / "Win64_x64")
        dll_dirs.append(Path(r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"))
    seen_dll = set()
    for dll_dir in dll_dirs:
        if not dll_dir.is_dir():
            continue
        path = str(dll_dir)
        if path in seen_dll:
            continue
        seen_dll.add(path)
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(path)
            except Exception:
                pass
        if path not in os.environ.get("PATH", ""):
            os.environ["PATH"] = path + os.pathsep + os.environ.get("PATH", "")
        added.append(path)

    py_dirs: list[Path] = [PROJECT_ROOT / "third_party" / "mvs_python"]
    for root in _discover_mvs_roots():
        py_dirs.append(root / "Development" / "Samples" / "Python")
        py_dirs.append(root / "Development" / "Samples" / "Python" / "MvImport")
    for folder in py_dirs:
        if folder.is_dir() and str(folder) not in sys.path:
            sys.path.insert(0, str(folder))
            added.append(str(folder))
    return added


def mvs_sdk_available() -> tuple[bool, str]:
    _ensure_mvs_paths()
    try:
        from MvCameraControl_class import MvCamera  # type: ignore

        _ = MvCamera
        return True, "MvCameraControl_class 可用"
    except Exception as exc:
        return False, f"未找到海康 Python SDK（请安装 MVS 并含 Development/Samples/Python）：{exc}"


def _ip_to_int(ip: str) -> int:
    parts = [int(x) for x in str(ip).strip().split(".")]
    if len(parts) != 4:
        raise ValueError(f"非法 IP：{ip}")
    return (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8) | parts[3]


def _frame_to_bgr(data: bytes, width: int, height: int, pixel_type: int):
    import numpy as np
    import cv2

    # 常见像素格式（与 CameraParams 宏一致的常用值）
    PixelType_Gvsp_Mono8 = 0x01080001
    PixelType_Gvsp_BayerRG8 = 0x01080009
    PixelType_Gvsp_BayerGB8 = 0x0108000A
    PixelType_Gvsp_BayerGR8 = 0x01080008
    PixelType_Gvsp_BayerBG8 = 0x0108000B
    PixelType_Gvsp_RGB8_Packed = 0x02180014
    PixelType_Gvsp_BGR8_Packed = 0x02180015

    arr = np.frombuffer(data, dtype=np.uint8)
    if pixel_type == PixelType_Gvsp_Mono8:
        gray = arr[: width * height].reshape((height, width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if pixel_type == PixelType_Gvsp_BGR8_Packed:
        return arr[: width * height * 3].reshape((height, width, 3)).copy()
    if pixel_type == PixelType_Gvsp_RGB8_Packed:
        rgb = arr[: width * height * 3].reshape((height, width, 3))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    bayer_map = {
        PixelType_Gvsp_BayerRG8: cv2.COLOR_BayerRG2BGR,
        PixelType_Gvsp_BayerGB8: cv2.COLOR_BayerGB2BGR,
        PixelType_Gvsp_BayerGR8: cv2.COLOR_BayerGR2BGR,
        PixelType_Gvsp_BayerBG8: cv2.COLOR_BayerBG2BGR,
    }
    if pixel_type in bayer_map:
        raw = arr[: width * height].reshape((height, width))
        return cv2.cvtColor(raw, bayer_map[pixel_type])
    # 兜底：按 Mono8 试
    if arr.size >= width * height:
        gray = arr[: width * height].reshape((height, width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise RuntimeError(f"暂不支持的像素格式：0x{int(pixel_type):x}")


def capture_one_frame(
    *,
    ip: str = "",
    save_dir: str | Path | None = None,
    tag: str = "corner",
    timeout_ms: int = 2000,
) -> dict[str, Any]:
    """打开海康 GigE 相机抓一帧，保存为 JPG。不弹 MVS 界面。"""
    ok, note = mvs_sdk_available()
    if not ok:
        return {"success": False, "message": note}

    from MvCameraControl_class import MvCamera  # type: ignore
    from CameraParams_header import (  # type: ignore
        MV_CC_DEVICE_INFO,
        MV_CC_DEVICE_INFO_LIST,
        MV_FRAME_OUT_INFO_EX,
        MV_GIGE_DEVICE,
    )

    try:
        from CameraParams_const import MV_ACCESS_Exclusive  # type: ignore
    except Exception:
        MV_ACCESS_Exclusive = 1

    out_dir = Path(save_dir or DEFAULT_CAPTURE_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        MvCamera.MV_CC_Initialize()
    except Exception:
        pass

    device_list = MV_CC_DEVICE_INFO_LIST()
    ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE, device_list)
    if ret != 0 or int(device_list.nDeviceNum) <= 0:
        return {
            "success": False,
            "message": f"未枚举到海康 GigE 相机（ret=0x{ret & 0xFFFFFFFF:x}）。检查网线/IP/是否被 MVS 占用。",
        }

    sel = 0
    selected_ip = ""
    want = str(ip or "").strip()
    for i in range(int(device_list.nDeviceNum)):
        info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        nip = int(info.SpecialInfo.stGigEInfo.nCurrentIp) & 0xFFFFFFFF
        cur = f"{(nip >> 24) & 0xFF}.{(nip >> 16) & 0xFF}.{(nip >> 8) & 0xFF}.{nip & 0xFF}"
        if not want or cur == want:
            sel = i
            selected_ip = cur
            break
    else:
        return {
            "success": False,
            "message": f"在线相机中未找到 IP={want}（共 {device_list.nDeviceNum} 台）",
            "ip": want,
        }

    cam = MvCamera()
    st_dev = cast(device_list.pDeviceInfo[sel], POINTER(MV_CC_DEVICE_INFO)).contents
    ret = cam.MV_CC_CreateHandle(st_dev)
    if ret != 0:
        return {"success": False, "message": f"CreateHandle 失败 0x{ret & 0xFFFFFFFF:x}", "ip": selected_ip}

    opened = False
    grabbing = False
    try:
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            return {
                "success": False,
                "message": f"打开相机失败 0x{ret & 0xFFFFFFFF:x}（请先关掉 MVS 里对该相机的连接）",
                "ip": selected_ip,
            }
        opened = True

        try:
            packet = cam.MV_CC_GetOptimalPacketSize()
            if packet > 0:
                cam.MV_CC_SetIntValue("GevSCPSPacketSize", packet)
        except Exception:
            pass

        # 连续取流
        try:
            cam.MV_CC_SetEnumValue("TriggerMode", 0)
        except Exception:
            pass

        from CameraParams_header import MVCC_INTVALUE  # type: ignore

        st_param = MVCC_INTVALUE()
        memset(byref(st_param), 0, sizeof(MVCC_INTVALUE))
        ret = cam.MV_CC_GetIntValue("PayloadSize", st_param)
        if ret != 0:
            return {"success": False, "message": f"读 PayloadSize 失败 0x{ret & 0xFFFFFFFF:x}", "ip": selected_ip}
        payload = int(st_param.nCurValue)

        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            return {"success": False, "message": f"StartGrabbing 失败 0x{ret & 0xFFFFFFFF:x}", "ip": selected_ip}
        grabbing = True

        frame_info = MV_FRAME_OUT_INFO_EX()
        memset(byref(frame_info), 0, sizeof(frame_info))
        data_buf = (c_ubyte * payload)()
        ret = cam.MV_CC_GetOneFrameTimeout(byref(data_buf), payload, frame_info, int(timeout_ms))
        if ret != 0:
            return {
                "success": False,
                "message": f"取流超时/失败 0x{ret & 0xFFFFFFFF:x}",
                "ip": selected_ip,
            }

        width = int(frame_info.nWidth)
        height = int(frame_info.nHeight)
        n_len = int(frame_info.nFrameLen)
        pixel_type = int(frame_info.enPixelType)
        raw = bytes(bytearray(data_buf)[:n_len])
        bgr = _frame_to_bgr(raw, width, height, pixel_type)

        import cv2

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        out_path = out_dir / f"hik_{tag}_{stamp}.jpg"
        if not cv2.imwrite(str(out_path), bgr):
            return {"success": False, "message": f"写盘失败：{out_path}", "ip": selected_ip}

        return {
            "success": True,
            "device_id": "CAM_CORNER",
            "image_path": str(out_path.resolve()),
            "rgb_path": str(out_path.resolve()),
            "width": width,
            "height": height,
            "ip": selected_ip,
            "source": "hikvision_sdk",
            "message": f"海康已拍照 {out_path.name}（{width}x{height}）",
        }
    finally:
        try:
            if grabbing:
                cam.MV_CC_StopGrabbing()
        except Exception:
            pass
        try:
            if opened:
                cam.MV_CC_CloseDevice()
        except Exception:
            pass
        try:
            cam.MV_CC_DestroyHandle()
        except Exception:
            pass
