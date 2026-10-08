# -*- coding: utf-8 -*-
"""梅卡 Mech-Eye 采图：本机直连优先；工控机路径预留。

依赖：本机 Python 可 import mecheye（官方支持 3.7–3.11；项目 runtime 若为 3.12 需另装兼容包或换环境）。
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAPTURE_DIR = PROJECT_ROOT / "workdir" / "camera_captures" / "mechmind"


def sdk_available() -> tuple[bool, str]:
    try:
        from mecheye.area_scan_3d_camera import Camera  # type: ignore

        _ = Camera
        return True, "mecheye SDK 可用"
    except Exception as exc:
        return False, f"未找到 mecheye（请 pip install MechEyeAPI，Python 3.7–3.11）：{exc}"


def discover_ips() -> list[str]:
    ok, _ = sdk_available()
    if not ok:
        return []
    from mecheye.area_scan_3d_camera import Camera  # type: ignore

    infos = Camera.discover_cameras() or []
    ips = []
    for info in infos:
        ip = str(getattr(info, "ip_address", "") or "").strip()
        if ip:
            ips.append(ip)
    return ips


def capture_rgbd(
    *,
    ip: str = "",
    save_dir: str | Path | None = None,
    tag: str = "pallet",
) -> dict[str, Any]:
    """本机直连：按 IP（或枚举到的第一台）采 RGB + 深度图。"""
    ok, note = sdk_available()
    if not ok:
        return {"success": False, "route": "direct", "message": note}

    from mecheye.area_scan_3d_camera import (  # type: ignore
        Camera,
        ColorTypeOf2DCamera_Monochrome,
        Frame2D,
        Frame3D,
    )
    import cv2
    import numpy as np

    want = str(ip or "").strip()
    camera = Camera()
    connected_ip = want
    try:
        if want:
            status = camera.connect(want)
            if not status.is_ok():
                return {
                    "success": False,
                    "route": "direct",
                    "ip": want,
                    "message": f"直连相机失败 IP={want}：{status.error_description()}",
                }
        else:
            infos = Camera.discover_cameras() or []
            if not infos:
                return {
                    "success": False,
                    "route": "direct",
                    "message": "直连：未发现梅卡相机（检查网线/同网段/是否被 Viewer 占用）",
                }
            connected_ip = str(getattr(infos[0], "ip_address", "") or "")
            status = camera.connect(infos[0])
            if not status.is_ok():
                return {
                    "success": False,
                    "route": "direct",
                    "ip": connected_ip,
                    "message": f"直连枚举相机失败：{status.error_description()}",
                }

        out_dir = Path(save_dir or DEFAULT_CAPTURE_DIR)
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        rgb_path = out_dir / f"mech_{tag}_{stamp}_rgb.png"
        depth_path = out_dir / f"mech_{tag}_{stamp}_depth.png"

        frame2d = Frame2D()
        st2 = camera.capture_2d(frame2d)
        if not st2.is_ok():
            return {
                "success": False,
                "route": "direct",
                "ip": connected_ip,
                "message": f"采 2D 失败：{st2.error_description()}",
            }
        if frame2d.color_type() == ColorTypeOf2DCamera_Monochrome:
            image2d = frame2d.get_gray_scale_image()
        else:
            image2d = frame2d.get_color_image()
        rgb = np.asarray(image2d.data())
        if rgb.ndim == 2:
            rgb = cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)
        if not cv2.imwrite(str(rgb_path), rgb):
            return {"success": False, "route": "direct", "message": f"写 RGB 失败：{rgb_path}"}

        frame3d = Frame3D()
        st3 = camera.capture_3d(frame3d)
        if not st3.is_ok():
            return {
                "success": False,
                "route": "direct",
                "ip": connected_ip,
                "message": f"采深度失败：{st3.error_description()}",
            }
        depth_map = frame3d.get_depth_map()
        depth = np.asarray(depth_map.data(), dtype=np.float32)
        # 存 16U 毫米深度便于后续算法；无效用 0
        depth_mm = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth_u16 = np.clip(depth_mm, 0, 65535).astype(np.uint16)
        if not cv2.imwrite(str(depth_path), depth_u16):
            return {"success": False, "route": "direct", "message": f"写深度失败：{depth_path}"}

        return {
            "success": True,
            "route": "direct",
            "device_id": "CAM_PALLET",
            "ip": connected_ip,
            "rgb_path": str(rgb_path.resolve()),
            "depth_path": str(depth_path.resolve()),
            "image_path": str(rgb_path.resolve()),
            "source": "mechmind_sdk_direct",
            "message": f"梅卡直连已拍照 {rgb_path.name}",
        }
    finally:
        try:
            camera.disconnect()
        except Exception:
            pass


def capture_rgbd_via_ipc(*, ipc_ip: str, tag: str = "pallet") -> dict[str, Any]:
    """工控机路径占位：当前不能从笔记本远程代拍，仅返回明确失败与后续 TODO。"""
    ipc = str(ipc_ip or "").strip()
    return {
        "success": False,
        "route": "ipc",
        "ipc_ip": ipc,
        "message": (
            f"直连失败后已切换工控机路由（ipc_ip={ipc or '未配置'}），"
            "但远程采图尚未实现：请到工控机用 Viewer 验图，或在工控机上运行本程序。"
            # TODO: 工控机远程采图（RPC/共享目录/在工控机部署 runtime）
        ),
    }
