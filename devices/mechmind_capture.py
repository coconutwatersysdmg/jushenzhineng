# -*- coding: utf-8 -*-
"""梅卡 Mech-Eye 采图：本机直连优先；工控机路径预留。

主项目 runtime 为 Python 3.12，MechEyeAPI 官方仅 3.7–3.11。
因此优先用侧车 runtime_mecheye（tools/build_mecheye_runtime.bat 构建）；
若当前解释器已能 import mecheye，则进程内直接采。
"""
from __future__ import annotations

import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAPTURE_DIR = PROJECT_ROOT / "workdir" / "camera_captures" / "mechmind"
MECHEYE_RUNTIME_PYTHON = PROJECT_ROOT / "runtime_mecheye" / "python.exe"
MECHEYE_WORKER = PROJECT_ROOT / "tools" / "mechmind_grab_worker.py"


def _import_ok() -> tuple[bool, str]:
    try:
        from mecheye.area_scan_3d_camera import Camera  # type: ignore

        _ = Camera
        return True, "当前进程 mecheye 可用"
    except Exception as exc:
        return False, str(exc)


def mecheye_sidecar_ready() -> tuple[bool, str]:
    if MECHEYE_RUNTIME_PYTHON.is_file():
        return True, str(MECHEYE_RUNTIME_PYTHON)
    return False, (
        "缺少 runtime_mecheye。请运行 tools\\build_mecheye_runtime.bat "
        "（主 runtime 是 3.12，梅卡 SDK 需 3.11 侧车）"
    )


def sdk_available() -> tuple[bool, str]:
    ok, note = _import_ok()
    if ok:
        return True, note
    side_ok, side_note = mecheye_sidecar_ready()
    if side_ok:
        return True, f"将使用梅卡侧车：{side_note}"
    return False, f"未找到 mecheye（进程内：{note}；{side_note}）"


def discover_ips() -> list[str]:
    ok, _ = _import_ok()
    if ok:
        from mecheye.area_scan_3d_camera import Camera  # type: ignore

        infos = Camera.discover_cameras() or []
        return [
            str(getattr(info, "ip_address", "") or "").strip()
            for info in infos
            if str(getattr(info, "ip_address", "") or "").strip()
        ]
    side_ok, _ = mecheye_sidecar_ready()
    if not side_ok:
        return []
    code = (
        "import json; from mecheye.area_scan_3d_camera import Camera; "
        "print(json.dumps([str(getattr(i,'ip_address','') or '') for i in (Camera.discover_cameras() or [])]))"
    )
    try:
        proc = subprocess.run(
            [str(MECHEYE_RUNTIME_PYTHON), "-c", code],
            capture_output=True,
            text=True,
            timeout=30,
            cwd=str(PROJECT_ROOT),
        )
        if proc.returncode != 0:
            return []
        raw = (proc.stdout or "").strip().splitlines()[-1] if proc.stdout else "[]"
        ips = json.loads(raw)
        return [str(x).strip() for x in ips if str(x).strip()]
    except Exception:
        return []


def capture_rgbd_inprocess(
    *,
    ip: str = "",
    save_dir: str | Path | None = None,
    tag: str = "pallet",
) -> dict[str, Any]:
    """在当前解释器内采图（需已能 import mecheye）。"""
    ok, note = _import_ok()
    if not ok:
        return {"success": False, "route": "direct", "message": f"进程内无 mecheye：{note}"}

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


def _capture_via_sidecar(
    *,
    ip: str = "",
    save_dir: str | Path | None = None,
    tag: str = "pallet",
) -> dict[str, Any]:
    side_ok, side_note = mecheye_sidecar_ready()
    if not side_ok:
        return {"success": False, "route": "direct", "message": side_note}
    if not MECHEYE_WORKER.is_file():
        return {
            "success": False,
            "route": "direct",
            "message": f"缺少 worker：{MECHEYE_WORKER}",
        }
    out = Path(save_dir or DEFAULT_CAPTURE_DIR)
    out.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(MECHEYE_RUNTIME_PYTHON),
        str(MECHEYE_WORKER),
        "--ip",
        str(ip or ""),
        "--tag",
        str(tag or "pallet"),
        "--out-dir",
        str(out),
    ]
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120,
            cwd=str(PROJECT_ROOT),
        )
    except Exception as exc:
        return {"success": False, "route": "direct", "message": f"侧车采图启动失败：{exc}"}
    line = ""
    for candidate in reversed((proc.stdout or "").splitlines()):
        if candidate.strip().startswith("{"):
            line = candidate.strip()
            break
    if not line:
        err = (proc.stderr or proc.stdout or "").strip()[:500]
        return {
            "success": False,
            "route": "direct",
            "message": f"侧车无有效输出（code={proc.returncode}）：{err}",
        }
    try:
        result = json.loads(line)
    except Exception as exc:
        return {
            "success": False,
            "route": "direct",
            "message": f"侧车 JSON 解析失败：{exc}；raw={line[:200]}",
        }
    if isinstance(result, dict):
        result.setdefault("source", "mechmind_sidecar")
        result["sidecar_python"] = str(MECHEYE_RUNTIME_PYTHON)
    return result if isinstance(result, dict) else {
        "success": False,
        "route": "direct",
        "message": "侧车返回非 dict",
    }


def capture_rgbd(
    *,
    ip: str = "",
    save_dir: str | Path | None = None,
    tag: str = "pallet",
) -> dict[str, Any]:
    """本机直连采 RGB-D：进程内 mecheye 优先，否则走 runtime_mecheye 侧车。"""
    ok, _ = _import_ok()
    if ok:
        return capture_rgbd_inprocess(ip=ip, save_dir=save_dir, tag=tag)
    return _capture_via_sidecar(ip=ip, save_dir=save_dir, tag=tag)


def capture_rgbd_via_ipc(*, ipc_ip: str, tag: str = "pallet") -> dict[str, Any]:
    """工控机路径占位：当前不能从笔记本远程代拍。"""
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


def diagnose() -> dict[str, Any]:
    """给现场/调试用的自检信息。"""
    in_ok, in_note = _import_ok()
    side_ok, side_note = mecheye_sidecar_ready()
    return {
        "main_python": sys.executable,
        "main_version": sys.version.split()[0],
        "inprocess_mecheye": in_ok,
        "inprocess_note": in_note,
        "sidecar_ready": side_ok,
        "sidecar_note": side_note,
        "sdk_available": sdk_available()[0],
    }
