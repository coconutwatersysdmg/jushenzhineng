# -*- coding: utf-8 -*-
"""Livox Mid360 radar adapter used when runtime.device_mode=real."""
from __future__ import annotations

import socket
from pathlib import Path
from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from devices.base import RadarAdapter
from services.livox_service import LivoxService

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _local_ipv4_addresses() -> set[str]:
    """本机当前网卡上的 IPv4（不含仅回环以外的探测失败则尽量兜底）。"""
    found: set[str] = {"127.0.0.1"}
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = str(info[4][0] or "").strip()
            if ip:
                found.add(ip)
    except Exception:
        pass
    try:
        import psutil  # type: ignore
        for addrs in (psutil.net_if_addrs() or {}).values():
            for item in addrs or []:
                if getattr(item, "family", None) == socket.AF_INET:
                    ip = str(getattr(item, "address", "") or "").strip()
                    if ip:
                        found.add(ip)
    except Exception:
        pass
    # Windows 常见：用 UDP 连外网地址拿出站网卡 IP（不发包）
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("8.8.8.8", 80))
            ip = str(sock.getsockname()[0] or "").strip()
            if ip:
                found.add(ip)
        finally:
            sock.close()
    except Exception:
        pass
    return found


class RealLivoxRadarAdapter(RadarAdapter):
    """Capture a PCD via the Livox runtime shipped under third_party/livox_runtime."""

    def __init__(self, twin: DigitalTwinState, config: Mapping[str, Any] | None = None):
        self.twin = twin
        self.config = dict(config or {})
        # 统一外接设备配置已内含采集参数；不再依赖 livox_config.ini
        self.service = LivoxService(project_root=PROJECT_ROOT, config_file=None)
        self.pcd_path = ""
        self.use_live_capture = bool(self.config.get("use_live_capture", True))

    def set_point_cloud_path(self, path: str):
        self.pcd_path = str(path or "")

    def probe(self) -> dict:
        """连接检查：不采点云。

        真采模式必须同时满足：采集程序存在 + 配置的 host_ip 落在本机网卡上。
        只因 exe 在仓库里就报 ONLINE，会在非实验室电脑上误报「已连接」。
        """
        if not self.use_live_capture:
            self.twin.update_device("RADAR", status="ONLINE", task="FILE_MODE")
            return {
                "success": True,
                "device_id": "RADAR",
                "message": "雷达为离线 PCD 模式（未启用实采）",
                "use_live_capture": False,
            }
        if not self.service.exe_path.is_file():
            self.twin.update_device("RADAR", status="OFFLINE", task="LIVOX_MISSING")
            return {
                "success": False,
                "device_id": "RADAR",
                "message": f"Livox 采集程序不存在：{self.service.exe_path}",
            }
        host = str((self.config or {}).get("host_ip") or "").strip()
        if not host:
            self.twin.update_device("RADAR", status="OFFLINE", task="NO_HOST_IP")
            return {
                "success": False,
                "device_id": "RADAR",
                "message": "未配置雷达 host_ip（电脑连雷达网卡的 IPv4）",
                "exe_path": str(self.service.exe_path),
                "use_live_capture": True,
            }
        local_ips = _local_ipv4_addresses()
        if host not in local_ips:
            self.twin.update_device("RADAR", status="OFFLINE", task="HOST_IP_MISSING")
            return {
                "success": False,
                "device_id": "RADAR",
                "message": (
                    f"本机网卡没有雷达 host_ip={host}；"
                    f"当前 IPv4：{', '.join(sorted(local_ips)) or '无'}。"
                    "不在实验室 / 未接雷达网线时不应判定为已连接。"
                ),
                "host_ip": host,
                "local_ipv4": sorted(local_ips),
                "exe_path": str(self.service.exe_path),
                "use_live_capture": True,
            }
        self.twin.update_device("RADAR", status="ONLINE", task="PROBE_OK")
        return {
            "success": True,
            "device_id": "RADAR",
            "message": f"雷达网卡就绪（host_ip={host}，采集程序可用）",
            "host_ip": host,
            "exe_path": str(self.service.exe_path),
            "use_live_capture": True,
        }

    def locate_truck(self, cargo: dict) -> dict:
        self.twin.update_device("RADAR", status="ONLINE", task="LIVOX_CAPTURE")
        override = Path(str((cargo or {}).get("point_cloud_path") or self.pcd_path or ""))
        # 真采模式下忽略 examples 联调 PCD，避免“没改雷达也 SUCCESS”
        override_text = str(override).replace("\\", "/")
        allow_override = override.is_file() and not (
            self.use_live_capture and ("/examples/" in f"/{override_text}" or "example" in override_text.lower())
        )
        if allow_override:
            path = str(override.resolve())
            self.twin.update_device("RADAR", task="PCD_READY")
            return {
                "success": True,
                "pcd_path": path,
                "source": "override_file",
                "message": f"使用指定雷达 PCD：{path}",
            }

        if not self.service.exe_path.is_file():
            self.twin.update_device("RADAR", status="FAILED", task="LIVOX_MISSING")
            return {
                "success": False,
                "message": (
                    "Livox 采集程序不存在："
                    f"{self.service.exe_path}。请确认 third_party/livox_runtime 已部署。"
                ),
            }

        try:
            result = self.service.capture_once()
        except Exception as exc:
            self.twin.update_device("RADAR", status="FAILED", task="LIVOX_ERROR")
            return {"success": False, "message": f"Livox 采集异常：{exc}"}

        if not result.success or not Path(result.pcd_path).is_file():
            self.twin.update_device("RADAR", status="FAILED", task="LIVOX_FAILED")
            detail = (result.stderr or result.stdout or "").strip()
            return {
                "success": False,
                "return_code": result.return_code,
                "stdout": result.stdout,
                "stderr": result.stderr,
                "message": (
                    f"Livox 采集失败（return_code={result.return_code}）："
                    f"{detail or '未生成 PCD。请检查雷达网线、host_ip 与电脑网卡 IPv4 是否一致。'}"
                ),
            }

        path = str(Path(result.pcd_path).resolve())
        self.twin.update_device("RADAR", status="ONLINE", task="PCD_READY")
        return {
            "success": True,
            "pcd_path": path,
            "point_count": result.point_count,
            "source": "livox_mid360",
            "message": f"Livox Mid360 采集完成：{path}",
        }
