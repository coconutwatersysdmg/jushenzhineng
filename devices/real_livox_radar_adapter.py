# -*- coding: utf-8 -*-
"""Livox Mid360 radar adapter used when runtime.device_mode=real."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from devices.base import RadarAdapter
from services.livox_service import LivoxService

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class RealLivoxRadarAdapter(RadarAdapter):
    """Capture a PCD via the Livox runtime shipped under third_party/livox_runtime."""

    def __init__(self, twin: DigitalTwinState, config: Mapping[str, Any] | None = None):
        self.twin = twin
        self.config = dict(config or {})
        config_file = self.config.get("config_file") or (PROJECT_ROOT / "config" / "livox_config.ini")
        self.service = LivoxService(project_root=PROJECT_ROOT, config_file=config_file)
        self.pcd_path = ""

    def set_point_cloud_path(self, path: str):
        self.pcd_path = str(path or "")

    def locate_truck(self, cargo: dict) -> dict:
        self.twin.update_device("RADAR", status="ONLINE", task="LIVOX_CAPTURE")
        override = Path(str((cargo or {}).get("point_cloud_path") or self.pcd_path or ""))
        if override.is_file():
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
                "message": f"Livox 采集失败：{detail or '未生成 PCD'}",
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
