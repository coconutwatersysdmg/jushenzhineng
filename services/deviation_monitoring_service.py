# -*- coding: utf-8 -*-
"""托盘/货物偏差判定与卡车底板托盘动态监测。"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

from config.system_config import get_system_config

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class DeviationMonitoringService:
    def __init__(self, config_path: Path | None = None):
        # config_path 保留仅为兼容旧调用；配置已改为 config/system_config.py
        self.config_path = Path(config_path) if config_path else PROJECT_ROOT / "config" / "system_config.py"
        self.config = self._load_config()

    def _load_config(self) -> Dict[str, Any]:
        return get_system_config()

    def evaluate_pallet_cargo(self, measurement: Dict[str, Any], stage: str) -> Dict[str, Any]:
        gate = self.config.get("deviation_gate", {}).get("pallet_cargo", {})
        dx = float(measurement.get("dx_mm", 0.0))
        dy = float(measurement.get("dy_mm", 0.0))
        dz = float(measurement.get("dz_mm", 0.0))
        yaw = float(measurement.get("yaw_deg", 0.0))
        limits = {
            "x_mm": float(gate.get("max_abs_x_mm", 30.0)),
            "y_mm": float(gate.get("max_abs_y_mm", 30.0)),
            "z_mm": float(gate.get("max_abs_z_mm", 20.0)),
            "yaw_deg": float(gate.get("max_abs_yaw_deg", 3.0)),
        }
        checks = {
            "x": abs(dx) <= limits["x_mm"],
            "y": abs(dy) <= limits["y_mm"],
            "z": abs(dz) <= limits["z_mm"],
            "yaw": abs(yaw) <= limits["yaw_deg"],
        }
        passed = all(checks.values())
        return {
            "success": passed,
            "stage": stage,
            "measurement": {"dx_mm": dx, "dy_mm": dy, "dz_mm": dz, "yaw_deg": yaw},
            "limits": limits,
            "checks": checks,
            "message": (
                f"{stage}托盘/货物偏差合格"
                if passed else
                f"{stage}托盘/货物偏差超限，禁止继续自动动作"
            ),
        }

    @staticmethod
    def _distance_mm(a: Dict[str, Any], b: Dict[str, Any]) -> float:
        return math.sqrt(
            (float(a.get("x_mm", 0.0)) - float(b.get("x_mm", 0.0))) ** 2
            + (float(a.get("y_mm", 0.0)) - float(b.get("y_mm", 0.0))) ** 2
            + (float(a.get("z_mm", 0.0)) - float(b.get("z_mm", 0.0))) ** 2
        )

    def evaluate_dynamic_monitor(self, monitor_result: Dict[str, Any], phase: str) -> Dict[str, Any]:
        cfg = self.config.get("dynamic_monitor", {})
        max_drift = float(cfg.get("max_position_drift_mm", 15.0))
        max_yaw_drift = float(cfg.get("max_yaw_drift_deg", 2.0))
        pallets = []
        all_stable = True
        for item in monitor_result.get("pallets", []) or []:
            samples = item.get("samples", []) or []
            max_pos = 0.0
            max_yaw = 0.0
            if samples:
                origin = samples[0]
                for sample in samples[1:]:
                    max_pos = max(max_pos, self._distance_mm(origin, sample))
                    max_yaw = max(max_yaw, abs(float(sample.get("yaw_deg", 0.0)) - float(origin.get("yaw_deg", 0.0))))
            stable = max_pos <= max_drift and max_yaw <= max_yaw_drift
            all_stable = all_stable and stable
            pallets.append({
                "instance_id": item.get("instance_id"),
                "latest_pose": samples[-1] if samples else item.get("latest_pose", {}),
                "sample_count": len(samples),
                "max_position_drift_mm": round(max_pos, 3),
                "max_yaw_drift_deg": round(max_yaw, 3),
                "stable": stable,
            })
        return {
            "success": all_stable,
            "phase": phase,
            "pallets": pallets,
            "limits": {
                "max_position_drift_mm": max_drift,
                "max_yaw_drift_deg": max_yaw_drift,
            },
            "message": (
                f"{phase}卡车底板托盘动态监测稳定"
                if all_stable else
                f"{phase}检测到托盘位移/转角漂移超限，禁止继续放货"
            ),
        }
