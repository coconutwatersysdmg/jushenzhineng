# -*- coding: utf-8 -*-
from __future__ import annotations

from copy import deepcopy
from datetime import datetime
from pathlib import Path
from threading import RLock
from typing import Any, Dict
import json

from .geometry import Pose6D, compose_pose, parent_pose_for_child, relative_pose
from config.external_devices_config import get_device_layout_config


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CAMERA_INTRINSIC_CONFIG = PROJECT_ROOT / "config" / "sensor_coordinate_config" / "camera_intrinsic.json"


class DigitalTwinState:
    """唯一数字孪生状态源。相机世界坐标由父机械臂位姿和安装外参派生。"""

    def __init__(self):
        self._lock = RLock()
        self.reset()

    @staticmethod
    def _load_device_config():
        try:
            return get_device_layout_config()
        except Exception:
            return {}

    @staticmethod
    def _load_intrinsic_config():
        if CAMERA_INTRINSIC_CONFIG.is_file():
            try:
                return json.loads(CAMERA_INTRINSIC_CONFIG.read_text(encoding="utf-8"))
            except Exception:
                pass
        return {}

    def reset(self):
        with getattr(self, "_lock", RLock()):
            cfg = self._load_device_config()
            intrinsic_cfg = self._load_intrinsic_config()
            self.state: Dict[str, Any] = {
                "updated_at": datetime.now().isoformat(timespec="seconds"),
                "coordinate_frame": "world",
                "unit": "mm",
                "axis": {"x": "right", "y": "vehicle_forward", "z": "up"},
                "phase": "IDLE",
                "round": 0,
                "cargo": {},
                # Complete task inventory.  The active ``cargo`` remains the
                # convenient control/UI focus, while this list is the durable
                # scene source for staged, carried and already placed loads.
                "cargo_inventory": [],
                "pallet": {},
                "truck": {
                    "truck_id": "TRUCK-01",
                    "pose": Pose6D(0, 9000, 0, 0, 0, 0).to_dict(),
                    "board_mode": "UNKNOWN",
                    "corners": {},
                    "regions": [],
                    "occupied": [],
                    "available": [],
                    "current_target": {},
                    "remaining_space": {},
                },
                "devices": {},
                "cameras": {},
                "parallel": {
                    "pick": {"status": "IDLE", "message": ""},
                    "radar": {"status": "IDLE", "message": ""},
                },
                "messages": [],
                "results": [],
                "alarm": None,
                "feedback_compensation_mm": {"dx": 0.0, "dy": 0.0, "dz": 0.0},
                "previous_scene_signature": None,
            }
            # 设备与相机父子关系由配置驱动；默认为唯一的插取/装载臂及其挂载相机。
            self.state["devices"]["PLC"] = self._device("PLC", "controller", Pose6D(), "SYSTEM")
            radar_cfg = (cfg.get("radar") or {})
            self.state["devices"]["RADAR"] = self._device("RADAR", "radar", Pose6D.from_any(radar_cfg.get("initial_pose", {"x_mm":2000,"y_mm":-4200,"z_mm":2400,"pitch_deg":-15,"yaw_deg":90})), "WORLD")
            robots = cfg.get("robots") or {
                "PICK_ARM":{"initial_pose":{"x_mm":-3500,"y_mm":1000,"z_mm":1800,"yaw_deg":-90}},
            }
            for rid, rcfg in robots.items():
                self.state["devices"][rid] = self._device(rid, "robot", Pose6D.from_any((rcfg or {}).get("initial_pose")), "WORLD")
                self.state["devices"][rid]["role"] = (rcfg or {}).get("role", "")
            cameras = cfg.get("cameras") or {
                "CAM_PICK":{"parent_robot_id":"PICK_ARM","sensor":"RGB-D","mount_pose":{"x_mm":250,"z_mm":-80,"yaw_deg":180}},
            }
            for cid, ccfg in cameras.items():
                parent = (ccfg or {}).get("parent_robot_id")
                if parent not in self.state["devices"]:
                    raise RuntimeError(f"相机 {cid} 的父机械臂不存在：{parent}")
                sensor = str((ccfg or {}).get("sensor", "RGB")).upper()
                self.state["cameras"][cid] = self._camera(cid, parent, Pose6D.from_any((ccfg or {}).get("mount_pose")), rgbd=("D" in sensor))
                self.state["cameras"][cid]["role"] = (ccfg or {}).get("role", "")
                cal_name = (ccfg or {}).get("calibration_name")
                self.state["cameras"][cid]["calibration_name"] = cal_name
                raw_intr = ((intrinsic_cfg.get("cameras") or {}).get(cal_name) or {}) if cal_name else {}
                K = raw_intr.get("K") or [[600,0,320],[0,600,240],[0,0,1]]
                try:
                    self.state["cameras"][cid]["intrinsics"] = {
                        "K": K,
                        "fx": float(K[0][0]), "fy": float(K[1][1]),
                        "cx": float(K[0][2]), "cy": float(K[1][2]),
                        "distortion": raw_intr.get("distortion") or [0,0,0,0,0],
                        "depth_scale": float(raw_intr.get("depth_scale", 0.001)),
                        "status": raw_intr.get("status", "unknown"),
                    }
                except Exception:
                    pass
            self._update_camera_world_poses_unlocked()

    @staticmethod
    def _device(device_id, kind, pose: Pose6D, frame):
        return {
            "device_id": device_id, "kind": kind, "status": "ONLINE", "task": "IDLE",
            "frame": frame, "pose": pose.to_dict(),
        }

    @staticmethod
    def _camera(camera_id, parent_robot_id, mount_pose: Pose6D, rgbd: bool):
        return {
            "camera_id": camera_id,
            "parent_robot_id": parent_robot_id,
            "status": "READY",
            "task": "IDLE",
            "rgbd": bool(rgbd),
            "mount_pose": mount_pose.to_dict(),
            "world_pose": Pose6D().to_dict(),
            "intrinsics": {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0},
        }

    def _touch(self):
        self.state["updated_at"] = datetime.now().isoformat(timespec="seconds")

    def _update_camera_world_poses_unlocked(self):
        for cam in self.state.get("cameras", {}).values():
            parent = self.state.get("devices", {}).get(cam.get("parent_robot_id"), {})
            parent_pose = Pose6D.from_any(parent.get("pose"))
            mount_pose = Pose6D.from_any(cam.get("mount_pose"))
            cam["world_pose"] = compose_pose(parent_pose, mount_pose).to_dict()

    def _update_attached_cargo_pose_unlocked(self):
        cargo = self.state.get("cargo") or {}
        attachment = cargo.get("attachment") or {}
        parent_id = attachment.get("parent_device_id")
        parent = self.state.get("devices", {}).get(parent_id) if parent_id else None
        if not parent:
            return
        parent_pose = Pose6D.from_any(parent.get("pose"))
        local_pose = Pose6D.from_any(attachment.get("local_pose"))
        pose = compose_pose(parent_pose, local_pose).to_dict()
        cargo["pose"] = pose
        pallet = self.state.get("pallet") or {}
        pallet.update({"pose": deepcopy(pose), "status": "CARRIED", "attached_to": parent_id})
        self.state["pallet"] = pallet
        self._sync_active_cargo_to_inventory_unlocked()

    def _sync_active_cargo_to_inventory_unlocked(self):
        cargo = self.state.get("cargo") or {}
        cargo_id = cargo.get("instance_id")
        if not cargo_id:
            return
        inventory = self.state.setdefault("cargo_inventory", [])
        for index, item in enumerate(inventory):
            if item.get("instance_id") == cargo_id:
                inventory[index] = deepcopy(cargo)
                return
        inventory.append(deepcopy(cargo))

    def update_robot_pose(self, robot_id: str, pose: Pose6D, task: str = ""):
        with self._lock:
            dev = self.state["devices"][robot_id]
            dev["pose"] = Pose6D.from_any(pose).to_dict()
            if task:
                dev["task"] = task
            self._update_camera_world_poses_unlocked()
            self._update_attached_cargo_pose_unlocked()
            self._touch()

    def update_device(self, device_id: str, **fields):
        with self._lock:
            self.state["devices"].setdefault(device_id, {"device_id": device_id, "pose": Pose6D().to_dict()})
            self.state["devices"][device_id].update(fields)
            if device_id.endswith("ARM") and "pose" in fields:
                self._update_camera_world_poses_unlocked()
                self._update_attached_cargo_pose_unlocked()
            self._touch()

    def update_camera(self, camera_id: str, **fields):
        with self._lock:
            self.state["cameras"][camera_id].update(fields)
            # world_pose 仍由机械臂位姿派生，禁止调用方覆盖。
            self._update_camera_world_poses_unlocked()
            self._touch()

    def set_cargo(self, cargo: Dict[str, Any]):
        with self._lock:
            self.state["cargo"] = deepcopy(cargo)
            self._sync_active_cargo_to_inventory_unlocked()
            self._touch()

    def set_cargo_inventory(self, cargos) -> None:
        """Replace the complete task inventory used by the 3-D scene."""
        with self._lock:
            self.state["cargo_inventory"] = [deepcopy(c) for c in (cargos or [])]
            self._sync_active_cargo_to_inventory_unlocked()
            self._touch()

    def attach_cargo(self, parent_device_id: str = "PICK_ARM", local_pose=None) -> Dict[str, Any]:
        """Rigidly bind the current pallet/cargo to a robot after fork pickup."""
        with self._lock:
            parent = self.state.get("devices", {}).get(parent_device_id)
            cargo = self.state.get("cargo") or {}
            if not parent:
                raise KeyError(f"货物绑定机械臂不存在：{parent_device_id}")
            if not cargo:
                raise RuntimeError("当前没有可绑定的货物")
            existing = cargo.get("attachment") or {}
            if existing.get("parent_device_id") == parent_device_id:
                self._update_attached_cargo_pose_unlocked()
                return deepcopy(existing)
            resolved_local_pose = (
                Pose6D.from_any(local_pose).to_dict()
                if local_pose is not None
                else relative_pose(
                    Pose6D.from_any(parent.get("pose")),
                    Pose6D.from_any(cargo.get("pose")),
                ).to_dict()
            )
            attachment = {
                "parent_device_id": parent_device_id,
                "local_pose": resolved_local_pose,
                "attached_at": datetime.now().isoformat(timespec="seconds"),
            }
            cargo["attachment"] = attachment
            cargo["attached_to"] = parent_device_id
            cargo["status"] = "CARRIED"
            self.state["pallet"] = {
                "pose": deepcopy(cargo.get("pose") or {}),
                "status": "CARRIED",
                "attached_to": parent_device_id,
            }
            self._update_attached_cargo_pose_unlocked()
            self._sync_active_cargo_to_inventory_unlocked()
            self._touch()
            return deepcopy(attachment)

    def robot_pose_for_cargo_target(self, parent_device_id: str, cargo_target_pose) -> Dict[str, float]:
        """Calculate the tool pose that puts an attached cargo at the requested pose."""
        with self._lock:
            cargo = self.state.get("cargo") or {}
            attachment = cargo.get("attachment") or {}
            if attachment.get("parent_device_id") != parent_device_id:
                raise RuntimeError(f"当前货物未绑定到 {parent_device_id}，禁止无载荷放置")
            return parent_pose_for_child(
                Pose6D.from_any(cargo_target_pose),
                Pose6D.from_any(attachment.get("local_pose")),
            ).to_dict()

    def set_attached_cargo_world_pose(self, parent_device_id: str, cargo_world_pose) -> Dict[str, Any]:
        """Move the carried load with the telescopic tool while its rail base stays outside."""
        with self._lock:
            cargo = self.state.get("cargo") or {}
            attachment = cargo.get("attachment") or {}
            parent = self.state.get("devices", {}).get(parent_device_id)
            if attachment.get("parent_device_id") != parent_device_id or not parent:
                raise RuntimeError(f"当前货物未绑定到 {parent_device_id}")
            target = Pose6D.from_any(cargo_world_pose)
            attachment["local_pose"] = relative_pose(Pose6D.from_any(parent.get("pose")), target).to_dict()
            cargo["attachment"] = attachment
            self._update_attached_cargo_pose_unlocked()
            self._touch()
            return deepcopy(cargo)

    def detach_cargo(self, final_pose, status: str = "PLACED") -> Dict[str, Any]:
        """Release the current cargo and keep it fixed at its final WORLD pose."""
        with self._lock:
            cargo = self.state.get("cargo") or {}
            cargo["pose"] = Pose6D.from_any(final_pose).to_dict()
            cargo["status"] = str(status)
            cargo["attached_to"] = None
            cargo["attachment"] = None
            self.state["pallet"] = {
                "pose": deepcopy(cargo["pose"]),
                "status": str(status),
                "attached_to": None,
            }
            self._sync_active_cargo_to_inventory_unlocked()
            self._touch()
            return deepcopy(cargo)

    def set_pallet(self, pallet: Dict[str, Any]):
        with self._lock:
            self.state["pallet"] = deepcopy(pallet)
            self._touch()

    def update_truck(self, **fields):
        with self._lock:
            self.state["truck"].update(deepcopy(fields))
            self._touch()

    def set_phase(self, phase: str, round_no: int | None = None):
        with self._lock:
            self.state["phase"] = str(phase)
            if round_no is not None:
                self.state["round"] = int(round_no)
            self._touch()

    def set_parallel(self, branch: str, status: str, message: str = "", result=None):
        with self._lock:
            self.state["parallel"][branch] = {"status": status, "message": message, "result": deepcopy(result)}
            self._touch()

    def add_message(self, source: str, status: str, message: str, data=None):
        with self._lock:
            self.state["messages"].append({
                "time": datetime.now().isoformat(timespec="seconds"),
                "source": source, "status": status, "message": str(message), "data": deepcopy(data),
            })
            self.state["messages"] = self.state["messages"][-300:]
            self._touch()

    def add_result(self, record: Dict[str, Any]):
        with self._lock:
            self.state["results"].append(deepcopy(record))
            self._touch()

    def set_alarm(self, text):
        with self._lock:
            self.state["alarm"] = text
            self._touch()

    def set_feedback_compensation(self, dx=0.0, dy=0.0, dz=0.0):
        with self._lock:
            self.state["feedback_compensation_mm"] = {"dx": float(dx), "dy": float(dy), "dz": float(dz)}
            self._touch()

    def camera_world_pose(self, camera_id: str) -> Dict[str, float]:
        with self._lock:
            self._update_camera_world_poses_unlocked()
            return deepcopy(self.state["cameras"][camera_id]["world_pose"])

    def snapshot(self):
        with self._lock:
            self._update_camera_world_poses_unlocked()
            self._update_attached_cargo_pose_unlocked()
            return deepcopy(self.state)
