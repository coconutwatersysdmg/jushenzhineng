# -*- coding: utf-8 -*-
from __future__ import annotations

from copy import deepcopy
from math import ceil, sqrt
from pathlib import Path
from time import sleep
from uuid import uuid4

from core.geometry import Pose6D
from core.digital_twin_state import DigitalTwinState
from utils.demo_assets import ensure_demo_corner_rgbd, ensure_demo_pre_pick_image, ensure_demo_rgb_image
from .base import PLCAdapter, RobotAdapter, RadarAdapter, ArmCameraAdapter


class MockPLCAdapter(PLCAdapter):
    def __init__(self, twin: DigitalTwinState):
        self.twin = twin
        self.connected = False
        self.commands = {}

    def connect(self):
        self.connected = True
        self.twin.update_device("PLC", status="ONLINE", task="CONNECTED")
        return {"success": True}

    def send_command(self, command: str, payload: dict) -> dict:
        cid = f"CMD-{uuid4().hex[:10].upper()}"
        self.commands[cid] = {"command": command, "payload": deepcopy(payload), "status": "SENT"}
        self.twin.update_device("PLC", task=f"{command} / WAIT_ACK")
        return {"success": True, "command_id": cid, "command": command, "payload": deepcopy(payload)}

    def wait_ack(self, command_id: str, timeout_ms: int = 5000) -> dict:
        sleep(0.01)
        if command_id not in self.commands:
            return {"success": False, "message": "未知 PLC command_id"}
        self.commands[command_id]["status"] = "ACK"
        self.twin.update_device("PLC", task="ACK")
        return {"success": True, "command_id": command_id, "ack": True}

    def send_message(self, module: str, status: str, message: str, data: dict | None = None) -> dict:
        self.twin.add_message(module, status, message, data)
        return {"success": True, "module": module, "status": status, "message": message, "data": data}

    def move_absolute_xyzr(
        self,
        targets,
        speed: float = 30.0,
        timeout_s: float = 60.0,
        soft_limits=None,
    ) -> dict:
        payload = {"targets": dict(targets or {}), "speed": float(speed), "timeout_s": float(timeout_s)}
        self.twin.update_device("PLC", task="ABS_MOVE_MOCK")
        self.twin.add_message("PLC", "SUCCESS", f"模拟绝对定位 {payload['targets']}", payload)
        return {"success": True, "message": f"模拟绝对定位完成：{payload['targets']}", **payload}


class MockRobotAdapter(RobotAdapter):
    def __init__(self, twin: DigitalTwinState, plc: PLCAdapter, config: dict | None = None):
        self.twin = twin
        self.plc = plc
        self.config = dict(config or {})
        self.motion_callback = None
        self.command_emitter = None
        self.world_to_gantry = dict(self.config.get("world_to_gantry") or {})
        self.default_speed = float(self.config.get("default_speed", 30.0))

    def _gantry_xyzr(self, pose: dict):
        mapping = self.world_to_gantry
        if not mapping:
            try:
                from config.external_devices_config import GANTRY
                mapping = dict((GANTRY or {}).get("world_to_gantry") or {})
            except Exception:
                mapping = {}
        if not mapping:
            return None
        try:
            from devices.world_to_gantry import transform_world_to_gantry
            return transform_world_to_gantry(pose, mapping)
        except Exception:
            return None

    def move_tool_world(self, robot_id: str, pose: dict, task: str = "") -> dict:
        # 孪生轨迹仍本地播放；真机写轴改由 plc_console 接收发布指令后执行
        cmd = self.plc.send_command("MOVE_TOOL_WORLD", {"robot_id": robot_id, "pose": pose, "task": task})
        ack = self.plc.wait_ack(cmd["command_id"])
        if not ack.get("success"):
            return ack
        start = Pose6D.from_any((self.twin.snapshot().get("devices", {}).get(robot_id, {}).get("pose") or {}))
        target = Pose6D.from_any(pose)
        distance = sqrt(
            (target.x_mm-start.x_mm)**2 +
            (target.y_mm-start.y_mm)**2 +
            (target.z_mm-start.z_mm)**2
        )
        angle_span = max(
            abs(target.roll_deg-start.roll_deg),
            abs(target.pitch_deg-start.pitch_deg),
            abs(target.yaw_deg-start.yaw_deg),
        )
        # Publish bounded trajectory samples.  With the UI's 420 ms easing,
        # even long head-to-tail travel remains visibly continuous.
        segment_count = max(1, int(ceil(distance/2200.0)), int(ceil(angle_span/45.0)))
        keys = ("x_mm", "y_mm", "z_mm", "roll_deg", "pitch_deg", "yaw_deg")
        start_data, target_data = start.to_dict(), target.to_dict()
        for segment in range(1, segment_count+1):
            fraction = segment/segment_count
            sample = {
                key: float(start_data[key]) + (float(target_data[key])-float(start_data[key]))*fraction
                for key in keys
            }
            self.twin.update_robot_pose(robot_id, Pose6D.from_any(sample), task=task or "MOVING")
            if callable(self.motion_callback):
                self.motion_callback(robot_id, deepcopy(sample), task or "MOVING")
        gantry_xyzr = self._gantry_xyzr(target.to_dict())
        if callable(self.command_emitter):
            self.command_emitter({
                "robot_id": robot_id,
                "world_pose": target.to_dict(),
                "task": task or "MOVE_TOOL_WORLD",
                "gantry_xyzr": gantry_xyzr,
                "speed": self.default_speed,
            })
        return {
            "success": True, "robot_id": robot_id, "pose": target.to_dict(),
            "trajectory_segments": segment_count,
            "gantry_xyzr": gantry_xyzr,
            "export_only": True,
            "message": f"{robot_id} 已连续运动到目标位（指令已发布）",
        }

    def retract(self, robot_id: str) -> dict:
        target = {
            "x_mm": -3500.0, "y_mm": 1000.0, "z_mm": 1800.0,
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": -90.0,
        }
        return self.move_tool_world(robot_id, target, task="RETRACT")

    def fork_pallet(self, pallet_result: dict) -> dict:
        # TODO: 与PLC交互 — 下发插取托盘动作给 PLC
        cmd = self.plc.send_command("FORK_PALLET", pallet_result)
        # TODO: 与PLC交互 — 等 PLC 确认插取完成
        ack = self.plc.wait_ack(cmd["command_id"])
        if ack.get("success"):
            self.twin.update_device("PICK_ARM", task="PALLET_PICKED")
        return {"success": bool(ack.get("success")), "message": "托盘插取完成" if ack.get("success") else "托盘插取失败"}

    def place(self, cargo: dict, target: dict) -> dict:
        # TODO: 与PLC交互 — 下发放货动作给 PLC
        cmd = self.plc.send_command("PLACE_CARGO", {"cargo": cargo, "target": target})
        # TODO: 与PLC交互 — 等 PLC 确认放货完成
        ack = self.plc.wait_ack(cmd["command_id"])
        if ack.get("success"):
            pose = target.get("final_world_pose") or {}
            self.twin.detach_cargo(pose, status="PLACED")
        return {"success": bool(ack.get("success")), "message": "货物已放置" if ack.get("success") else "放置失败", "target": target}


class MockRadarAdapter(RadarAdapter):
    def __init__(self, twin: DigitalTwinState):
        self.twin = twin
        self.pcd_path = ""
        self.force_success = True

    def set_point_cloud_path(self, path: str): self.pcd_path = str(path or "")

    def probe(self) -> dict:
        self.twin.update_device("RADAR", status="ONLINE", task="MOCK_READY")
        return {"success": True, "device_id": "RADAR", "message": "Mock 雷达就绪"}

    def locate_truck(self, cargo: dict) -> dict:
        self.twin.update_device("RADAR", task="LOCATING")
        if not self.force_success:
            self.twin.update_device("RADAR", status="FAILED", task="LOCATE_FAILED")
            return {"success": False, "message": "雷达找车失败"}
        p = Path(str(cargo.get("point_cloud_path") or self.pcd_path or ""))
        if p.is_file():
            return {"success": True, "pcd_path": str(p.resolve()), "message": "雷达取得车辆点云"}
        # 无 PCD 时提供4/6点联调数据；这些点只用于相机搜索，不决定最终板型。
        if int((cargo or {}).get("radar_demo_corner_count", 4) or 4) == 6:
            return {
                "success": True, "demo": True, "message": "雷达联调：返回 6 个粗定位角点（仅用于相机搜索）",
                "coordinate_frame": "world", "coordinate_unit": "mm",
                "corner_ids": ["P1","P2","P3","P4","P5","P6"],
                "world_points": {
                    "P1":{"x":-1250,"y":3000,"z":1650}, "P2":{"x":1250,"y":3000,"z":1650},
                    "P3":{"x":-1250,"y":7380,"z":1650}, "P4":{"x":1250,"y":7380,"z":1650},
                    "P5":{"x":-1250,"y":15000,"z":1450}, "P6":{"x":1250,"y":15000,"z":1450}
                }
            }
        return {
            "success": True, "demo": True, "message": "雷达联调：返回 4 个粗定位角点（仅用于相机搜索）",
            "coordinate_frame": "world", "coordinate_unit": "mm",
            "corner_ids": ["P1", "P2", "P3", "P4"],
            "world_points": {
                "P1": {"x": -1250, "y": 3000, "z": 1450},
                "P2": {"x":  1250, "y": 3000, "z": 1450},
                "P3": {"x": -1250, "y": 15000, "z": 1450},
                "P4": {"x":  1250, "y": 15000, "z": 1450}
            }
        }


class MockArmCameraAdapter(ArmCameraAdapter):
    def __init__(self, twin: DigitalTwinState):
        self.twin = twin
        self.inputs = {
            "CAM_PICK": {"rgb": "", "depth": ""},
        }
        self.tagged_images = {}
        self.tagged_depths = {}
        self.demo_enabled = False

    def set_demo_enabled(self, enabled: bool):
        self.demo_enabled = bool(enabled)

    def probe(self) -> dict:
        self.twin.update_device("CAM_PICK", status="ONLINE", task="MOCK_READY")
        return {"success": True, "device_id": "CAM_PICK", "message": "Mock 相机就绪"}

    def set_rgbd(self, rgb: str, depth: str):
        self.inputs["CAM_PICK"] = {"rgb": str(rgb or ""), "depth": str(depth or "")}

    def set_tagged_image(self, tag: str, path: str):
        self.tagged_images[str(tag)] = str(path or "")

    def set_tagged_depth(self, tag: str, path: str):
        self.tagged_depths[str(tag)] = str(path or "")

    @staticmethod
    def _file(path):
        p = Path(str(path or "")).expanduser()
        return p.resolve() if p.is_file() else None

    def capture_rgbd(self, camera_id: str, tag: str = "") -> dict:
        self.twin.update_camera(camera_id, task=f"CAPTURE_RGBD:{tag}", status="CAPTURE")
        info = self.inputs.get(camera_id, {})
        rgb = self._file(self.tagged_images.get(str(tag), "")) or self._file(info.get("rgb"))
        depth = self._file(self.tagged_depths.get(str(tag), "")) or self._file(info.get("depth"))
        demo = False
        if self.demo_enabled and (not rgb or not depth) and (
            str(tag) in {f"P{i}" for i in range(1, 7)} or str(tag) in {"CORNER_TAIL", "CORNER_HEAD"}
        ):
            rgb, depth = ensure_demo_corner_rgbd(tag)
            demo = True
        return {
            "success": bool(rgb and depth), "camera_id": camera_id, "tag": tag,
            "camera_world_pose": self.twin.camera_world_pose(camera_id),
            "rgb_path": str(rgb) if rgb else "", "depth_path": str(depth) if depth else "",
            "demo": demo, "source": "generated_demo_rgbd" if demo else "camera_or_debug_input",
        }

    def capture_rgb(self, camera_id: str, tag: str = "") -> dict:
        self.twin.update_camera(camera_id, task=f"CAPTURE:{tag}", status="CAPTURE")
        p = self._file(self.tagged_images.get(str(tag), ""))
        demo = False
        if self.demo_enabled and p is None:
            p = ensure_demo_pre_pick_image() if str(tag) in {"pre_pick_offset", "post_place_offset"} else ensure_demo_rgb_image(tag)
            demo = True
        return {
            "success": bool(p), "camera_id": camera_id, "tag": tag,
            "camera_world_pose": self.twin.camera_world_pose(camera_id),
            "image_path": str(p) if p else "",
            "demo": demo, "source": "generated_demo_image" if demo else "camera_or_debug_input",
        }
