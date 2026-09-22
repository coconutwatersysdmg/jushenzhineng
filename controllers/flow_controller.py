# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError
from copy import deepcopy
from datetime import datetime
from math import ceil, sqrt
from pathlib import Path
from time import perf_counter
from typing import Any, Dict, List, Mapping
from uuid import uuid4

from core.digital_twin_state import DigitalTwinState
from core.geometry import Pose6D, parent_pose_for_child
from devices.device_factory import create_device_adapters
from services.algorithm_facade import AlgorithmFacade
from services.lab_camera_visit_planner import LabCameraVisitPlanner, merge_lab_corner_points
from services.sensor_calibration_service import SensorCalibrationService
from services.camera_corner_world_service import CameraCornerWorldService
from services.camera_board_geometry_service import CameraBoardGeometryService
from services.space_manager import SpaceManager
from services.vision_measurement_interfaces import NeighborPalletPoseService, PalletBoardRegionDeviationService
from services.placement_compensation_service import PlacementCompensationService
from services.two_face_observation_service import TwoFaceObservationService
from services.dynamic_monitoring_service import DynamicMonitoringService
from services.module_evidence_service import ModuleEvidenceService
from services.vehicle_database_service import create_vehicle_database_service
from services.plc_motion_publisher import PlcMotionPublisher
from utils.demo_assets import ensure_demo_pre_pick_image
from config.system_config import get_system_config
from config.external_devices_config import get_device_layout_config
import config.feature_switches as feature_switches

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULT_FILE = PROJECT_ROOT / "workdir" / "overall_results.json"
STATE_FILE = PROJECT_ROOT / "workdir" / "twin_state.json"
# TODO: 与PLC交互 — 外接设备统一配置：config/external_devices_config.py
GANTRY_LEFT_X_MM = -3500.0
GANTRY_RIGHT_X_MM = 3500.0
GANTRY_CLEARANCE_Z_MM = 4800.0


class FlowController:
    """v8 首轮建图 + 后续循环装载控制器。

    首轮：2 -> 3(并行3.1/3.2) -> 4 -> 5 -> 6 -> 7 -> 8.1 -> 8.2 -> 8.3 -> 9 -> 10.1 -> 10.2 -> 11 -> 12
    后续：2 -> 3.1 -> 8.2 -> 8.3 -> 9 -> 10.1 -> 10.2 -> 11 -> 12

    8.3 是根据用户“下一轮包含8.3”的描述明确出来的目标锁定节点：
    它不重新建图，只把持久可用空间、历史反馈与8.2临近托盘补偿合成为本轮最终目标。
    """

    FIRST_STEPS = [
        ("DEVICE_CHECK", "1. 外接设备连接检查（PLC / 雷达 / 相机）"),
        ("PRE_PICK_OFFSET", "2. 货物-托盘偏差分析 / S-F message"),
        ("PARALLEL_LOCATE", "3. 并行：3.1插孔插取 + 3.2雷达找车"),
        ("RADAR_TO_CAMERA", "4. 雷达粗点 -> PLC -> 机械臂挂载相机"),
        ("CAPTURE_CORNERS", "5. 左外侧单次通过：车尾/车头各1组 RGB-D"),
        ("CORNER_RECOGNITION", "6. 角点 .pt 像素识别 / 人工审核矫正"),
        ("CAMERA_TO_WORLD", "7. 像素+深度+动态外参 -> WORLD"),
        ("INITIAL_SPACE_PLAN", "8.1 相机判断平/高低板 + 两列×1.2m盲码规划"),
        ("NEIGHBOR_POSE", "8.2 拍临近托盘姿态 / 计算当前补偿"),
        ("TARGET_CONFIRM", "8.3 可用空间状态确认 / 锁定本轮目标"),
        ("PRE_PLACE_MONITOR", "8.4 放置前 RGB-D 动态监测"),
        ("PLACE", "9. 计算最终放置点 / PLC / 放货"),
        ("POST_PLACE_BOTTOM", "10.0 放置后底层托盘 RGB-D 检测"),
        ("POST_REGION", "10.1 托盘-车板区域偏差 + 货物两面观测"),
        ("POST_CARGO_OFFSET", "10.2 托盘-货物偏差 / 补偿"),
        ("FEEDBACK", "11. 偏差回PLC / 修正下一托盘 / 更新空间"),
        ("RETURN", "12. 机械臂返回 / 下一轮"),
    ]
    REPEAT_STEPS = [
        ("PRE_PICK_OFFSET", "2. 货物-托盘偏差分析 / S-F message"),
        ("PICK_ONLY", "3.1 机械臂找插孔并插取（跳过雷达）"),
        ("NEIGHBOR_POSE", "8.2 拍临近托盘姿态 / 计算当前补偿"),
        ("TARGET_CONFIRM", "8.3 复用首轮车板模型 / 锁定下一可用区域"),
        ("PRE_PLACE_MONITOR", "8.4 放置前 RGB-D 动态监测"),
        ("PLACE", "9. 计算最终放置点 / PLC / 放货"),
        ("POST_PLACE_BOTTOM", "10.0 放置后底层托盘 RGB-D 检测"),
        ("POST_REGION", "10.1 托盘-车板区域偏差 + 货物两面观测"),
        ("POST_CARGO_OFFSET", "10.2 托盘-货物偏差 / 补偿"),
        ("FEEDBACK", "11. 偏差回PLC / 修正下一托盘 / 更新空间"),
        ("RETURN", "12. 机械臂返回 / 下一轮"),
    ]
    # 实验室模式：设备检查 → 插孔∥雷达 → 精定位空壳 → 手动放货后俯拍校验（可多轮）
    LAB_FIRST_STEPS = [
        ("DEVICE_CHECK", "1. 外接设备连接检查（PLC / 雷达 / 相机）"),
        ("LAB_SENSE", "2. 相机插孔识别 ∥ 雷达底板四角粗定位"),
        ("LAB_CORNER_SHELL", "3. 相机精定位与轮廓确认（实验室空壳）"),
        ("LAB_PLACE_VERIFY", "4. 手动放货后俯拍位置校验"),
    ]
    LAB_REPEAT_STEPS = [
        ("LAB_SENSE", "2. 相机插孔识别（复用首轮雷达车板）"),
        ("LAB_CORNER_SHELL", "3. 相机精定位与轮廓确认（实验室空壳）"),
        ("LAB_PLACE_VERIFY", "4. 手动放货后俯拍位置校验"),
    ]

    @staticmethod
    def is_lab_profile() -> bool:
        return str(feature_switches.RUN_PROFILE or "").strip().lower() == "lab"

    def __init__(self, twin=None, plc=None, robot=None, radar=None, camera=None, algorithms=None):
        self.device_config = get_device_layout_config()
        self.system_config = get_system_config()
        self.allow_demo = bool((self.system_config.get("runtime") or {}).get("allow_demo_device_data", True))
        self.twin = twin or DigitalTwinState()
        if plc is None or robot is None or radar is None or camera is None:
            mode, built_plc, built_robot, built_radar, built_camera = create_device_adapters(
                self.twin, self.system_config
            )
            self.device_mode = mode
            self.plc = plc or built_plc
            self.robot = robot or built_robot
            self.radar = radar or built_radar
            self.camera = camera or built_camera
        else:
            self.device_mode = str((self.system_config.get("runtime") or {}).get("device_mode") or "mock")
            self.plc = plc
            self.robot = robot
            self.radar = radar
            self.camera = camera
        # Real hardware mode must not synthesize demo frames behind the scenes.
        if str(self.device_mode).lower() == "real":
            self.allow_demo = False
        if hasattr(self.camera, "set_demo_enabled"):
            self.camera.set_demo_enabled(self.allow_demo)
        self.algorithms = algorithms or AlgorithmFacade()
        self.calibration = SensorCalibrationService()
        c_cfg = self.system_config.get("camera_corner_world") or {}
        self.corner_world = CameraCornerWorldService(self.calibration, depth_window=int(c_cfg.get("depth_sample_window", 5)))
        g_cfg = self.system_config.get("camera_board_geometry") or {}
        self.board_geometry = CameraBoardGeometryService(
            row_length_mm=float(g_cfg.get("row_length_mm", 1200.0)),
            high_low_height_threshold_mm=float(g_cfg.get("high_low_height_threshold_mm", 80.0)),
            max_borrow_mm=float(g_cfg.get("max_borrow_mm", 650.0)),
            min_second_section_remaining_mm=float(g_cfg.get("min_second_section_remaining_mm", 600.0)),
        )
        self.space = SpaceManager()
        self.neighbor_pose = NeighborPalletPoseService()
        self.region_deviation = PalletBoardRegionDeviationService()
        self.placement_comp = PlacementCompensationService()
        self.dynamic_monitoring = DynamicMonitoringService()
        self.module_evidence = ModuleEvidenceService()
        db_cfg=self.system_config.get("database") or {}
        self.vehicle_db=create_vehicle_database_service(db_cfg)
        self.loading_session_id=""; self._db_message_cursor=0
        roles = self.device_config.get("camera_roles") or {}
        self.two_face = TwoFaceObservationService(
            self.twin, self.robot, self.camera,
            robot_id="PICK_ARM",
            camera_id=roles.get("observe", "CAM_PICK"),
        )
        self.queue: List[Dict[str, Any]] = []
        self.completed: List[Dict[str, Any]] = []
        self.round_index = 0
        self.step_index = 0
        self.running = False
        self.finished = False
        self.round_data: Dict[str, Any] = {}
        self.results: List[Dict[str, Any]] = []
        self.debug_inputs: Dict[str, Any] = {}
        self.state_listener = None
        self.truck_initialized = False
        self.corner_review_callback = None
        review_cfg = self.system_config.get("corner_review") or {}
        self.corner_review_enabled = bool(review_cfg.get("enabled", True))
        self.corner_review_auto_accept_demo = bool(review_cfg.get("auto_accept_demo", False))
        # TODO: 与PLC交互 — 启动时连接 PLC（mock 假连；real 用 config/system_config.py 的 devices.plc）
        self.plc.connect()
        if hasattr(self.robot, "motion_callback"):
            self.robot.motion_callback = self._notify_motion
        if hasattr(self.robot, "command_emitter"):
            self.robot.command_emitter = self._on_robot_motion_command
        self.plc_publisher = PlcMotionPublisher()
        self.plc_command_listener = None
        self.auto_push_plc_commands = False
        self.twin.add_message("CALIBRATION", "INFO", "已加载用户标定配置；运动相机使用机械臂实时位姿×安装外参", self.calibration.diagnostic_summary())
        self.set_debug_inputs(self.module_evidence.example_debug_inputs())
        self.twin.add_message("MODULE_INPUTS", "INFO", "联调输入源已就绪；模型输入/输出只在流程到达并完成调用后显示", {"module_count":len(self.module_evidence.snapshot())})
        self.twin.add_message(
            "PLC_MOTION",
            "INFO",
            "运动坐标确认后优先经主系统已连接的 PLC 直接写轴；独立 plc_console 仅作备用通道",
            {"server": "jushenzhineng-plc-console"},
        )

    def set_state_listener(self, listener):
        """Receive intermediate motion snapshots so the UI can animate real step actions."""
        self.state_listener = listener if callable(listener) else None

    def set_plc_command_listener(self, listener):
        self.plc_command_listener = listener if callable(listener) else None
        self.plc_publisher.set_listener(self.plc_command_listener)

    def set_auto_push_plc(self, enabled: bool) -> None:
        self.auto_push_plc_commands = bool(enabled)

    def _resolve_motion_xyzr(self, cmd: Mapping[str, Any]) -> dict[str, float]:
        raw = cmd.get("gantry_xyzr")
        if isinstance(raw, Mapping) and all(k in raw for k in ("X", "Y", "Z", "R")):
            return {k: float(raw[k]) for k in ("X", "Y", "Z", "R")}
        from devices.world_to_gantry import WorldToGantryError, transform_world_to_gantry

        mapping = dict(getattr(self.robot, "world_to_gantry", None) or {})
        if not mapping:
            from config.external_devices_config import GANTRY

            mapping = dict((GANTRY or {}).get("world_to_gantry") or {})
        try:
            return transform_world_to_gantry(cmd.get("world_pose") or {}, mapping)
        except WorldToGantryError as exc:
            raise RuntimeError(f"无法换算 XYZR：{exc}") from exc

    def _execute_motion_on_local_plc(self, cmd: Mapping[str, Any]) -> dict[str, Any] | None:
        """主系统 PLC 已连接时直接写轴；无法本地执行时返回 None（交由控制台备用通道）。"""
        plc = self.plc
        if not getattr(plc, "connected", False):
            return None

        try:
            targets = self._resolve_motion_xyzr(cmd)
        except Exception as exc:
            return {
                "success": False,
                "message": str(exc),
                "cmd_id": cmd.get("cmd_id"),
                "channel": "local_plc",
            }

        try:
            from config.external_devices_config import GANTRY
        except Exception:
            GANTRY = {}
        speed = float(cmd.get("speed") or (GANTRY or {}).get("default_speed") or 30.0)
        timeout_s = float((GANTRY or {}).get("move_timeout_s") or 60.0)
        soft_limits = dict((GANTRY or {}).get("soft_limits") or {})

        if hasattr(plc, "move_absolute_xyzr"):
            result = plc.move_absolute_xyzr(
                targets,
                speed=speed,
                timeout_s=timeout_s,
                soft_limits=soft_limits or None,
            )
            return {
                "success": bool(result.get("success")),
                "accepted": bool(result.get("success")),
                "message": str(
                    result.get("message")
                    or ("本地 PLC 绝对定位完成" if result.get("success") else "本地 PLC 写轴失败")
                ),
                "cmd_id": cmd.get("cmd_id"),
                "channel": "local_plc",
                "targets": targets,
                "result": result,
            }

        # mock / 仅状态通道：记账成功，不强制依赖独立控制台
        if hasattr(plc, "send_command"):
            plc.send_command(
                "MOVE_ABSOLUTE_XYZR",
                {"targets": targets, "speed": speed, "cmd_id": cmd.get("cmd_id")},
            )
        return {
            "success": True,
            "accepted": True,
            "message": f"已记账下发（当前 PLC 适配器无真机写轴）：{targets}",
            "cmd_id": cmd.get("cmd_id"),
            "channel": "local_plc_bookkeeping",
            "targets": targets,
        }

    def push_last_plc_command(self) -> dict:
        """确认下发：优先经主系统已连接 PLC 直接写轴；否则再尝试独立 plc_console。"""
        cmd = self.plc_publisher.last_command
        if not cmd:
            return {"success": False, "message": "没有可下发的运动指令"}

        local = self._execute_motion_on_local_plc(cmd)
        if local is not None:
            return local

        remote = self.plc_publisher.push(cmd)
        if remote.get("success"):
            return {**remote, "channel": "plc_console"}
        return {
            "success": False,
            "message": (
                "主系统 PLC 未连接，且独立控制台不可用："
                f"{remote.get('message') or '未知错误'}。"
                "请先完成设备检查中的 PLC 连接，或启动 plc_console 作为备用。"
            ),
            "cmd_id": cmd.get("cmd_id"),
            "channel": "none",
            "console": remote,
        }

    def _on_robot_motion_command(self, payload: dict):
        """机器人适配器终点回调：落盘 + 通知 UI；按开关决定是否实时推送。"""
        pose = dict(payload.get("world_pose") or {})
        cmd = self.plc_publisher.build(
            robot_id=str(payload.get("robot_id") or "PICK_ARM"),
            world_pose=pose,
            task=str(payload.get("task") or ""),
            step=str(self.current_step[0] if self.running else ""),
            round_index=int(self.round_index),
            gantry_xyzr=payload.get("gantry_xyzr"),
            speed=float(payload.get("speed") or 30.0),
            extra={
                k: v
                for k, v in dict(payload).items()
                if k not in {"robot_id", "world_pose", "task", "gantry_xyzr", "speed"}
            }
            or None,
        )
        result = self.plc_publisher.publish_local(cmd)
        # 是否实时推送由主界面决定（弹窗确认 / 自动运行勾选），此处只落盘并通知 UI
        if self.auto_push_plc_commands:
            # 标记给 UI：本条在展示时可自动推送
            result = {**result, "_ui_auto_push": True}
        return {"command": result, "pushed": False, "push_result": None}

    def _notify_motion(self, robot_id: str, pose: Dict[str, Any], task: str):
        if self.loading_session_id:
            self.vehicle_db.record_motion(self.loading_session_id, robot_id, pose, task)
        if self.state_listener:
            self.state_listener(self.snapshot(), {"robot_id": robot_id, "pose": deepcopy(pose), "task": str(task)})

    def _move_attached_cargo_world(self, target_pose: Dict[str, Any], task: str) -> Dict[str, Any]:
        """Animate telescopic load motion while the arm base remains on the exterior rail."""
        snapshot=self.twin.snapshot(); cargo=snapshot.get("cargo") or {}
        start=Pose6D.from_any(cargo.get("pose")); target=Pose6D.from_any(target_pose)
        distance=sqrt((target.x_mm-start.x_mm)**2+(target.y_mm-start.y_mm)**2+(target.z_mm-start.z_mm)**2)
        segments=max(1,int(ceil(distance/900.0)))
        start_data,target_data=start.to_dict(),target.to_dict()
        keys=("x_mm","y_mm","z_mm","roll_deg","pitch_deg","yaw_deg")
        self.twin.update_device("PICK_ARM",task=task)
        for segment in range(1,segments+1):
            fraction=segment/segments
            pose={key:float(start_data[key])+(float(target_data[key])-float(start_data[key]))*fraction for key in keys}
            self.twin.set_attached_cargo_world_pose("PICK_ARM",pose)
            robot_pose=((self.twin.snapshot().get("devices") or {}).get("PICK_ARM") or {}).get("pose") or {}
            self._notify_motion("PICK_ARM",robot_pose,task)
        # 伸缩放货也发布终点指令（WORLD=货物目标；臂在外侧轨）
        self._on_robot_motion_command({
            "robot_id": "PICK_ARM",
            "world_pose": robot_pose,
            "task": task,
            "gantry_xyzr": None,
            "speed": 30.0,
            "cargo_world_pose": target.to_dict(),
        })
        return {"success":True,"task":task,"trajectory_segments":segments,"cargo_pose":target.to_dict()}

    @staticmethod
    def _load_json(path: Path):
        try:
            return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
        except Exception:
            return {}

    def _camera_for_role(self, role: str, default: str):
        return str((self.device_config.get("camera_roles") or {}).get(role, default))

    @property
    def is_first_round(self): return self.round_index == 0
    @property
    def steps(self):
        if self.is_lab_profile():
            return self.LAB_FIRST_STEPS if self.is_first_round else self.LAB_REPEAT_STEPS
        return self.FIRST_STEPS if self.is_first_round else self.REPEAT_STEPS
    @property
    def current_step(self):
        done_msg = "实验室本轮/全部完成" if self.is_lab_profile() else "全部货物装载完成"
        if self.finished:
            return ("DONE", done_msg)
        if self.step_index >= len(self.steps):
            return ("DONE", done_msg)
        return self.steps[self.step_index]
    @property
    def current_cargo(self): return self.queue[self.round_index] if 0 <= self.round_index < len(self.queue) else None

    def set_plan(self, items):
        self.queue=[]; seq=0
        for item in items:
            qty=max(1,int(item.get("quantity",1)))
            for unit in range(1,qty+1):
                seq+=1; c=deepcopy(item); c["unit_index"]=unit; c["sequence"]=seq
                c["instance_id"]=f"{c.get('cargo_code','CARGO')}-{unit:03d}-{seq:03d}"
                inventory_base=c.get("inventory_id") or c.get("stock_id")
                c["inventory_id"]=str(f"{inventory_base}-{unit:03d}" if inventory_base and qty>1 else (inventory_base or f"STOCK-{uuid4().hex[:12].upper()}"))
                self.queue.append(c)
        # All task cargo exists from the initial scene. Pickup and the one-pass
        # camera route start at the left tail. Target-side selection begins only
        # after the placement target has been locked.
        # Explicit task/recognition poses remain authoritative when supplied.
        for index, cargo in enumerate(self.queue):
            cargo.setdefault("pose", {
                "x_mm": -2400.0,
                "y_mm": 1000.0 - index * max(1450.0, float(cargo.get("width_mm", 1000) or 1000) + 350.0),
                "z_mm": 0.0,
                "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": -90.0,
            })
            cargo["status"] = "STAGED"
            cargo["attached_to"] = None
            cargo["attachment"] = None
        self.reset(keep_plan=True)
        self.vehicle_db.register_inventory(self.queue)

    def set_debug_inputs(self, values: Dict[str, Any]):
        self.debug_inputs=deepcopy(values or {})
        self._apply_live_radar_policy()
        self._apply_live_camera_policy()
        if hasattr(self.radar,"set_point_cloud_path"):
            self.radar.set_point_cloud_path(self.debug_inputs.get("point_cloud_path", ""))
        pallet_rgb=self.debug_inputs.get("pallet_rgb", ""); pallet_depth=self.debug_inputs.get("pallet_depth", "")
        if pallet_rgb and hasattr(self.camera,"set_tagged_image"):
            self.camera.set_tagged_image("pallet_hole",pallet_rgb)
        if pallet_depth and hasattr(self.camera,"set_tagged_depth"):
            self.camera.set_tagged_depth("pallet_hole",pallet_depth)
        for tag,path in (self.debug_inputs.get("images") or {}).items():
            if hasattr(self.camera,"set_tagged_image"): self.camera.set_tagged_image(tag,path)
        for tag,path in (self.debug_inputs.get("corner_images") or {}).items():
            if hasattr(self.camera,"set_tagged_image"): self.camera.set_tagged_image(tag,path)
        for tag,path in (self.debug_inputs.get("corner_depths") or {}).items():
            if hasattr(self.camera,"set_tagged_depth"): self.camera.set_tagged_depth(tag,path)
        for tag,path in (self.debug_inputs.get("depths") or {}).items():
            if hasattr(self.camera,"set_tagged_depth"): self.camera.set_tagged_depth(tag,path)
        # 真机策略可能清空了部分 tag；再刷一遍，避免上面循环把空路径漏掉。
        self._reapply_cleared_live_camera_tags()

    def _use_live_radar(self) -> bool:
        if str(getattr(self, "device_mode", "mock")).lower() != "real":
            return False
        livox = (self.system_config.get("devices") or {}).get("livox") or {}
        return bool(livox.get("use_live_capture", True))

    def _apply_live_radar_policy(self):
        """真机 + use_live_capture：丢掉离线示例 PCD，强制走 Livox 实采。"""
        if not self._use_live_radar():
            return
        self.debug_inputs["point_cloud_example_only"] = False
        pcd = str(self.debug_inputs.get("point_cloud_path") or "").replace("\\", "/")
        # 默认联调 PCD 在 examples/ 下；清掉后 RealLivox 才会 capture_once
        if (not pcd) or ("/examples/" in f"/{pcd}") or ("example" in pcd.lower()):
            self.debug_inputs["point_cloud_path"] = ""
        self.twin.add_message(
            "RADAR",
            "INFO",
            "已切换真雷达实采：忽略离线示例 PCD，将调用 Livox Mid360 采集",
            {"use_live_capture": True, "host_config": "config/external_devices_config.py → LIVOX.host_ip"},
        )

    def _live_camera_force_tags(self) -> tuple[str, ...]:
        # 实验室 / 真实：这些步骤必须实拍，不允许调试预填示例图短路。
        return ("pallet_hole", "pre_pick_offset", "lab_place_verify")

    def _apply_live_camera_policy(self):
        """真机：第2步等偏移检测丢掉调试预填 JPG，强制走相机实拍。"""
        if str(getattr(self, "device_mode", "mock")).lower() != "real":
            return
        images = self.debug_inputs.setdefault("images", {})
        depths = self.debug_inputs.setdefault("depths", {})
        cleared = []
        for tag in self._live_camera_force_tags():
            had_offline_input = False
            if str(images.get(tag) or "").strip():
                images[tag] = ""
                had_offline_input = True
            if str(depths.get(tag) or "").strip():
                depths[tag] = ""
                had_offline_input = True
            if tag == "pallet_hole":
                if str(self.debug_inputs.get("pallet_rgb") or "").strip():
                    self.debug_inputs["pallet_rgb"] = ""
                    had_offline_input = True
                if str(self.debug_inputs.get("pallet_depth") or "").strip():
                    self.debug_inputs["pallet_depth"] = ""
                    had_offline_input = True
            if had_offline_input:
                cleared.append(tag)
        if cleared:
            self.twin.add_message(
                "CAMERA",
                "INFO",
                "真机模式：已忽略调试示例图，步骤将实拍：" + ", ".join(cleared),
                {"cleared_tags": cleared, "capture_dir": "workdir/camera_captures"},
            )

    def _reapply_cleared_live_camera_tags(self):
        if str(getattr(self, "device_mode", "mock")).lower() != "real":
            return
        for tag in self._live_camera_force_tags():
            if hasattr(self.camera, "set_tagged_image"):
                self.camera.set_tagged_image(tag, "")
            if hasattr(self.camera, "set_tagged_depth"):
                self.camera.set_tagged_depth(tag, "")

    def default_debug_inputs(self):
        return self.module_evidence.example_debug_inputs()

    def _evidence(self, module_id: str, inputs: Dict[str, Any], output: Any, started: float,
                  model_invoked: bool = False, note: str = "", status: str | None = None):
        final_status=status or ("SUCCESS" if isinstance(output,Mapping) and output.get("success",True) else "FAILED")
        evidence=self.module_evidence.record(
            module_id,inputs,output,(perf_counter()-started)*1000.0,
            status=final_status,model_invoked=model_invoked,used_by_flow=True,note=note,
        )
        if self.loading_session_id:
            self.vehicle_db.record_module_run(
                self.loading_session_id,evidence,
                str((self.current_cargo or {}).get("instance_id") or ""),
            )
        return evidence

    def reset(self, keep_plan=False):
        if getattr(self,"loading_session_id","") and not self.finished:
            self.vehicle_db.finish_session(self.loading_session_id,"RESET","用户重置或重新加载装载计划")
        q=deepcopy(self.queue) if keep_plan else []
        self.twin.reset(); self.queue=q; self.completed=[]; self.round_index=0; self.step_index=0
        self.running=False; self.finished=False; self.round_data={}; self.results=[]; self.truck_initialized=False
        self.space.reset()
        self.loading_session_id=""; self._db_message_cursor=0
        self.twin.set_cargo_inventory(self.queue)
        self._save()

    def start(self):
        if not self.queue: raise RuntimeError("装载计划为空")
        if not self.loading_session_id:
            truck=self.twin.snapshot().get("truck") or {}
            first=self.queue[0] if self.queue else {}
            task_code=str(first.get("task_code") or first.get("task_id") or "")
            self.loading_session_id=self.vehicle_db.start_session(truck,self.queue,task_code)
        self.running=True; self.finished=False; self._load_cargo_to_twin(); self._persist_database_snapshot(); return self.snapshot()

    def _load_cargo_to_twin(self):
        cargo_id=(self.current_cargo or {}).get("instance_id")
        inventory=self.twin.snapshot().get("cargo_inventory") or []
        c=deepcopy(next((item for item in inventory if item.get("instance_id")==cargo_id),self.current_cargo or {}))
        c["status"]="CURRENT"; self.twin.set_cargo(c); self.twin.set_phase(self.current_step[1],self.round_index+1)

    def _record(self, code, name, status, message, data=None):
        rec={"time":datetime.now().isoformat(timespec="seconds"),"round":self.round_index+1,"step_code":code,"step_name":name,"status":status,"cargo_id":(self.current_cargo or {}).get("instance_id"),"message":str(message),"data":deepcopy(data)}
        self.results.append(rec); self.twin.add_result(rec); self.twin.add_message(code,status.upper(),message,data)
        if self.loading_session_id:
            self.vehicle_db.record_step(self.loading_session_id,rec)
            twin=self.twin.snapshot()
            if code=="INITIAL_SPACE_PLAN" and isinstance(data,Mapping):
                self.vehicle_db.record_board_snapshot(self.loading_session_id,self.round_index+1,twin.get("truck") or {},data.get("geometry") or {})
            if code=="PLACE" and status=="success" and isinstance(data,Mapping):
                self.vehicle_db.record_placement(self.loading_session_id,self.round_index+1,rec.get("cargo_id") or "",data)
            self._persist_database_snapshot(twin)
        self._save(); return rec

    def _persist_database_snapshot(self,twin=None):
        if not self.loading_session_id: return
        twin=twin or self.twin.snapshot()
        self.vehicle_db.sync_state(self.loading_session_id,twin)
        messages=list(twin.get("messages") or [])
        if self._db_message_cursor>len(messages): self._db_message_cursor=0
        self.vehicle_db.record_messages(self.loading_session_id,messages[self._db_message_cursor:])
        self._db_message_cursor=len(messages)

    def _capture_rgb(self, camera_id: str, tag: str, required=True):
        cap=self.camera.capture_rgb(camera_id,tag=tag)
        # 真机适配器常用 rgb_path；流程统一吃 image_path
        if isinstance(cap, dict) and not cap.get("image_path") and cap.get("rgb_path"):
            cap["image_path"]=cap["rgb_path"]
        if not cap.get("success") and self.allow_demo and tag == "pre_pick_offset":
            demo_path=ensure_demo_pre_pick_image()
            cap={
                "success":True,
                "camera_id":camera_id,
                "tag":tag,
                "camera_world_pose":self.twin.camera_world_pose(camera_id),
                "image_path":str(demo_path),
                "demo":True,
                "source":"generated_demo_image",
            }
        if required and not cap.get("success"): raise RuntimeError(f"{camera_id} 未取得 {tag} JPG")
        if required and not cap.get("image_path"):
            raise RuntimeError(f"{camera_id} 取得了 {tag} 结果但缺少 image_path/rgb_path")
        return cap

    def _capture_rgbd(self, camera_id: str, tag: str):
        cap=self.camera.capture_rgbd(camera_id,tag=tag)
        if not cap.get("success"): raise RuntimeError(f"{camera_id} 未取得 {tag} 的 RGB + 对齐深度")
        return cap

    def _probe_one_device(self, device_id: str, probe_fn, timeout_s: float = 8.0) -> dict:
        try:
            with ThreadPoolExecutor(max_workers=1) as pool:
                raw = pool.submit(probe_fn).result(timeout=max(1.0, float(timeout_s)))
            out = dict(raw or {})
            out.setdefault("device_id", device_id)
            out.setdefault("success", False)
            out.setdefault("message", "无返回信息")
            return out
        except FuturesTimeoutError:
            self.twin.update_device(device_id, status="OFFLINE", task="PROBE_TIMEOUT")
            return {
                "success": False,
                "device_id": device_id,
                "message": f"{device_id} 探测超时（>{timeout_s:.0f}s）",
            }
        except Exception as exc:
            self.twin.update_device(device_id, status="FAILED", task="PROBE_ERROR")
            return {"success": False, "device_id": device_id, "message": f"{device_id} 探测异常：{exc}"}

    def _device_check(self):
        """第1步：检查 PLC / 雷达 / 相机连接。失败只记日志，不阻断下一步。"""
        checks = []

        def _plc():
            return self.plc.connect()

        def _radar():
            if hasattr(self.radar, "probe"):
                return self.radar.probe()
            self.twin.update_device("RADAR", status="ONLINE", task="NO_PROBE")
            return {"success": True, "device_id": "RADAR", "message": "雷达适配器无 probe，已跳过硬件探测"}

        def _camera():
            if hasattr(self.camera, "connect"):
                return self.camera.connect()
            if hasattr(self.camera, "probe"):
                return self.camera.probe()
            self.twin.update_device("CAM_PICK", status="ONLINE", task="NO_PROBE")
            return {"success": True, "device_id": "CAM_PICK", "message": "相机适配器无 connect/probe，已跳过硬件探测"}

        for device_id, fn in (("PLC", _plc), ("RADAR", _radar), ("CAM_PICK", _camera)):
            item = self._probe_one_device(device_id, fn)
            item.setdefault("device_id", device_id)
            checks.append(item)
            ok_one = bool(item.get("success"))
            status = "SUCCESS" if ok_one else "FAILED"
            twin_status = "ONLINE" if ok_one else "OFFLINE"
            if device_id == "CAM_PICK":
                self.twin.update_camera(device_id, status=twin_status, task=status)
            else:
                self.twin.update_device(device_id, status=twin_status, task=status)
            self.twin.add_message(
                "DEVICE_CHECK",
                status,
                f"{device_id}：{item.get('message')}",
                item,
            )

        ok = [c for c in checks if c.get("success")]
        bad = [c for c in checks if not c.get("success")]
        all_ok = len(bad) == 0
        summary = (
            "外接设备全部就绪：" + "、".join(c["device_id"] for c in ok)
            if all_ok
            else (
                "外接设备检查未全部通过；失败："
                + "、".join(f"{c['device_id']}({c.get('message')})" for c in bad)
                + "（已记录，继续下一步）"
            )
        )
        result = {
            "success": all_ok,
            "continue_anyway": True,
            "all_online": all_ok,
            "online_count": len(ok),
            "failed_count": len(bad),
            "devices": {c["device_id"]: c for c in checks},
            "checks": checks,
            "message": summary,
            "device_mode": getattr(self, "device_mode", "mock"),
        }
        self.twin.add_message("DEVICE_CHECK", "SUCCESS" if all_ok else "FAILED", summary, result)
        self.round_data["device_check"] = result
        return result

    def _pre_pick_offset(self):
        """插取前货-托偏移。真机强制实拍；拍照/识别失败只记日志，不阻断下一步。"""
        cam=self._camera_for_role("pre_pick_offset","CAM_PICK")
        live=str(getattr(self,"device_mode","mock")).lower()=="real"
        if live and hasattr(self.camera,"set_tagged_image"):
            # 再清一次，防止中途又被调试输入写回示例图。
            self.camera.set_tagged_image("pre_pick_offset","")

        result={
            "success":False,
            "should_fork":False,
            "capture_success":False,
            "analysis_success":False,
            "continue_anyway":True,
            "phase":"pre_pick",
            "device_mode":getattr(self,"device_mode","mock"),
            "camera_id":cam,
            "message":"",
        }

        cap=self._capture_rgb(cam,"pre_pick_offset",required=False)
        result["capture"]=deepcopy(cap) if isinstance(cap,Mapping) else {"raw":cap}
        image_path=str((cap or {}).get("image_path") or (cap or {}).get("rgb_path") or "")
        if not (cap or {}).get("success") or not image_path:
            fail_msg=str((cap or {}).get("message") or f"{cam} 未取得 pre_pick_offset JPG")
            result["message"]=f"拍照失败：{fail_msg}（已记录，继续下一步）"
            self.twin.add_message("PRE_PICK_OFFSET","FAILED",result["message"],result)
            # TODO: 与PLC交互 — 上报插取前货托偏移检测结果（可否插取）
            self.plc.send_message("PALLET_OFFSET_PRE_PICK","FAILED",result["message"],result)
            self.round_data["pre_pick_offset"]=result
            return result

        result["capture_success"]=True
        result["image_path"]=image_path
        result["capture_source"]=(cap or {}).get("source")
        result["demo_input"]=bool((cap or {}).get("demo"))
        self.twin.add_message(
            "PRE_PICK_OFFSET",
            "INFO",
            f"已拍照：{image_path}" + ("（联调/示例）" if result["demo_input"] else "（实拍）"),
            {"image_path":image_path,"source":result.get("capture_source"),"demo":result["demo_input"]},
        )

        started=perf_counter()
        try:
            analysis=self.algorithms.pre_pick_offset(image_path,self.current_cargo or {})
            self._evidence(
                "PALLET_OVERHANG_PRE",
                {"image_path":image_path,"cargo_id":(self.current_cargo or {}).get("instance_id")},
                analysis,started,
            )
            if isinstance(analysis,Mapping):
                result.update(deepcopy(analysis))
            result["capture_success"]=True
            result["analysis_success"]=True
            result["continue_anyway"]=True
            if result.get("demo_input") or (cap or {}).get("demo"):
                result["demo_input"]=True
                result["input_source"]=(cap or {}).get("source")
                result["message"]="联调示例图（非相机实拍）："+str(result.get("message") or "偏移分析完成")
            if not result.get("should_fork"):
                base=str(result.get("message") or "插取前偏移不合格")
                result["message"]=f"{base}（已记录，继续下一步）"
                result["success"]=False
            else:
                result["success"]=True
                result["message"]=str(result.get("message") or "插取前偏移合格，允许插取")
        except Exception as exc:
            result["analysis_success"]=False
            result["should_fork"]=False
            result["success"]=False
            result["message"]=f"偏移识别失败：{exc}（已记录，继续下一步）"
            self._evidence(
                "PALLET_OVERHANG_PRE",
                {"image_path":image_path,"cargo_id":(self.current_cargo or {}).get("instance_id")},
                {"success":False,"message":str(exc)},started,
                status="FAILED",note="偏移算法异常，步骤软继续",
            )

        status="SUCCESS" if result.get("should_fork") else "FAILED"
        detail={
            "image_path":result.get("image_path"),
            "overhang_percent":result.get("overhang_percent"),
            "threshold_percent":result.get("threshold_percent"),
            "decision":result.get("decision"),
            "should_fork":result.get("should_fork"),
            "capture_success":result.get("capture_success"),
            "analysis_success":result.get("analysis_success"),
            "capture_source":result.get("capture_source"),
        }
        self.twin.add_message("PRE_PICK_OFFSET",status,result.get("message") or status,detail)
        self.plc.send_message("PALLET_OFFSET_PRE_PICK",status,result.get("message",status),result)
        self.round_data["pre_pick_offset"]=result
        return result

    def _pick_task(self):
        if self.round_data.get("pick_result",{}).get("success"): return self.round_data["pick_result"]
        self.twin.set_parallel("pick","RUNNING","3.1 插孔定位/插取中")
        pick_cfg=((self.device_config.get("robots") or {}).get("PICK_ARM") or {})
        cargo_mount=pick_cfg.get("cargo_mount_pose") or {
            "x_mm":0.0,"y_mm":1100.0,"z_mm":-350.0,
            "roll_deg":0.0,"pitch_deg":0.0,"yaw_deg":0.0,
        }
        staged_pose=Pose6D.from_any((self.twin.snapshot().get("cargo") or {}).get("pose"))
        pickup_tool=parent_pose_for_child(staged_pose,Pose6D.from_any(cargo_mount)).to_dict()
        approach=deepcopy(pickup_tool); approach["z_mm"]+=700.0
        self.robot.move_tool_world("PICK_ARM",approach,"MOVE_TO_TAIL_STAGED_CARGO")
        self.robot.move_tool_world("PICK_ARM",pickup_tool,"FIND_PALLET_HOLE")
        cam=self._camera_for_role("pallet_hole","CAM_PICK")
        cap=self.camera.capture_rgbd(cam,tag="pallet_hole")
        if cap.get("success"):
            started=perf_counter(); hole=self.algorithms.pallet_hole_recognize(cap["rgb_path"],cap["depth_path"],self.current_cargo)
            self._evidence("PALLET_HOLE_YOLO",{"rgb_path":cap["rgb_path"],"depth_path":cap["depth_path"]},hole,started,model_invoked=True)
        elif self.allow_demo:
            hole={"success":True,"demo":True,"left_xyz_mm":[-120,0,1000],"right_xyz_mm":[120,0,1000],"message":"插孔联调数据"}
            started=perf_counter(); self._evidence("PALLET_HOLE_YOLO",{"rgb_path":"","depth_path":""},hole,started,model_invoked=False,note="未取得RGB-D，模型未调用，使用联调插孔数据",status="FALLBACK")
        else:
            hole={"success":False,"message":"缺少插孔RGB-D"}
        if not hole.get("success"):
            result={"success":False,"message":hole.get("message","插孔定位失败"),"hole_result":hole}
        else:
            # 插孔算法输出相机坐标；相机挂在PICK_ARM上，使用拍摄时动态相机WORLD位姿转换后再送PLC。
            try:
                import numpy as np
                T = self.calibration.dynamic_camera_world_matrix(cap.get("camera_world_pose") or self.twin.camera_world_pose(cam))
                for side in ("left", "right"):
                    xyz = hole.get(f"{side}_xyz_mm")
                    if xyz is not None:
                        q = self.calibration.transform_point(T, xyz)
                        hole[f"{side}_world_xyz_mm"] = [float(x) for x in q]
                hole["world_coordinate_frame"] = "world"
            except Exception as exc:
                hole["world_transform_warning"] = str(exc)
            fork=self.robot.fork_pallet(hole)
            result={"success":bool(fork.get("success")),"message":fork.get("message"),"hole_result":hole,"fork_result":fork}
            if result["success"]:
                # 3.1完成后货物/托盘成为PICK_ARM的刚性载荷；后续任何
                # PICK_ARM位姿变化都必须同步更新货物WORLD位姿。
                result["cargo_attachment"]=self.twin.attach_cargo("PICK_ARM",cargo_mount)
                lift_pose=deepcopy(pickup_tool); lift_pose["z_mm"]+=850.0
                result["lift_move"]=self.robot.move_tool_world("PICK_ARM",lift_pose,"LIFT_STAGED_CARGO")
                result["message"]="托盘插取完成，货物已绑定 PICK_ARM 并进入随动运载状态"
        self.round_data["pick_result"]=result
        status="SUCCESS" if result.get("success") else "FAILED"
        self.twin.set_parallel("pick",status,result.get("message",""),result)
        # TODO: 与PLC交互 — 上报插孔定位/插取结果（3.1 并行支路）
        self.plc.send_message("PALLET_PICK",status,result.get("message",""),result)
        return result

    @staticmethod
    def _point_dict(v):
        if isinstance(v,Mapping): return {"x":float(v.get("x",v.get("x_mm",0))),"y":float(v.get("y",v.get("y_mm",0))),"z":float(v.get("z",v.get("z_mm",0)))}
        return {"x":float(v[0]),"y":float(v[1]),"z":float(v[2])}

    @staticmethod
    def _avg_point(a,b):
        return {k:0.5*(float(a[k])+float(b[k])) for k in ("x","y","z")}

    def _normalize_radar_result(self, raw: Dict[str,Any]) -> Dict[str,Any]:
        r=deepcopy(raw); frame=str(r.get("coordinate_frame") or "world").lower()
        world_raw=r.get("world_points") or {}
        ids=[str(x) for x in (r.get("corner_ids") or list(world_raw))]
        if not world_raw and isinstance(r.get("corner_points"),list):
            for item in r["corner_points"]:
                if isinstance(item,Mapping) and item.get("name"): world_raw[str(item["name"])]=item
            ids=[str(x) for x in (r.get("corner_ids") or list(world_raw))]
        world={}
        for pid in ids:
            if pid not in world_raw: continue
            p=self._point_dict(world_raw[pid])
            if frame not in {"world","radar_world"}:
                q=self.calibration.lidar_point_to_world([p["x"],p["y"],p["z"]]); p={"x":float(q[0]),"y":float(q[1]),"z":float(q[2])}
            world[pid]=p
        ids=[i for i in ids if i in world]
        collapsed=False
        if len(ids)==8 and all(f"P{i}" in world for i in range(1,9)):
            # 兼容旧点云模块8点输出，但仅压缩为“6个相机搜索粗点”。最终高低板仍由相机6点判断。
            old=deepcopy(world)
            world={
                "P1":old["P1"],"P2":old["P2"],
                "P3":self._avg_point(old["P3"],old["P5"]),
                "P4":self._avg_point(old["P4"],old["P6"]),
                "P5":old["P7"],"P6":old["P8"],
            }; ids=[f"P{i}" for i in range(1,7)]; collapsed=True
        if len(ids) not in {4,6}: raise RuntimeError(f"雷达找车必须输出4或6个粗角点（旧8点可自动压缩），当前：{ids}")
        return {
            "success":True,"message":r.get("message","雷达找车完成"),"coordinate_frame":"world","coordinate_unit":"mm",
            "corner_ids":ids,"world_points":world,"coarse_only":True,"collapsed_from_8_points":collapsed,
            "raw_radar_reference":{k:deepcopy(r.get(k)) for k in ("board_mode","boards","first_workface_loading_plan")},
            "final_board_judgement_source":"camera_not_radar",
        }

    def _radar_task(self):
        if self.round_data.get("radar_result",{}).get("success"): return self.round_data["radar_result"]
        self.twin.set_parallel("radar","RUNNING","3.2 雷达找车中")
        locate=self.radar.locate_truck(self.current_cargo)
        started=perf_counter(); raw_result=self.algorithms.radar_process(locate,self.current_cargo) if locate.get("success") else locate
        model_invoked=bool(locate.get("pcd_path"))
        # 真雷达实采时禁止再替换成联调粗点
        example_only=bool(model_invoked and self.debug_inputs.get("point_cloud_example_only") and not self._use_live_radar())
        algo_note = (
            "实验室几何雷达算法"
            if (model_invoked and feature_switches.USE_LAB_LIDAR_ALGO)
            else (
                "离线PCD已完成真实推理；因不属于当前场景标定，仅展示输入输出，不将其坐标发送给机械臂"
                if example_only
                else (
                    "Livox实采点云已处理"
                    if self._use_live_radar()
                    else "雷达仅用于相机粗搜索；最终板型由相机角点判断"
                )
            )
        )
        self._evidence(
            "POINTNET_TRUCK",
            {
                "pcd_path":locate.get("pcd_path",""),
                "example_only":example_only,
                "live_radar":self._use_live_radar(),
                "locate_source":locate.get("source"),
                "use_lab_lidar_algo":bool(feature_switches.USE_LAB_LIDAR_ALGO),
            },
            raw_result,
            started,
            model_invoked=model_invoked,
            note=algo_note if model_invoked else "无PCD，点云算法未调用",
            status="SUCCESS" if raw_result.get("success") else "FAILED",
        )
        if example_only:
            raw_result={
                "success":True,"demo":True,
                "message":"PointNet++离线示例已实际调用；当前场景运动继续使用已标定的4个联调粗点",
                "coordinate_frame":"world","coordinate_unit":"mm",
                "corner_ids":["P1","P2","P3","P4"],
                "world_points":{
                    "P1":{"x":-1250,"y":3000,"z":1450}, "P2":{"x":1250,"y":3000,"z":1450},
                    "P3":{"x":-1250,"y":15000,"z":1450}, "P4":{"x":1250,"y":15000,"z":1450},
                },
                "pointnet_example_reference":{
                    "pcd_path":locate.get("pcd_path"),"result_json_path":raw_result.get("result_json_path"),
                    "checkpoint_path":raw_result.get("checkpoint_path"),"board_count":raw_result.get("board_count"),
                    "corner_ids":raw_result.get("corner_ids"),"timing":raw_result.get("timing"),
                },
            }
        result=raw_result
        if result.get("success"): result=self._normalize_radar_result(result)
        self.round_data["radar_result"]=result
        status="SUCCESS" if result.get("success") else "FAILED"
        self.twin.set_parallel("radar",status,result.get("message",""),result)
        # TODO: 与PLC交互 — 上报雷达粗定位角点结果（3.2 并行支路）
        self.plc.send_message("RADAR_LOCATE",status,result.get("message",""),result)
        return result

    def _lab_pick_recognize(self):
        """实验室 2.1：到插孔位拍照识别两孔，发布坐标供 PLC 确认；不做物理插取。"""
        if self.round_data.get("pick_result", {}).get("success"):
            return self.round_data["pick_result"]
        self.twin.set_parallel("pick", "RUNNING", "实验室：插孔拍照识别中")
        pick_cfg = ((self.device_config.get("robots") or {}).get("PICK_ARM") or {})
        cargo_mount = pick_cfg.get("cargo_mount_pose") or {
            "x_mm": 0.0, "y_mm": 1100.0, "z_mm": -350.0,
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
        }
        staged_pose = Pose6D.from_any((self.twin.snapshot().get("cargo") or {}).get("pose"))
        pickup_tool = parent_pose_for_child(staged_pose, Pose6D.from_any(cargo_mount)).to_dict()
        approach = deepcopy(pickup_tool)
        approach["z_mm"] += 700.0

        # 接近位只更新孪生，不弹 PLC；识别出的插孔坐标再弹窗供用户改后下发。
        emitter = getattr(self.robot, "command_emitter", None)
        try:
            if hasattr(self.robot, "command_emitter"):
                self.robot.command_emitter = None
            self.robot.move_tool_world("PICK_ARM", approach, "MOVE_TO_TAIL_STAGED_CARGO")
            self.robot.move_tool_world("PICK_ARM", pickup_tool, "FIND_PALLET_HOLE")
        finally:
            if hasattr(self.robot, "command_emitter"):
                self.robot.command_emitter = emitter

        cam = self._camera_for_role("pallet_hole", "CAM_PICK")
        cap = self.camera.capture_rgbd(cam, tag="pallet_hole")
        if cap.get("success"):
            started = perf_counter()
            hole = self.algorithms.pallet_hole_recognize(cap["rgb_path"], cap["depth_path"], self.current_cargo)
            self._evidence(
                "PALLET_HOLE_YOLO",
                {"rgb_path": cap["rgb_path"], "depth_path": cap["depth_path"]},
                hole,
                started,
                model_invoked=True,
            )
        elif self.allow_demo:
            hole = {
                "success": True,
                "demo": True,
                "left_xyz_mm": [-120, 0, 1000],
                "right_xyz_mm": [120, 0, 1000],
                "result_image_path": "",
                "message": "插孔联调数据",
            }
            started = perf_counter()
            self._evidence(
                "PALLET_HOLE_YOLO",
                {"rgb_path": "", "depth_path": ""},
                hole,
                started,
                model_invoked=False,
                note="未取得RGB-D，使用联调插孔数据",
                status="FALLBACK",
            )
        else:
            hole = {"success": False, "message": "缺少插孔RGB-D"}

        if hole.get("success"):
            try:
                T = self.calibration.dynamic_camera_world_matrix(
                    cap.get("camera_world_pose") or self.twin.camera_world_pose(cam)
                )
                for side in ("left", "right"):
                    xyz = hole.get(f"{side}_xyz_mm")
                    if xyz is not None:
                        q = self.calibration.transform_point(T, xyz)
                        hole[f"{side}_world_xyz_mm"] = [float(x) for x in q]
                hole["world_coordinate_frame"] = "world"
            except Exception as exc:
                hole["world_transform_warning"] = str(exc)

            # 把左右插孔 WORLD 坐标作为可编辑下发目标（不做 fork）
            yaw = float(pickup_tool.get("yaw_deg", -90.0) or -90.0)
            for side, key, task in (
                ("left", "left_world_xyz_mm", "PALLET_HOLE_LEFT"),
                ("right", "right_world_xyz_mm", "PALLET_HOLE_RIGHT"),
            ):
                xyz = hole.get(key)
                if not xyz:
                    continue
                pose = {
                    "x_mm": float(xyz[0]),
                    "y_mm": float(xyz[1]),
                    "z_mm": float(xyz[2]),
                    "roll_deg": 0.0,
                    "pitch_deg": 0.0,
                    "yaw_deg": yaw,
                }
                self.robot.move_tool_world("PICK_ARM", pose, task)

            result = {
                "success": True,
                "lab_mode": True,
                "fork_skipped": True,
                "message": "插孔识别完成（实验室：仅下发坐标，不执行物理插取）",
                "hole_result": hole,
                "capture": deepcopy(cap) if isinstance(cap, Mapping) else cap,
                "image_path": str(
                    hole.get("result_image_path")
                    or (cap or {}).get("rgb_path")
                    or ""
                ),
            }
        else:
            result = {
                "success": False,
                "lab_mode": True,
                "fork_skipped": True,
                "message": hole.get("message", "插孔定位失败"),
                "hole_result": hole,
                "capture": deepcopy(cap) if isinstance(cap, Mapping) else cap,
                "image_path": str((cap or {}).get("rgb_path") or ""),
            }

        self.round_data["pick_result"] = result
        status = "SUCCESS" if result.get("success") else "FAILED"
        self.twin.set_parallel("pick", status, result.get("message", ""), result)
        self.plc.send_message("PALLET_PICK", status, result.get("message", ""), result)
        return result

    def _lab_sense(self):
        """实验室第2步：相机插孔识别 ∥ 雷达底板四角；软失败，结果都展示。

        后续轮次：只重跑插孔识别，复用首轮雷达车板（round_data 里预置的 radar_result / twin.corners）。
        """
        need_pick = not self.round_data.get("pick_result", {}).get("success")
        # 首轮才采雷达；后续轮若已有成功雷达结果则跳过
        need_radar = self.is_first_round and (not self.round_data.get("radar_result", {}).get("success"))
        with ThreadPoolExecutor(max_workers=1) as pool:
            fr = pool.submit(self._radar_task) if need_radar else None
            pick = self._lab_pick_recognize() if need_pick else self.round_data["pick_result"]
            radar = fr.result() if fr else (self.round_data.get("radar_result") or {"success": True, "message": "复用首轮雷达车板", "skipped": True})

        if radar.get("success") and radar.get("world_points"):
            corners = {
                pid: {"x": float(p["x"]), "y": float(p["y"]), "z": float(p["z"])}
                for pid, p in (radar.get("world_points") or {}).items()
            }
            if corners:
                self.twin.update_truck(
                    corners=corners,
                    board_mode="LAB_RADAR_COARSE",
                    camera_board_geometry={
                        "decision_source": "lab_radar_coarse",
                        "corner_ids": list(radar.get("corner_ids") or corners.keys()),
                    },
                )
                self.truck_initialized = True

        pick_ok = bool(pick.get("success"))
        radar_ok = bool(radar.get("success"))
        summary = {
            "success": pick_ok and radar_ok,
            "continue_anyway": True,
            "lab_mode": True,
            "pick": "SUCCESS" if pick_ok else "FAILED",
            "radar": "SUCCESS" if radar_ok else "FAILED",
            "radar_skipped": bool(radar.get("skipped")) or (not need_radar),
            "pick_result": pick,
            "radar_result": radar,
            "message": (
                "实验室感知完成：插孔与雷达四角均就绪"
                if pick_ok and radar_ok and need_radar
                else (
                    "实验室感知完成：插孔已识别（雷达复用首轮）"
                    if pick_ok and not need_radar
                    else f"实验室感知部分完成：插孔={('OK' if pick_ok else '失败')}，雷达={('OK' if radar_ok else '失败')}"
                )
            ),
        }
        self.round_data["lab_sense"] = summary
        if pick:
            self.round_data["pick_result"] = pick
        if radar and not radar.get("skipped"):
            self.round_data["radar_result"] = radar
        self.twin.add_message(
            "LAB_SENSE",
            "SUCCESS" if summary["success"] else "WARNING",
            summary["message"],
            {"pick": summary["pick"], "radar": summary["radar"], "radar_skipped": summary["radar_skipped"]},
        )
        return summary

    def _lab_corner_shell(self):
        """实验室第3步：雷达粗点引导 PLC 分组拍摄，相机 WORLD 点做最终角点。"""
        preserved_camera = self.round_data.get("camera_world_corners")
        preserved_final = self.round_data.get("final_world_corners")
        if preserved_camera and preserved_final:
            result = deepcopy(self.round_data.get("lab_corner_shell") or {})
            result.update(
                {
                    "success": True,
                    "lab_mode": True,
                    "reused": True,
                    "camera_world_corners": deepcopy(preserved_camera),
                    "final_world_corners": deepcopy(preserved_final),
                    "message": "实验室角点复用首轮相机 WORLD 结果，未重复移动相机扫描",
                }
            )
            self.twin.update_truck(
                corners=deepcopy(preserved_final),
                board_mode="LAB_CAMERA_FINAL",
                camera_board_geometry={"decision_source": "lab_camera_world_reused"},
            )
            self.round_data["lab_corner_shell"] = result
            self.twin.add_message("LAB_CORNER_SHELL", "SUCCESS", result["message"], result)
            self.twin.set_phase(result["message"], self.round_index + 1)
            return result

        radar = self.round_data.get("radar_result") or {}
        radar_points = radar.get("world_points") or {}
        if not radar.get("success") or not radar_points:
            result = {
                "success": False,
                "lab_mode": True,
                "camera_world_corners": {},
                "final_world_corners": {},
                "message": "实验室雷达没有可用 WORLD 角点，已停止本轮相机精定位",
            }
            self.round_data["lab_corner_shell"] = result
            self.twin.add_message("LAB_CORNER_SHELL", "WARNING", result["message"], result)
            return result

        try:
            planner = LabCameraVisitPlanner(self.algorithms.lab_camera_transform())
            targets = planner.build_pair_targets(
                radar_points,
                {"x": 0.0, "y": 0.0, "z": 380.0, "r": -80.0},
            )
            mapping = getattr(self.robot, "world_to_gantry", None)
            if not mapping:
                from config.external_devices_config import GANTRY

                mapping = GANTRY.get("world_to_gantry") or {}
        except Exception as exc:
            raise RuntimeError(f"实验室相机访问规划失败：{exc}") from exc

        camera_id = self._camera_for_role("corner", "CAM_PICK")
        capture_meta: dict[str, dict[str, Any]] = {}
        group_captures: dict[str, dict[str, Any]] = {}
        captured_groups: list[str] = []
        for item in targets:
            pair = tuple(item["pair"])
            group = "".join(pair)
            target_world_pose = planner.plc_to_world_pose(item["plc_command"], mapping)
            moved = self.robot.move_tool_world(
                "PICK_ARM",
                target_world_pose,
                f"LAB_CORNER_{group}",
            )
            plc_pose = self._plc_pose_for_lab_camera(moved, target_world_pose)
            cap = self._capture_rgbd(camera_id, f"LAB_CORNER_{group}")
            group_captures[group] = deepcopy(cap)
            captured_groups.append(group)
            camera_world_pose = deepcopy(cap.get("camera_world_pose") or self.twin.camera_world_pose(camera_id))
            for point_id in pair:
                capture_meta[point_id] = {
                    "camera_id": camera_id,
                    "capture_group": group,
                    "camera_world_pose": camera_world_pose,
                    "rgb_path": cap.get("rgb_path"),
                    "depth_path": cap.get("depth_path"),
                    "plc_pose": deepcopy(plc_pose),
                    "depth_scale_mm": cap.get("depth_scale_mm"),
                    "radar_coarse_world_xyz_mm": deepcopy(radar_points.get(point_id)),
                }

        try:
            camera_result = self.algorithms.lab_corner_world_recognize(
                ["P1", "P2", "P3", "P4"],
                capture_meta,
                group_captures,
            )
        except FileNotFoundError:
            raise
        except Exception as exc:
            camera_result = {
                "success": False,
                "algorithm": "lab_camera",
                "world_points": {},
                "message": f"实验室相机角点识别失败，逐点使用雷达兜底：{exc}",
            }

        camera_points = camera_result.get("world_points") or {}
        final_points = merge_lab_corner_points(radar_points, camera_points)
        result = {
            "success": bool(final_points),
            "lab_mode": True,
            "camera_capture_groups": captured_groups,
            "capture_meta": deepcopy(capture_meta),
            "camera_result": deepcopy(camera_result),
            "radar_world_corners": deepcopy(radar_points),
            "camera_world_corners": deepcopy(camera_points),
            "final_world_corners": deepcopy(final_points),
            "message": (
                "实验室相机 WORLD 角点已完成，逐点融合结果已更新"
                if camera_result.get("success")
                else "实验室相机角点部分失败，已逐点使用雷达 WORLD 点兜底"
            ),
        }
        self.round_data["camera_world_corners"] = deepcopy(camera_points)
        self.round_data["final_world_corners"] = deepcopy(final_points)
        self.round_data["lab_corner_shell"] = result
        self.twin.update_truck(
            corners=deepcopy(final_points),
            board_mode="LAB_CAMERA_FINAL",
            camera_board_geometry={
                "decision_source": "lab_camera_world_priority",
                "corner_sources": {pid: point.get("source") for pid, point in final_points.items()},
                "capture_groups": captured_groups,
            },
        )
        self.twin.add_message(
            "LAB_CORNER_SHELL",
            "SUCCESS" if result["success"] else "WARNING",
            result["message"],
            result,
        )
        self.twin.set_phase(result["message"], self.round_index + 1)
        return result

    def _lab_overhead_pose(self) -> dict:
        """估一个车板/货物上方的俯拍位姿（仅更新孪生，默认不写 PLC）。"""
        truck = self.twin.snapshot().get("truck") or {}
        corners = truck.get("corners") or {}
        if len(corners) >= 2:
            xs = [float(p.get("x", 0)) for p in corners.values()]
            ys = [float(p.get("y", 0)) for p in corners.values()]
            zs = [float(p.get("z", 0)) for p in corners.values()]
            return {
                "x_mm": sum(xs) / len(xs),
                "y_mm": sum(ys) / len(ys),
                "z_mm": max(zs) + 1800.0,
                "roll_deg": 0.0,
                "pitch_deg": 0.0,
                "yaw_deg": -90.0,
            }
        cargo = (self.twin.snapshot().get("cargo") or {}).get("pose") or {}
        return {
            "x_mm": float(cargo.get("x_mm", 0) or 0),
            "y_mm": float(cargo.get("y_mm", 0) or 0),
            "z_mm": float(cargo.get("z_mm", 0) or 0) + 1800.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": float(cargo.get("yaw_deg", -90) or -90),
        }

    def _lab_place_verify(self):
        """实验室第4步：人工搬货到车板后，臂上相机俯拍确认位置（算法暂留空）。"""
        overhead = self._lab_overhead_pose()
        emitter = getattr(self.robot, "command_emitter", None)
        try:
            if hasattr(self.robot, "command_emitter"):
                self.robot.command_emitter = None
            self.robot.move_tool_world("PICK_ARM", overhead, "LAB_PLACE_OVERHEAD")
        finally:
            if hasattr(self.robot, "command_emitter"):
                self.robot.command_emitter = emitter

        cam = self._camera_for_role("observe", "CAM_PICK")
        if not cam:
            cam = self._camera_for_role("pallet_hole", "CAM_PICK")
        cap = self._capture_rgb(cam, "lab_place_verify", required=False)
        image_path = str((cap or {}).get("image_path") or (cap or {}).get("rgb_path") or "")

        # 算法未就绪：只落盘拍照结果，占位字段留给后续接入「托盘相对车板」检测。
        algo = {
            "success": False,
            "pending_algorithm": True,
            "message": "俯拍位置校验算法尚未接入（仓内无「托盘相对车板俯拍确认」专用模块），仅保留照片",
        }
        result = {
            "success": bool((cap or {}).get("success") and image_path),
            "continue_anyway": True,
            "lab_mode": True,
            "manual_place": True,
            "capture": deepcopy(cap) if isinstance(cap, Mapping) else cap,
            "image_path": image_path,
            "overhead_pose": overhead,
            "algorithm": algo,
            "message": (
                f"已俯拍：{image_path}（算法待接入，请人工确认托盘落点）"
                if image_path
                else f"俯拍失败：{(cap or {}).get('message') or '无图像'}（已记录，可继续下一托）"
            ),
        }
        if result["success"]:
            # 实验室：标记本托已“放置”（人工搬上），便于下一轮切换货物
            cargo = deepcopy(self.twin.snapshot().get("cargo") or {})
            if cargo:
                cargo["status"] = "PLACED_MANUAL_LAB"
                cargo["attached_to"] = None
                self.twin.set_cargo(cargo)
        self.round_data["lab_place_verify"] = result
        status = "SUCCESS" if result.get("success") else "WARNING"
        self.twin.add_message("LAB_PLACE_VERIFY", status, result["message"], result)
        self.plc.send_message("LAB_PLACE_VERIFY", status, result["message"], result)
        return result

    def _advance_lab_round(self):
        """实验室一轮结束：进入下一托；雷达和首轮相机车板结果带到下一轮。"""
        self.completed.append(deepcopy(self.round_data))
        preserved_radar = None
        preserved_camera = None
        preserved_final = None
        for item in reversed(self.completed):
            radar = item.get("radar_result")
            if isinstance(radar, Mapping) and radar.get("success") and radar.get("world_points"):
                preserved_radar = deepcopy(radar)
            camera = item.get("camera_world_corners")
            final = item.get("final_world_corners")
            if camera and final:
                preserved_camera = deepcopy(camera)
                preserved_final = deepcopy(final)
            if preserved_radar and preserved_camera and preserved_final:
                break
        self.round_index += 1
        self.step_index = 0
        self.round_data = {}
        if preserved_radar:
            self.round_data["radar_result"] = preserved_radar
        if preserved_camera and preserved_final:
            self.round_data["camera_world_corners"] = preserved_camera
            self.round_data["final_world_corners"] = preserved_final
        if self.round_index >= len(self.queue):
            self.finished = True
            self.running = False
            self.twin.set_phase("实验室全部托盘流程完成", self.round_index)
            if self.loading_session_id:
                self.vehicle_db.finish_session(self.loading_session_id, "COMPLETED")
            return {"finished": True}
        self._load_cargo_to_twin()
        self.twin.set_phase(self.current_step[1], self.round_index + 1)
        return {"finished": False, "next_round": self.round_index + 1}

    def _parallel_locate(self):
        need_pick=not self.round_data.get("pick_result",{}).get("success")
        need_radar=not self.round_data.get("radar_result",{}).get("success")
        # Radar remains concurrent, but the robot branch runs on the caller/UI
        # thread so every interpolated pose is rendered in order instead of a
        # worker queue collapsing into the final position.
        with ThreadPoolExecutor(max_workers=1) as pool:
            fr=pool.submit(self._radar_task) if need_radar else None
            pick=self._pick_task() if need_pick else self.round_data["pick_result"]
            radar=fr.result() if fr else self.round_data["radar_result"]
        summary={"pick":"SUCCESS" if pick.get("success") else "FAILED","radar":"SUCCESS" if radar.get("success") else "FAILED","pick_result":pick,"radar_result":radar}
        # 四种组合均已分别发送PLC message；成功分支会被保留，下一次只重试失败分支。
        if not pick.get("success") or not radar.get("success"):
            raise RuntimeError(f"并行结果：插取={summary['pick']}，雷达={summary['radar']}；已保留成功分支，下一次只重试失败分支")
        return summary

    def _radar_to_camera(self):
        radar=self.round_data["radar_result"]; assignments={}
        points=radar["world_points"]
        endpoint_y=[float(points[pid]["y"]) for pid in radar["corner_ids"]]
        tail_y,head_y=min(endpoint_y),max(endpoint_y)
        midpoint=(tail_y+head_y)/2.0
        group_z={"TAIL":0.0,"HEAD":0.0}
        for pid in radar["corner_ids"]:
            p=points[pid]; group="TAIL" if float(p["y"]) <= midpoint else "HEAD"
            group_z[group]=max(group_z[group],float(p["z"]))
        for pid in radar["corner_ids"]:
            p=points[pid]; x,y,z=float(p["x"]),float(p["y"]),float(p["z"])
            group="TAIL" if y <= midpoint else "HEAD"
            stop_y=tail_y if group=="TAIL" else head_y
            # Photography remains one left-side pass: tail shot, then head shot.
            pose={"x_mm":GANTRY_LEFT_X_MM,"y_mm":stop_y,"z_mm":group_z[group]+500.0,"roll_deg":0,"pitch_deg":0,"yaw_deg":-90.0}
            assignments[pid]={
                "robot_id":"PICK_ARM","camera_id":self._camera_for_role("corner","CAM_PICK"),
                "capture_group":group,"radar_coarse_world_xyz_mm":[x,y,z],
                "target_robot_pose_world":pose,
            }
        # TODO: 与PLC交互 — 把雷达粗点换算成车尾/车头拍照目标位，下发给 PLC
        cmd=self.plc.send_command("SET_CORNER_CAPTURE_TARGETS",{"corner_count":len(assignments),"capture_count":2,"assignments":assignments})
        # TODO: 与PLC交互 — 等 PLC 确认已收到角点拍摄目标（超时见 plc.ack_timeout_ms）
        ack=self.plc.wait_ack(cmd["command_id"],int((self.system_config.get("plc") or {}).get("ack_timeout_ms",5000)))
        if not ack.get("success"): raise RuntimeError("PLC 未确认角点拍摄目标")
        self.round_data["corner_assignments"]=assignments
        return {"success":True,"assignments":assignments,"plc_ack":ack}

    def _capture_corners(self):
        images={}; meta={}; dimgs=self.debug_inputs.get("corner_depths") or {}; rimgs=self.debug_inputs.get("corner_images") or {}
        assignments=self.round_data["corner_assignments"]
        groups={name:[pid for pid,a in assignments.items() if a.get("capture_group")==name] for name in ("TAIL","HEAD")}
        group_captures={}
        for group in ("TAIL","HEAD"):
            point_ids=groups[group]
            if not point_ids: continue
            first=assignments[point_ids[0]]; tag=f"CORNER_{group}"
            moved=self.robot.move_tool_world(first["robot_id"],first["target_robot_pose_world"],task=f"CAPTURE_{group}")
            plc_pose=self._plc_pose_for_lab_camera(moved, first.get("target_robot_pose_world") or {})
            rgb_input=rimgs.get(group) or next((rimgs.get(pid) for pid in point_ids if rimgs.get(pid)),None)
            depth_input=dimgs.get(group) or next((dimgs.get(pid) for pid in point_ids if dimgs.get(pid)),None)
            if rgb_input and hasattr(self.camera,"set_tagged_image"): self.camera.set_tagged_image(tag,rgb_input)
            if depth_input and hasattr(self.camera,"set_tagged_depth"): self.camera.set_tagged_depth(tag,depth_input)
            cap=self._capture_rgbd(first["camera_id"],tag); group_captures[group]=deepcopy(cap)
            for pid in point_ids:
                a=assignments[pid]
                # A real endpoint capture remains one physical RGB-D group.
                # Offline module verification may additionally provide one
                # labelled example image/depth per Pn so every .pt invocation
                # has a known input and remains independently auditable.
                point_rgb=rimgs.get(pid) or cap["rgb_path"]
                point_depth=dimgs.get(pid) or cap["depth_path"]
                images[pid]=point_rgb
                meta[pid]={
                    "camera_id":a["camera_id"],"capture_group":group,
                    "camera_world_pose":deepcopy(cap["camera_world_pose"]),
                    "rgb_path":point_rgb,"depth_path":point_depth,
                    "depth_mode":(self.system_config.get("camera_corner_world") or {}).get("depth_mode","raw"),
                    "demo":bool(cap.get("demo")),"source":cap.get("source"),
                    "example_input":bool(self.debug_inputs.get("corner_inputs_example_only")),
                    "demo_target_world_xyz_mm":deepcopy(a.get("radar_coarse_world_xyz_mm")),
                    "plc_pose":deepcopy(plc_pose),
                    "depth_scale_mm":cap.get("depth_scale_mm"),
                }
        self.round_data["corner_images"]=images; self.round_data["corner_capture_meta"]=meta
        self.round_data["corner_group_captures"]=group_captures
        return {"success":True,"physical_capture_count":len(group_captures),"capture_groups":groups,"image_paths_by_point":images,"capture_meta":meta}

    @staticmethod
    def _plc_pose_for_lab_camera(moved: Mapping[str, Any] | None, target_world_pose: Mapping[str, Any]) -> Dict[str, float]:
        """提取实验室相机算法需要的 PLC XYZR。优先用运动返回的 gantry_xyzr。"""
        moved = moved or {}
        raw = moved.get("gantry_xyzr") or moved.get("plc_pose")
        if isinstance(raw, Mapping):
            if all(k in raw for k in ("X", "Y", "Z", "R")):
                return {"x": float(raw["X"]), "y": float(raw["Y"]), "z": float(raw["Z"]), "r": float(raw["R"])}
            if all(k in raw for k in ("x", "y", "z", "r")):
                return {k: float(raw[k]) for k in ("x", "y", "z", "r")}
        # mock / 无 gantry 返回时：用目标 WORLD 近似（实验室轴对齐场景可用）
        pose = target_world_pose or {}
        return {
            "x": float(pose.get("x_mm", pose.get("x", 0.0)) or 0.0),
            "y": float(pose.get("y_mm", pose.get("y", 0.0)) or 0.0),
            "z": float(pose.get("z_mm", pose.get("z", 0.0)) or 0.0),
            "r": float(pose.get("yaw_deg", pose.get("yaw", pose.get("R", pose.get("r", -80.0)))) or -80.0),
        }

    def set_corner_review_callback(self, callback):
        """UI registers a blocking human-review dialog here for step 6."""
        self.corner_review_callback = callback if callable(callback) else None

    def _apply_corner_review(self, recognition: Dict[str, Any]) -> Dict[str, Any]:
        """Optional human review/correction after .pt corner recognition."""
        if not self.corner_review_enabled:
            recognition = deepcopy(recognition)
            recognition["reviewed"] = False
            recognition["review_skipped"] = True
            recognition["message"] = str(recognition.get("message") or "") + "｜人工审核已关闭"
            return recognition
        source = str(recognition.get("source") or recognition.get("recognition_source") or "")
        if self.corner_review_auto_accept_demo and source in {"generated_demo_rgbd"}:
            recognition = deepcopy(recognition)
            recognition["reviewed"] = True
            recognition["review_skipped"] = True
            recognition["corrected_count"] = 0
            recognition["message"] = str(recognition.get("message") or "") + "｜联调角点自动通过人工审核"
            return recognition
        if self.corner_review_callback is None:
            recognition = deepcopy(recognition)
            recognition["reviewed"] = False
            recognition["review_skipped"] = True
            recognition["message"] = str(recognition.get("message") or "") + "｜无界面审核回调，已跳过人工审核"
            return recognition
        ids = list(recognition.get("corner_ids") or self.round_data["radar_result"]["corner_ids"])
        image_points = recognition.get("image_points") or {}
        for pid in ids:
            point = dict(image_points.get(pid) or {})
            if not point.get("image"):
                point["image"] = (self.round_data.get("corner_images") or {}).get(pid)
                image_points[pid] = point
        reviewed = self.corner_review_callback(
            image_points,
            ids,
            str((self.current_cargo or {}).get("instance_id") or f"round{self.round_index+1}"),
        )
        if not isinstance(reviewed, Mapping) or not reviewed.get("success"):
            raise RuntimeError(str((reviewed or {}).get("message") or "角点人工审核未通过"))
        merged = deepcopy(recognition)
        merged.update(deepcopy(reviewed))
        merged["success"] = True
        merged["recognition_source"] = recognition.get("recognition_source") or recognition.get("source")
        merged["model_path"] = recognition.get("model_path")
        merged["result_dir"] = recognition.get("result_dir")
        started = perf_counter()
        self._evidence(
            "CORNER_REVIEW",
            {
                "corner_ids": ids,
                "model_image_points": recognition.get("image_points"),
                "corrected_point_ids": reviewed.get("corrected_point_ids") or [],
            },
            merged,
            started,
            model_invoked=False,
            note="角点人工审核/手动矫正",
            status="SUCCESS",
        )
        return merged

    def _corner_recognition(self):
        ids=self.round_data["radar_result"]["corner_ids"]
        meta=self.round_data.get("corner_capture_meta") or {}
        if feature_switches.USE_LAB_CAMERA_ALGO and not (
            self.allow_demo and ids and all((meta.get(pid) or {}).get("demo") for pid in ids)
        ):
            started=perf_counter()
            result=self.algorithms.lab_corner_world_recognize(
                ids,
                meta,
                self.round_data.get("corner_group_captures") or {},
            )
            self._evidence(
                "CORNER_YOLO",
                {
                    "image_paths_by_point":self.round_data.get("corner_images"),
                    "corner_ids":ids,
                    "use_lab_camera_algo":True,
                },
                result,
                started,
                model_invoked=True,
                note="实验室 cam_yolo_lab：YOLO+深度+外参直接出 WORLD",
                status="SUCCESS" if result.get("success") else "FAILED",
            )
            # 实验室路径已得到 WORLD，跳过像素审核（审核面向像素修正）
            result=deepcopy(result)
            result["reviewed"]=False
            result["review_skipped"]=True
            self.round_data["corner_recognition"]=result
            if result.get("success") and result.get("world_points"):
                self.round_data["lab_camera_world_corners"]=deepcopy(result)
            return result
        if self.allow_demo and ids and all((meta.get(pid) or {}).get("demo") for pid in ids):
            image_points={}
            for pid in ids:
                camera_id=meta[pid]["camera_id"]
                intr=(((self.twin.snapshot().get("cameras") or {}).get(camera_id) or {}).get("intrinsics") or {})
                image_points[pid]={
                    "x":float(intr.get("cx",658.0)),"y":float(intr.get("cy",374.0)),
                    "confidence":1.0,"status":"联调示例角点","point_name":pid,
                    "image":self.round_data["corner_images"][pid],"source":"generated_demo_rgbd",
                }
            result={"success":True,"corner_ids":list(ids),"image_points":image_points,"source":"generated_demo_rgbd","message":"缺失角点数据已自动补齐；使用联调 RGB-D 中心标记"}
        else:
            started=perf_counter(); result=self.algorithms.corner_image_recognize(self.round_data["corner_images"],ids,self.current_cargo)
            self._evidence("CORNER_YOLO",{"image_paths_by_point":self.round_data["corner_images"],"corner_ids":ids,"use_lab_camera_algo":False},result,started,model_invoked=True)
        if result.get("source")=="generated_demo_rgbd":
            started=perf_counter(); self._evidence("CORNER_YOLO",{"image_paths_by_point":self.round_data["corner_images"],"corner_ids":ids},result,started,model_invoked=False,note="联调中心点分支，角点.pt未调用",status="FALLBACK")
        result=self._apply_corner_review(result)
        self.round_data["corner_recognition"]=result; return result

    def _camera_to_world(self):
        ids=self.round_data["radar_result"]["corner_ids"]
        meta=self.round_data["corner_capture_meta"]
        # 实验室相机开关：识别步骤已直接产出 WORLD，这里透传，避免二次变换
        lab_ready=self.round_data.get("lab_camera_world_corners")
        if feature_switches.USE_LAB_CAMERA_ALGO and isinstance(lab_ready, Mapping) and lab_ready.get("success") and lab_ready.get("world_points"):
            started=perf_counter()
            result={
                "success":True,
                "source":"lab_yolo_d435i_world",
                "coordinate_frame":"world",
                "coordinate_unit":"mm",
                "corner_ids":list(ids),
                "world_points":deepcopy(lab_ready.get("world_points")),
                "details":deepcopy(lab_ready.get("details") or {}),
                "radar_used_for_final_geometry":False,
                "message":"实验室相机算法 WORLD 结果透传（未再跑现场 CameraCornerWorldService）",
                "use_lab_camera_algo":True,
            }
            self._evidence("CAMERA_WORLD",{"capture_meta":meta,"use_lab_camera_algo":True},result,started,note="实验室一体 WORLD 透传")
            self.round_data["camera_world_corners"]=result
            self.twin.update_truck(corners=result["world_points"])
            return result
        if self.allow_demo and ids and all((meta.get(pid) or {}).get("demo") for pid in ids):
            world={}
            for pid in ids:
                xyz=meta[pid].get("demo_target_world_xyz_mm") or [0.0,0.0,0.0]
                world[pid]={"x":float(xyz[0]),"y":float(xyz[1]),"z":float(xyz[2]),"source":"generated_demo_rgbd"}
            result={
                "success":True,"corner_ids":list(ids),"world_points":world,
                "coordinate_frame":"world","coordinate_unit":"mm","source":"generated_demo_rgbd",
                "message":"联调 RGB-D 已自动补齐；WORLD 角点沿用当前雷达粗点作为模拟相机测量值",
                "production_warning":"该分支仅用于界面/流程联调，真实运行仍使用像素+深度+动态外参转换",
            }
        elif self.allow_demo and ids and all((meta.get(pid) or {}).get("example_input") for pid in ids):
            # The bundled JPG/depth pairs are valid model-call examples, but
            # they are not calibrated to this truck scene. Run the conversion
            # for auditable I/O, then use the scene's coarse demo corners for
            # layout so the two-column regions remain on the truck deck.
            started=perf_counter()
            converted=self.corner_world.convert(
                (self.round_data["corner_recognition"].get("image_points") or {}),
                meta,
                self.round_data["radar_result"],
                depth_mode=(self.system_config.get("camera_corner_world") or {}).get("depth_mode","raw"),
            )
            world={}
            for pid in ids:
                xyz=meta[pid].get("demo_target_world_xyz_mm") or [0.0,0.0,0.0]
                world[pid]={"x":float(xyz[0]),"y":float(xyz[1]),"z":float(xyz[2]),"source":"example_rgbd_scene_aligned"}
            result={
                "success":True,"corner_ids":list(ids),"world_points":world,
                "coordinate_frame":"world","coordinate_unit":"mm","source":"example_rgbd_scene_aligned",
                "message":"联调示例 RGB-D 已完成转换调用；场景区域使用当前车辆粗点对齐，保持在车板两列内",
                "raw_example_conversion":converted,
                "production_warning":"示例图片/深度不属于当前车辆标定，正式运行不启用场景对齐分支",
            }
            self._evidence(
                "CAMERA_WORLD",
                {"image_points":self.round_data["corner_recognition"].get("image_points"),"capture_meta":meta,"scene_alignment_reference":self.round_data["radar_result"].get("world_points")},
                result,started,note="示例RGB-D完成转换调用，但布局使用当前车辆粗点防止示例标定混入场景",status="FALLBACK",
            )
        else:
            started=perf_counter()
            result=self.corner_world.convert(
                (self.round_data["corner_recognition"].get("image_points") or {}),
                meta,
                self.round_data["radar_result"],
                depth_mode=(self.system_config.get("camera_corner_world") or {}).get("depth_mode","raw"),
            )
            self._evidence("CAMERA_WORLD",{"image_points":self.round_data["corner_recognition"].get("image_points"),"capture_meta":meta},result,started)
        if result.get("source")=="generated_demo_rgbd":
            started=perf_counter(); self._evidence("CAMERA_WORLD",{"capture_meta":meta},result,started,note="联调分支沿用雷达粗点，不是正式像素深度转换",status="FALLBACK")
        self.round_data["camera_world_corners"]=result
        self.twin.update_truck(corners=result["world_points"])
        return result

    def _initial_space_plan(self):
        camera_result=self.round_data["camera_world_corners"]
        started=perf_counter(); geometry=self.board_geometry.analyze(camera_result["world_points"],camera_result["corner_ids"])
        snap=self.space.initialize_from_camera_geometry(geometry); self.truck_initialized=True
        self.round_data["camera_board_geometry"]=geometry
        self.twin.update_truck(board_mode=geometry["board_mode"],camera_board_geometry=geometry,regions=snap["regions"],occupied=snap["occupied"],available=snap["available"],remaining_space={"available_count":len(snap["available"]),"borrow_plan":snap.get("borrow_plan")})
        result={"success":True,"geometry":geometry,"space":snap,"radar_used_for_board_judgement":False}
        self._evidence("BOARD_GEOMETRY_PLAN",{"world_points":camera_result["world_points"],"corner_ids":camera_result["corner_ids"]},result,started)
        return result

    def _side_for_target(self, target):
        target=target or {}
        column=str(target.get("column") or "").strip().upper()
        pose=target.get("final_world_pose") or target.get("nominal_world_pose") or {}
        # Camera corner ordering can swap A/B labels, so select the physical
        # rail from the target's WORLD X relative to the truck centre. Column
        # is only a fallback for incomplete external targets.
        truck_pose=((self.twin.snapshot().get("truck") or {}).get("pose") or {})
        truck_x=float(truck_pose.get("x_mm",0.0) or 0.0)
        if "x_mm" in pose or "x" in pose:
            target_x=float(pose.get("x_mm",pose.get("x",0.0)) or 0.0)
            right_side=target_x>truck_x
        else:
            right_side=column=="B"
        outer_x=GANTRY_RIGHT_X_MM if right_side else GANTRY_LEFT_X_MM
        yaw=90.0 if right_side else -90.0
        return "PICK_ARM",self._camera_for_role("neighbor","CAM_PICK"),outer_x,yaw

    @staticmethod
    def _side_name(outer_x: float) -> str:
        return "RIGHT" if float(outer_x)>0.0 else "LEFT"

    def _move_gantry_to_target_side(self, target, target_z_mm: float, task: str, pitch_deg: float = 0.0):
        """Travel on the two-rail gantry and lower on the selected side."""
        rid,cam,outer_x,yaw=self._side_for_target(target)
        pose=(target or {}).get("final_world_pose") or (target or {}).get("nominal_world_pose") or {}
        target_y=float(pose.get("y_mm",0.0) or 0.0)
        current=Pose6D.from_any((((self.twin.snapshot().get("devices") or {}).get(rid) or {}).get("pose") or {}))
        clearance=max(GANTRY_CLEARANCE_Z_MM,float(target_z_mm)+350.0,current.z_mm)
        side=self._side_name(outer_x); moves=[]

        def move(pose_data, phase):
            result=self.robot.move_tool_world(rid,pose_data,task=f"{task}_{phase}")
            moves.append(result)
            if not result.get("success"):
                raise RuntimeError(result.get("message") or f"龙门架{phase}运动失败")

        if abs(current.z_mm-clearance)>1.0:
            move({"x_mm":current.x_mm,"y_mm":current.y_mm,"z_mm":clearance,"roll_deg":0.0,"pitch_deg":0.0,"yaw_deg":current.yaw_deg},"LIFT_CLEARANCE")
        if abs(current.y_mm-target_y)>1.0:
            move({"x_mm":current.x_mm,"y_mm":target_y,"z_mm":clearance,"roll_deg":0.0,"pitch_deg":0.0,"yaw_deg":current.yaw_deg},"LONGITUDINAL")
        if abs(current.x_mm-outer_x)>1.0 or abs(current.yaw_deg-yaw)>1.0:
            move({"x_mm":outer_x,"y_mm":target_y,"z_mm":clearance,"roll_deg":0.0,"pitch_deg":0.0,"yaw_deg":yaw},f"CROSSBEAM_TO_{side}")
        lower={"x_mm":outer_x,"y_mm":target_y,"z_mm":float(target_z_mm),"roll_deg":0.0,"pitch_deg":float(pitch_deg),"yaw_deg":yaw}
        move(lower,f"LOWER_{side}")
        return {"success":True,"robot_id":rid,"camera_id":cam,"selected_side":side,"rail_x_mm":outer_x,"yaw_deg":yaw,"moves":moves,"final_pose":lower}

    def _neighbor_pose(self):
        started=perf_counter()
        tentative=self.space.peek_next_target(self.current_cargo)
        occupied=self.space.snapshot().get("occupied") or []
        if not occupied:
            result={"success":True,"source":"no_neighbor_first_position","compensation_world_mm":{"dx":0.0,"dy":0.0,"dz":0.0,"dyaw_deg":0.0},"message":"目标附近尚无已放托盘，8.2补偿为0"}
        else:
            rid,cam,outer_x,yaw=self._side_for_target(tentative); p=tentative["nominal_world_pose"]
            gantry_move=self._move_gantry_to_target_side(tentative,float(p["z_mm"])+500.0,"NEIGHBOR_PALLET_POSE")
            cap=self.camera.capture_rgb(cam,tag="neighbor_pose")
            dbg=self.debug_inputs.get("neighbor_pose_measurement")
            if not cap.get("success") and not self.allow_demo: raise RuntimeError("8.2 未取得临近托盘姿态 JPG")
            result=self.neighbor_pose.analyze(cap.get("image_path",""),tentative,dbg)
            result["camera_id"]=cam; result["camera_world_pose"]=cap.get("camera_world_pose")
            result["gantry_move"]=gantry_move; result["selected_side"]=self._side_name(outer_x)
        self.round_data["neighbor_pose"]=result; self.round_data["tentative_target"]=tentative
        self._evidence("NEIGHBOR_POSE",{"image_path":result.get("image_path","") ,"target":tentative,"measurement":self.debug_inputs.get("neighbor_pose_measurement")},result,started,note="该接口当前没有正式视觉权重；有输入时消费外部/离线测量")
        # TODO: 与PLC交互 — 上报临近托盘姿态/补偿测量结果（8.2）
        self.plc.send_message("NEIGHBOR_PALLET_POSE","SUCCESS",result.get("message",""),result)
        return {"success":True,"tentative_target":tentative,"neighbor_pose":result}

    def _target_confirm(self):
        if not self.truck_initialized: raise RuntimeError("首轮相机车板模型尚未初始化")
        started=perf_counter(); target=self.space.confirm_target(self.round_data.get("neighbor_pose"),first=(self.round_index==0),cargo=self.current_cargo)
        self.round_data["placement_target"]=target
        snap=self.space.snapshot(); self.twin.update_truck(current_target=target,regions=snap["regions"],occupied=snap["occupied"],available=snap["available"])
        result={"success":True,"target":target,"message":"已依据可用空间、历史反馈和8.2临近托盘补偿锁定本轮目标"}
        self._evidence("SPACE_TARGET",{"space":snap,"neighbor_pose":self.round_data.get("neighbor_pose"),"cargo":self.current_cargo},result,started)
        return result

    def _dynamic_monitor(self, phase: str):
        target=self.round_data.get("placement_target") or {}
        rid,cam,outer_x,yaw=self._side_for_target(target)
        pose=target.get("final_world_pose") or {}
        task="PRE_PLACE_DYNAMIC_MONITOR" if phase=="pre_place" else "POST_PLACE_BOTTOM_PALLET_DETECT"
        gantry_move=self._move_gantry_to_target_side(target,float(pose.get("z_mm",0.0))+650.0,task,pitch_deg=-10.0)
        tag="pre_place_monitor" if phase=="pre_place" else "post_place_bottom_pallet"
        cap=self._capture_rgbd(cam,tag)
        camera_path=self.debug_inputs.get("dynamic_camera_path") or str(PROJECT_ROOT/"examples"/"dynamic_monitoring"/"camera.json")
        started=perf_counter(); result=self.dynamic_monitoring.analyze(
            cap["rgb_path"],cap["depth_path"],camera_path,phase,
            (self.current_cargo or {}).get("instance_id","cargo"),
        )
        result["gantry_move"]=gantry_move; result["selected_side"]=self._side_name(outer_x)
        module_id="DYNAMIC_PRE_PLACE" if phase=="pre_place" else "DYNAMIC_POST_PLACE"
        self._evidence(module_id,{"rgb_path":cap["rgb_path"],"depth_path":cap["depth_path"],"camera_path":camera_path,"camera_world_pose":cap.get("camera_world_pose")},result,started)
        key="pre_place_monitor" if phase=="pre_place" else "post_place_bottom_pallet"
        self.round_data[key]=result
        # TODO: 与PLC交互 — 上报放货前/放货后动态监测结果给 PLC
        self.plc.send_message(module_id,"SUCCESS",result.get("message","动态监测完成"),result)
        return result

    def _place(self):
        target=self.round_data.get("placement_target")
        if not target: raise RuntimeError("缺少8.3锁定的放置目标")
        # TODO: 与PLC交互 — 把最终放置点下发给 PLC，再驱动放货
        cmd=self.plc.send_command("SET_PLACE_POINT",target)
        # TODO: 与PLC交互 — 等 PLC 确认最终放置点已接收
        ack=self.plc.wait_ack(cmd["command_id"],int((self.system_config.get("plc") or {}).get("ack_timeout_ms",5000)))
        if not ack.get("success"): raise RuntimeError("PLC 未确认最终放置点")
        final_cargo_pose=deepcopy(target.get("final_world_pose") or {})
        if not final_cargo_pose: raise RuntimeError("8.3目标缺少最终货物WORLD位姿")
        carry_move=self._move_gantry_to_target_side(
            target,float(final_cargo_pose.get("z_mm",0.0))+1200.0,
            "CARRY_TO_PLACEMENT_SIDE",
        )
        approach_cargo_pose=deepcopy(final_cargo_pose)
        approach_cargo_pose["z_mm"]=float(approach_cargo_pose.get("z_mm",0.0))+700.0
        extend_move=self._move_attached_cargo_world(approach_cargo_pose,"EXTEND_CARGO_FROM_OUTSIDE_TO_TARGET")
        lower_move=self._move_attached_cargo_world(final_cargo_pose,"LOWER_CARGO_TO_TARGET")
        if not lower_move.get("success"): raise RuntimeError(lower_move.get("message","货物下降到目标失败"))
        result=self.robot.place(self.current_cargo,target)
        if not result.get("success"): raise RuntimeError(result.get("message","放置失败"))
        placed_cargo=self.twin.detach_cargo(final_cargo_pose,"PLACED")
        self.twin.update_device("PICK_ARM",task="CARGO_RELEASED")
        self.round_data["place_result"]=result
        return {"success":True,"target":target,"selected_side":carry_move["selected_side"],"plc_ack":ack,"carry_move":carry_move,"extend_move":extend_move,"lower_move":lower_move,"place_result":result,"placed_cargo":placed_cargo}

    def _post_region(self):
        started=perf_counter()
        target=self.round_data["placement_target"]; rid,cam,outer_x,yaw=self._side_for_target(target); p=target["final_world_pose"]
        gantry_move=self._move_gantry_to_target_side(target,float(p["z_mm"])+500.0,"PALLET_BOARD_REGION_DEVIATION",pitch_deg=-10.0)
        cap=self.camera.capture_rgb(cam,tag="region_deviation")
        if not cap.get("success") and not self.allow_demo: raise RuntimeError("10.1 未取得托盘-车板区域偏差 JPG")
        dev=self.region_deviation.analyze(cap.get("image_path",""),target,self.debug_inputs.get("post_region_measurement"))
        # 两个相邻面观测：侧边到位 -> 向车板内侧伸入。
        imgs=self.debug_inputs.get("images") or {}
        if imgs.get("face_a") and hasattr(self.camera,"set_tagged_image"): self.camera.set_tagged_image("face_a",imgs["face_a"])
        if imgs.get("face_b") and hasattr(self.camera,"set_tagged_image"): self.camera.set_tagged_image("face_b",imgs["face_b"])
        faces=self.two_face.observe(self.current_cargo,target)
        result={"success":True,"region_deviation":dev,"two_face_observation":faces,"camera_id":cam,"selected_side":self._side_name(outer_x),"gantry_move":gantry_move}
        self._evidence("REGION_DEVIATION",{"image_path":cap.get("image_path","") ,"target":target,"measurement":self.debug_inputs.get("post_region_measurement")},dev,started,note="该接口当前没有正式视觉权重；示例消费外部测量")
        self._evidence("TWO_FACE_CAPTURE",{"face_a":faces.get("face_a"),"face_b":faces.get("face_b"),"target":target},faces,started,note="当前只完成两个相邻面采集，没有识别模型")
        self.round_data["post_region"]=result
        # TODO: 与PLC交互 — 上报托盘相对车板区域偏差（10.1）
        self.plc.send_message("PALLET_BOARD_REGION_DEVIATION","SUCCESS",dev.get("message",""),dev); return result

    def _post_cargo_offset(self):
        target=self.round_data["placement_target"]; rid,cam,outer_x,yaw=self._side_for_target(target)
        p=target["final_world_pose"]
        gantry_move=self._move_gantry_to_target_side(target,float(p["z_mm"])+500.0,"PALLET_CARGO_POST_PLACE_OFFSET",pitch_deg=-10.0)
        cap=self._capture_rgb(cam,"post_place_offset",required=True)
        started=perf_counter(); result=self.algorithms.post_place_offset(cap["image_path"],self.current_cargo)
        self._evidence("PALLET_CARGO_POST",{"image_path":cap["image_path"],"cargo":self.current_cargo},result,started)
        if cap.get("demo"):
            result["demo_input"]=True; result["input_source"]=cap.get("source")
            result["message"]="联调示例图（非相机实拍）："+str(result.get("message") or "放置后偏移分析完成")
        # 这是相机局部横向补偿量；未做方向标定前不直接写入WORLD XY。
        signed=result.get("signed_center_offset_mm")
        result["compensation_camera_horizontal_mm"]=None if signed in (None,"") else -float(signed)
        result["applied_directly_to_world_next_target"]=False
        result["selected_side"]=self._side_name(outer_x); result["gantry_move"]=gantry_move
        self.round_data["post_cargo_offset"]=result
        # TODO: 与PLC交互 — 上报放货后货托偏移（相机局部补偿量）
        self.plc.send_message("PALLET_CARGO_POST_PLACE","SUCCESS",result.get("message",""),result); return result

    def _feedback(self):
        started=perf_counter()
        reg=(self.round_data.get("post_region") or {}).get("region_deviation") or {}
        fb=self.placement_comp.next_feedback_from_region(reg)
        self.space.set_persistent_feedback(fb); self.twin.set_feedback_compensation(fb["dx"],fb["dy"],fb["dz"])
        occupied=self.space.occupy_current(self.current_cargo["instance_id"]); snap=self.space.snapshot()
        self.twin.update_truck(regions=snap["regions"],occupied=snap["occupied"],available=snap["available"],remaining_space={"available_count":len(snap["available"]),"borrow_plan":snap.get("borrow_plan")})
        cargo_comp=(self.round_data.get("post_cargo_offset") or {}).get("compensation_camera_horizontal_mm")
        data={"next_pallet_world_compensation_mm":fb,"cargo_on_pallet_camera_horizontal_compensation_mm":cargo_comp,"occupied_region":occupied,"available_count":len(snap["available"])}
        # TODO: 与PLC交互 — 把本轮偏差反馈给 PLC，并用于修正下一托盘目标/空间
        self.plc.send_message("PLACEMENT_FEEDBACK","SUCCESS","偏差信息已返回PLC并更新下一托盘可用空间",data)
        result={"success":True,**data}
        self._evidence("PLACEMENT_FEEDBACK",{"region_result":reg,"post_cargo_offset":self.round_data.get("post_cargo_offset"),"space_before_update":snap},result,started)
        return result

    def _return(self):
        # The next round always starts with staged cargo at the left-tail home.
        # From a right-side placement, first lift, travel to the tail and cross
        # there; never cut diagonally through the truck/load envelope.
        home={"column":"A","final_world_pose":{"x_mm":GANTRY_LEFT_X_MM,"y_mm":1000.0,"z_mm":1800.0}}
        gantry_return=self._move_gantry_to_target_side(home,1800.0,"RETURN_TO_LEFT_TAIL")
        results=[self.robot.retract("PICK_ARM")]
        return {"success":all(bool(x.get("success")) for x in results),"selected_side":"LEFT","gantry_return":gantry_return,"robot_returns":results}

    def _advance_round_after_return(self):
        self.completed.append(deepcopy(self.round_data))
        self.round_index+=1; self.step_index=0; self.round_data={}
        if self.round_index>=len(self.queue):
            self.finished=True; self.running=False
            return {"finished":True}
        self._load_cargo_to_twin()
        return {"finished":False,"next_round_skips":["3.2 RADAR","4 RADAR_TO_CAMERA","5 CAPTURE_CORNERS","6 CORNER_RECOGNITION","7 CAMERA_TO_WORLD","8.1 INITIAL_SPACE_PLAN"]}

    def execute_next(self):
        if self.finished: return self.snapshot()
        if not self.running: self.start()
        code,name=self.current_step; self.twin.set_phase(name,self.round_index+1)
        try:
            if code=="DEVICE_CHECK": data=self._device_check()
            elif code=="LAB_SENSE": data=self._lab_sense()
            elif code=="LAB_CORNER_SHELL": data=self._lab_corner_shell()
            elif code=="LAB_PLACE_VERIFY":
                data=self._lab_place_verify()
                soft_ok=bool((data or {}).get("success"))
                self._record(
                    code,
                    name,
                    "success" if soft_ok else "warning",
                    str((data or {}).get("message") or name),
                    data,
                )
                advance=self._advance_lab_round()
                self._persist_database_snapshot()
                self.twin.set_alarm(None)
                self._save()
                return self.snapshot()
            elif code=="PRE_PICK_OFFSET": data=self._pre_pick_offset()
            elif code=="PARALLEL_LOCATE": data=self._parallel_locate()
            elif code=="PICK_ONLY":
                data=self._pick_task()
                if not data.get("success"): raise RuntimeError(data.get("message","插取失败"))
            elif code=="RADAR_TO_CAMERA": data=self._radar_to_camera()
            elif code=="CAPTURE_CORNERS": data=self._capture_corners()
            elif code=="CORNER_RECOGNITION": data=self._corner_recognition()
            elif code=="CAMERA_TO_WORLD": data=self._camera_to_world()
            elif code=="INITIAL_SPACE_PLAN": data=self._initial_space_plan()
            elif code=="NEIGHBOR_POSE": data=self._neighbor_pose()
            elif code=="TARGET_CONFIRM": data=self._target_confirm()
            elif code=="PRE_PLACE_MONITOR": data=self._dynamic_monitor("pre_place")
            elif code=="PLACE": data=self._place()
            elif code=="POST_PLACE_BOTTOM": data=self._dynamic_monitor("post_place_bottom_pallet")
            elif code=="POST_REGION": data=self._post_region()
            elif code=="POST_CARGO_OFFSET": data=self._post_cargo_offset()
            elif code=="FEEDBACK": data=self._feedback()
            elif code=="RETURN":
                data=self._return()
                if not data.get("success"):
                    raise RuntimeError("机械臂返回失败")
                # 先按当前轮保存第12步，再切换到下一轮，避免历史记录轮次错位。
                self._record(code,name,"success",name,data)
                advance=self._advance_round_after_return()
                self._persist_database_snapshot()
                if advance.get("finished") and self.loading_session_id:
                    self.vehicle_db.finish_session(self.loading_session_id,"COMPLETED")
                self._save()
                return self.snapshot()
            else: raise RuntimeError(code)
            rec_status="success"
            rec_message=name
            if code=="DEVICE_CHECK":
                soft_ok=bool((data or {}).get("all_online"))
                if not soft_ok:
                    rec_status="warning"
                    rec_message=str((data or {}).get("message") or name)
            elif code=="LAB_SENSE":
                soft_ok=bool((data or {}).get("success"))
                if not soft_ok:
                    rec_status="warning"
                    rec_message=str((data or {}).get("message") or name)
            elif code=="LAB_CORNER_SHELL":
                pass
            elif code=="PRE_PICK_OFFSET":
                # 拍照/识别失败也继续；总体结果用 warning 标出来。
                soft_ok=bool((data or {}).get("capture_success") and (data or {}).get("should_fork"))
                if not soft_ok:
                    rec_status="warning"
                    rec_message=str((data or {}).get("message") or name)
            self._record(code,name,rec_status,rec_message,data); self.step_index+=1
            self.twin.set_alarm(None)
            self.twin.set_phase(self.current_step[1],self.round_index+1)
        except Exception as exc:
            self._record(code,name,"failed",str(exc),{"error":str(exc),"parallel":deepcopy(self.twin.snapshot().get("parallel"))}); self.twin.set_alarm(str(exc)); raise
        self._save(); return self.snapshot()

    def continue_after_step_failure(self, reason: str = "") -> dict:
        """用户确认在硬失败后仍前进：本步记 warning，step_index +1，不重跑失败逻辑。"""
        if self.finished:
            return self.snapshot()
        code, name = self.current_step
        detail = {
            "forced_continue": True,
            "reason": str(reason or "用户选择失败后继续"),
        }
        self._record(
            code,
            name,
            "warning",
            f"失败后继续：{detail['reason']}",
            detail,
        )
        self.twin.set_alarm(None)
        self.step_index += 1
        if self.step_index >= len(self.steps):
            # 理论上 RETURN 成功才会换轮；若 RETURN 失败后仍继续，当作本轮结束尝试进入下一货
            self.step_index = len(self.steps) - 1
            try:
                advance = self._advance_round_after_return()
                if advance.get("finished") and self.loading_session_id:
                    self.vehicle_db.finish_session(self.loading_session_id, "COMPLETED")
            except Exception:
                self.finished = True
                self.running = False
        else:
            self.twin.set_phase(self.current_step[1], self.round_index + 1)
        self._save()
        return self.snapshot()

    def _save(self):
        STATE_FILE.parent.mkdir(parents=True,exist_ok=True)
        STATE_FILE.write_text(json.dumps(self.twin.snapshot(),ensure_ascii=False,indent=2,default=str),encoding="utf-8")
        RESULT_FILE.write_text(json.dumps({"results":self.results},ensure_ascii=False,indent=2,default=str),encoding="utf-8")

    def snapshot(self):
        return {"running":self.running,"finished":self.finished,"round":self.round_index+1 if self.queue else 0,"total":len(self.queue),"completed":len(self.completed),"first_round":self.is_first_round,"step_code":self.current_step[0],"step_name":self.current_step[1],"current_cargo":deepcopy(self.current_cargo),"round_data":deepcopy(self.round_data),"twin":self.twin.snapshot(),"results":deepcopy(self.results),"calibration":self.calibration.diagnostic_summary(),"module_evidence":self.module_evidence.snapshot(),"device_mode":getattr(self,"device_mode","mock"),"run_profile":feature_switches.RUN_PROFILE,"allow_demo":bool(self.allow_demo),"database":{"backend":self.vehicle_db.backend,"location":self.vehicle_db.location,"path":self.vehicle_db.location,"session_id":self.loading_session_id}}
