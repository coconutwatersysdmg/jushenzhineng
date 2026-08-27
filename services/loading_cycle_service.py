# -*- coding: utf-8 -*-
"""具身智能循环装载主流程。

每件货物：
托盘插孔识别 -> 找货 -> 货物上托盘 -> 雷达 PCD 点云处理车板 ->
角点 .pt 逐点视觉确认 -> 插取前托盘/货物偏差 -> 插取托盘 ->
放货前动态监测 -> 按车板/高低板规划放置 -> 放置后偏差 ->
放置后动态监测 -> 下一轮。

不依赖数据库、事件队列、E1-E6、登录或管理模块。
"""
from __future__ import annotations

import json
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from adapters.mock_adapters import MockCameraAdapter, MockRadarAdapter, MockRobotAdapter
from services.corner_recognition_service import CornerRecognitionService
from services.deviation_monitoring_service import DeviationMonitoringService
from services.pallet_hole_recognition_service import PalletHoleRecognitionService
from services.point_cloud_processing_service import PointCloudProcessingService
from services.truck_bed_planning_service import TruckBedPlanningService

PROJECT_ROOT = Path(__file__).resolve().parents[1]
STATE_FILE = PROJECT_ROOT / "runtime" / "loading_cycle_state.json"
CONFIG_FILE = PROJECT_ROOT / "config" / "system_config.json"


class LoadingCycleService:
    STEP_DEFS = (
        ("PALLET_HOLE", "识别托盘插孔"),
        ("FIND_CARGO", "寻找当前货物"),
        ("LOAD_PALLET", "货物放到托盘"),
        ("RADAR", "雷达点云解析车板"),
        ("CORNER", ".pt 确认车板角点"),
        ("PRE_PICK_DEVIATION", "插取前托盘/货物偏差"),
        ("FORK_PALLET", "插取托盘"),
        ("PRE_PLACE_MONITOR", "放货前监测车板托盘"),
        ("PLACE", "按车板规划放到卡车"),
        ("POST_PLACE_DEVIATION", "放置后托盘/货物偏差"),
        ("POST_PLACE_MONITOR", "放置后动态监测托盘"),
        ("NEXT", "本轮完成 / 下一轮"),
    )

    def __init__(
        self,
        robot=None,
        camera=None,
        radar=None,
        pallet_recognizer=None,
        point_cloud_processor=None,
        corner_recognizer=None,
        deviation_monitor=None,
        placement_planner=None,
        state_file: Path = STATE_FILE,
        allow_pallet_demo_fallback: Optional[bool] = None,
        allow_point_cloud_demo_fallback: Optional[bool] = None,
        allow_corner_demo_fallback: Optional[bool] = None,
    ):
        self.config = self._load_config()
        runtime_cfg = self.config.get("runtime", {})
        planning_cfg = self.config.get("placement_planning", {})
        self.robot = robot or MockRobotAdapter()
        self.camera = camera or MockCameraAdapter()
        self.radar = radar or MockRadarAdapter()
        self.pallet_recognizer = pallet_recognizer or PalletHoleRecognitionService()
        self.point_cloud_processor = point_cloud_processor or PointCloudProcessingService()
        self.corner_recognizer = corner_recognizer or CornerRecognitionService(
            model_path=PROJECT_ROOT / "models" / "corner_service.pt"
        )
        self.deviation_monitor = deviation_monitor or DeviationMonitoringService()
        self.placement_planner = placement_planner or TruckBedPlanningService(
            use_point_cloud_plan=bool(planning_cfg.get("use_point_cloud_plan", True))
        )
        self.state_file = Path(state_file)
        self.allow_pallet_demo_fallback = bool(
            runtime_cfg.get("allow_pallet_demo_fallback", True)
            if allow_pallet_demo_fallback is None else allow_pallet_demo_fallback
        )
        self.allow_point_cloud_demo_fallback = bool(
            runtime_cfg.get("allow_point_cloud_demo_fallback", False)
            if allow_point_cloud_demo_fallback is None else allow_point_cloud_demo_fallback
        )
        self.allow_corner_demo_fallback = bool(
            runtime_cfg.get("allow_corner_demo_fallback", False)
            if allow_corner_demo_fallback is None else allow_corner_demo_fallback
        )
        self.monitor_sample_count = int(self.config.get("dynamic_monitor", {}).get("sample_count", 5))
        self.plan: List[Dict[str, Any]] = []
        self.queue: List[Dict[str, Any]] = []
        self.completed: List[Dict[str, Any]] = []
        self.current_index = 0
        self.step_index = 0
        self.running = False
        self.finished = False
        self.round_data: Dict[str, Any] = {}
        self.logs: List[Dict[str, Any]] = []

    @staticmethod
    def _load_config() -> Dict[str, Any]:
        if CONFIG_FILE.is_file():
            try:
                return json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
            except Exception:
                pass
        return {}

    @property
    def step_code(self) -> str:
        if self.finished:
            return "DONE"
        return self.STEP_DEFS[min(self.step_index, len(self.STEP_DEFS) - 1)][0]

    @property
    def step_name(self) -> str:
        if self.finished:
            return "全部货物装载完成"
        return self.STEP_DEFS[min(self.step_index, len(self.STEP_DEFS) - 1)][1]

    @property
    def current_cargo(self) -> Optional[Dict[str, Any]]:
        if 0 <= self.current_index < len(self.queue):
            return self.queue[self.current_index]
        return None

    @property
    def total_count(self) -> int:
        return len(self.queue)

    @property
    def completed_count(self) -> int:
        return len(self.completed)

    def set_plan(self, items: Iterable[Dict[str, Any]]):
        self.plan = [deepcopy(x) for x in items if isinstance(x, dict)]
        self.queue = []
        sequence = 0
        for item in self.plan:
            quantity = max(1, int(item.get("quantity") or 1))
            for unit in range(1, quantity + 1):
                sequence += 1
                cargo = deepcopy(item)
                cargo["unit_index"] = unit
                cargo["sequence"] = sequence
                cargo["instance_id"] = f"{cargo.get('cargo_code') or 'CARGO'}-{unit:03d}-{sequence:03d}"
                self.queue.append(cargo)
        self.reset_runtime(keep_plan=True)

    def reset_runtime(self, keep_plan: bool = True):
        if not keep_plan:
            self.plan = []
            self.queue = []
        self.completed = []
        self.current_index = 0
        self.step_index = 0
        self.running = False
        self.finished = False
        self.round_data = {}
        self.logs = []
        self._save_state()

    def start(self):
        if not self.queue:
            raise RuntimeError("装载清单为空，请先添加货物")
        for name, device in (("机器人", self.robot), ("相机", self.camera), ("雷达", self.radar)):
            if not device.check_status():
                raise RuntimeError(f"{name}状态异常")
        self.running = True
        self.finished = False
        self._log("SYSTEM", f"开始循环装载，共 {len(self.queue)} 件货物")
        self._save_state()
        return self.snapshot()

    def _log(self, step: str, message: str, payload: Any = None):
        item = {
            "time": datetime.now().isoformat(timespec="seconds"),
            "step": step,
            "message": str(message),
        }
        if payload is not None:
            item["payload"] = payload
        self.logs.append(item)

    @staticmethod
    def _require_success(result: Dict[str, Any], default_message: str):
        if not isinstance(result, dict) or not result.get("success", False):
            message = (result or {}).get("message") if isinstance(result, dict) else default_message
            raise RuntimeError(message or default_message)
        return result

    @staticmethod
    def _pallet_demo_result() -> Dict[str, Any]:
        return {
            "success": True,
            "recognition_source": "pallet_demo_fallback",
            "coordinate_frame": "camera",
            "coordinate_unit": "mm",
            "left_xyz_mm": [-120.0, 0.0, 1000.0],
            "right_xyz_mm": [120.0, 0.0, 1000.0],
            "fork_holes": [],
            "message": "未配置托盘 RGB-D；联调模式使用示例插孔 XYZ。",
        }

    @staticmethod
    def _point_cloud_demo_result() -> Dict[str, Any]:
        """仅用于无 PCD 时的显式联调，正式配置默认关闭。"""
        label2 = {
            "board_label": "label_2",
            "corner_ids": ["P1", "P2", "P3", "P4"],
            "corners_xyz_mm": [
                [3000.0, 0.0, 1400.0],
                [3000.0, 3000.0, 1400.0],
                [7380.0, 0.0, 1430.0],
                [7380.0, 3000.0, 1430.0],
            ],
            "length_mm": 4380.0,
            "width_mm": 3000.0,
            "height_mean_mm": 1415.0,
            "tilt_angle_deg": 0.39,
            "offset_angle_deg": 2.28,
            "loading_plan": {
                "pallet_length_mm": 1200.0,
                "normal_pallet_count": 3,
                "remaining_length_mm": 780.0,
                "remaining_ratio": 0.65,
                "pad_trigger_ratio": 0.5,
                "pad_required": True,
                "remaining_workface_compensation_mm": 420.0,
                "final_pallet_count": 4,
            },
        }
        label3 = {
            "board_label": "label_3",
            "corner_ids": ["P5", "P6", "P7", "P8"],
            "corners_xyz_mm": [
                [3000.0, 0.0, 1200.0],
                [3000.0, 3000.0, 1200.0],
                [16000.0, 0.0, 1220.0],
                [16000.0, 3000.0, 1220.0],
            ],
            "length_mm": 13000.0,
            "width_mm": 3000.0,
            "height_mean_mm": 1210.0,
            "tilt_angle_deg": 0.09,
            "offset_angle_deg": 0.8,
        }
        boards = [label2, label3]
        world = {}
        points = []
        for board in boards:
            for name, xyz in zip(board["corner_ids"], board["corners_xyz_mm"]):
                world[name] = {"x": xyz[0], "y": xyz[1], "z": xyz[2]}
                points.append({"name": name, "x": xyz[0], "y": xyz[1], "z": xyz[2], "board_label": board["board_label"]})
        return {
            "success": True,
            "recognition_source": "point_cloud_demo_fallback",
            "coordinate_frame": "radar_world",
            "coordinate_unit": "mm",
            "board_count": 2,
            "board_mode": "high_low_board",
            "is_high_low_board": True,
            "low_board_label": "label_3",
            "high_board_label": "label_2",
            "height_difference_mm": 205.0,
            "corner_ids": [f"P{i}" for i in range(1, 9)],
            "corner_points": points,
            "world_points": world,
            "corner_points_xyz": [[p["x"], p["y"], p["z"]] for p in points],
            "corner_points_xyz_mm": [[p["x"], p["y"], p["z"]] for p in points],
            "boards": boards,
            "first_workface_label": "label_2",
            "first_workface_loading_plan": label2["loading_plan"],
            "message": "联调模式：使用示例高低板点云几何结果",
        }

    @staticmethod
    def _corner_demo_from_radar(radar_result: Dict[str, Any]) -> Dict[str, Any]:
        ids = [str(x) for x in (radar_result.get("corner_ids") or [])]
        raw_world = radar_result.get("world_points") or {}
        world = {}
        if isinstance(raw_world, dict):
            for name in ids:
                point = raw_world.get(name)
                if isinstance(point, dict) and "x" in point and "y" in point:
                    world[name] = {
                        "x": float(point["x"]),
                        "y": float(point["y"]),
                        "z": float(point.get("z", 0.0)),
                    }
        if len(world) != len(ids) or len(ids) < 4:
            raise RuntimeError("点云未返回完整车板角点，无法进行角点视觉联调 fallback")
        xyz = [[world[n]["x"], world[n]["y"], world[n]["z"]] for n in ids]
        return {
            "success": True,
            "recognition_source": "point_cloud_only_demo_fallback",
            "coordinate_frame": radar_result.get("coordinate_frame") or "radar_world",
            "coordinate_unit": radar_result.get("coordinate_unit") or "mm",
            "confirmed_corner_ids": ids,
            "image_points": {},
            "world_points": world,
            "corner_points_xyz": xyz,
            "corner_points_xyz_mm": xyz,
            "board_count": radar_result.get("board_count"),
            "board_mode": radar_result.get("board_mode"),
            "boards": radar_result.get("boards") or [],
            "first_workface_loading_plan": radar_result.get("first_workface_loading_plan"),
            "message": "联调模式：未使用角点图像，直接采用点云 3D 角点。",
        }

    def _loaded_for_monitor(self, include_current: bool = False) -> List[Dict[str, Any]]:
        result = [deepcopy(x) for x in self.completed]
        if include_current and self.round_data.get("place_result"):
            result.append(deepcopy(self.round_data))
        return result

    def _dynamic_monitor(self, phase: str, include_current: bool) -> Dict[str, Any]:
        loaded = self._loaded_for_monitor(include_current=include_current)
        raw = self._require_success(
            self.radar.monitor_truck_pallets(loaded, phase=phase, sample_count=self.monitor_sample_count),
            "卡车底板托盘动态监测失败",
        )
        evaluated = self.deviation_monitor.evaluate_dynamic_monitor(raw, phase=phase)
        return self._require_success(evaluated, evaluated.get("message") or "托盘动态监测超限")

    def execute_next_step(self) -> Dict[str, Any]:
        if self.finished:
            return self.snapshot()
        if not self.running:
            self.start()
        cargo = self.current_cargo
        if cargo is None:
            self.finished = True
            self.running = False
            self._log("DONE", "全部货物装载完成")
            self._save_state()
            return self.snapshot()

        code = self.step_code

        if code == "PALLET_HOLE":
            self._require_success(self.robot.move_to_pallet(), "机器人无法到达托盘位置")
            capture = self.camera.capture_pallet_rgbd()
            if isinstance(capture, dict) and capture.get("success"):
                pallet_result = self.pallet_recognizer.recognize(
                    capture.get("rgb_path", ""),
                    capture.get("depth_path", ""),
                    result_tag=str(cargo.get("instance_id") or cargo.get("cargo_code") or "cargo"),
                )
            elif self.allow_pallet_demo_fallback:
                pallet_result = self._pallet_demo_result()
            else:
                raise RuntimeError("托盘插孔识别需要 RGB 图和对齐的 16 位深度图")
            self._require_success(pallet_result, "托盘插孔识别失败")
            align = self._require_success(self.robot.align_with_pallet(pallet_result), "托盘插孔对位规划失败")
            self.round_data = {
                "cargo": deepcopy(cargo),
                "pallet_result": pallet_result,
                "pallet_align_result": align,
            }
            self._log(code, pallet_result.get("message") or "托盘插孔识别完成", pallet_result)

        elif code == "FIND_CARGO":
            result = self._require_success(self.robot.move_to_cargo(cargo), "前往货位失败")
            self.round_data["find_cargo_result"] = result
            self._log(code, result.get("message") or "已到达货位")

        elif code == "LOAD_PALLET":
            result = self._require_success(self.robot.load_cargo_to_pallet(cargo), "货物放置托盘失败")
            self.round_data["load_pallet_result"] = result
            self._log(code, result.get("message") or "货物已放置托盘")

        elif code == "RADAR":
            capture = self.radar.capture_truck_point_cloud(cargo)
            if isinstance(capture, dict) and capture.get("success") and capture.get("pcd_path"):
                radar_result = self.point_cloud_processor.process_file(
                    capture["pcd_path"],
                    result_tag=str(cargo.get("instance_id") or cargo.get("cargo_code") or "cargo"),
                )
            elif self.allow_point_cloud_demo_fallback:
                radar_result = self._point_cloud_demo_result()
            else:
                raise RuntimeError("雷达点云处理需要现场/测试 PCD 文件；当前未取得有效 .pcd")
            self._require_success(radar_result, "雷达点云车板解析失败")
            placement_plan = self.placement_planner.plan(radar_result, self.completed, cargo)
            self.round_data["radar_result"] = radar_result
            self.round_data["placement_plan"] = placement_plan
            self._log(
                code,
                radar_result.get("message") or "雷达点云车板解析完成",
                {
                    "board_count": radar_result.get("board_count"),
                    "board_mode": radar_result.get("board_mode"),
                    "corner_ids": radar_result.get("corner_ids"),
                    "first_workface_loading_plan": radar_result.get("first_workface_loading_plan"),
                    "placement_plan": placement_plan,
                },
            )

        elif code == "CORNER":
            radar_result = self.round_data.get("radar_result")
            if not radar_result:
                raise RuntimeError("缺少雷达点云车板结果")
            expected_ids = [str(x) for x in (radar_result.get("corner_ids") or [])]
            if len(expected_ids) < 4:
                raise RuntimeError("点云结果未提供至少 4 个车板角点")
            capture = self.camera.capture_corner_images(cargo, expected_corner_ids=expected_ids)
            image_source = capture.get("image_paths_by_point") if isinstance(capture, dict) else {}
            image_source = image_source if isinstance(image_source, dict) else {}
            missing = [name for name in expected_ids if not image_source.get(name)]
            if not missing:
                corner_result = self.corner_recognizer.recognize(
                    image_source=image_source,
                    radar_result=radar_result,
                    plate_no="",
                    result_tag=str(cargo.get("instance_id") or cargo.get("cargo_code") or "cargo"),
                    expected_corner_ids=expected_ids,
                )
                corner_result["message"] = (
                    f"{len(expected_ids)} 张局部图已分别由角点 .pt 确认，并与雷达 3D 角点一一绑定"
                )
            elif self.allow_corner_demo_fallback:
                corner_result = self._corner_demo_from_radar(radar_result)
                corner_result["missing_corner_images"] = missing
            else:
                raise RuntimeError(
                    f"当前车板需要 {len(expected_ids)} 张角点局部图（{', '.join(expected_ids)}）；"
                    f"缺少：{', '.join(missing)}"
                )
            self._require_success(corner_result, "角点确认失败")
            self.round_data["corner_result"] = corner_result
            self._log(code, corner_result.get("message") or "车板角点位置确认完成", corner_result.get("corner_points_xyz"))

        elif code == "PRE_PICK_DEVIATION":
            measurement = self._require_success(
                self.radar.measure_pallet_cargo_deviation(cargo, phase="pre_pick"),
                "插取前托盘/货物偏差测量失败",
            )
            result = self.deviation_monitor.evaluate_pallet_cargo(measurement, stage="插取前")
            self._require_success(result, "插取前托盘/货物偏差超限")
            self.round_data["pre_pick_deviation"] = result
            self._log(code, result["message"], result)

        elif code == "FORK_PALLET":
            pallet_result = self.round_data.get("pallet_result")
            deviation = self.round_data.get("pre_pick_deviation")
            if not pallet_result or not deviation:
                raise RuntimeError("缺少插孔识别或插取前偏差结果")
            result = self._require_success(self.robot.fork_pallet(pallet_result, deviation), "托盘插取失败")
            self.round_data["fork_result"] = result
            self._log(code, result.get("message") or "托盘插取完成")

        elif code == "PRE_PLACE_MONITOR":
            monitor = self._dynamic_monitor(phase="下一个货物放置前", include_current=False)
            self.round_data["pre_place_monitor"] = monitor
            self._log(code, monitor["message"], monitor)

        elif code == "PLACE":
            corner_result = self.round_data.get("corner_result")
            placement_plan = self.round_data.get("placement_plan")
            if not corner_result:
                raise RuntimeError("缺少车板角点确认结果，不能执行放置")
            if not placement_plan:
                raise RuntimeError("缺少基于点云车板的放置规划")
            result = self._require_success(
                self.robot.place_cargo_on_truck(cargo, placement_plan, corner_result),
                "货物/托盘放置卡车失败",
            )
            self.round_data["place_result"] = result
            self.round_data["target"] = placement_plan
            self._log(code, result.get("message") or "货物/托盘已放到卡车", placement_plan)

        elif code == "POST_PLACE_DEVIATION":
            measurement = self._require_success(
                self.radar.measure_pallet_cargo_deviation(cargo, phase="after_place"),
                "放置后托盘/货物偏差测量失败",
            )
            result = self.deviation_monitor.evaluate_pallet_cargo(measurement, stage="放置后")
            self._require_success(result, "放置后托盘/货物偏差超限")
            self.round_data["post_place_deviation"] = result
            self._log(code, result["message"], result)

        elif code == "POST_PLACE_MONITOR":
            monitor = self._dynamic_monitor(phase="本轮放货完成后", include_current=True)
            self.round_data["post_place_monitor"] = monitor
            self._log(code, monitor["message"], monitor)

        elif code == "NEXT":
            self._require_success(self.robot.return_for_next_round(), "机器人返回失败")
            finished_round = deepcopy(self.round_data)
            finished_round["finished_at"] = datetime.now().isoformat(timespec="seconds")
            self.completed.append(finished_round)
            self._log(code, f"第 {self.current_index + 1} 轮完成：{cargo.get('cargo_name') or cargo.get('cargo_code')}")
            self.current_index += 1
            self.round_data = {}
            if self.current_index >= len(self.queue):
                self.finished = True
                self.running = False
                self._log("DONE", f"全部 {len(self.queue)} 件货物装载完成")
                self.step_index = 0
                self._save_state()
                return self.snapshot()
            self.step_index = 0
            self._save_state()
            return self.snapshot()

        self.step_index += 1
        self._save_state()
        return self.snapshot()

    def snapshot(self) -> Dict[str, Any]:
        return {
            "running": self.running,
            "finished": self.finished,
            "step_code": self.step_code,
            "step_name": self.step_name,
            "current_index": self.current_index,
            "current_round": min(self.current_index + 1, self.total_count) if self.total_count else 0,
            "total_count": self.total_count,
            "completed_count": self.completed_count,
            "current_cargo": deepcopy(self.current_cargo),
            "round_data": deepcopy(self.round_data),
            "completed": deepcopy(self.completed),
            "logs": deepcopy(self.logs),
        }

    def _save_state(self):
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "plan": self.plan,
            "queue": self.queue,
            **self.snapshot(),
        }
        self.state_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
