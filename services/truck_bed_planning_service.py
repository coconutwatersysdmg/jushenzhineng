# -*- coding: utf-8 -*-
"""基于雷达车板几何结果的装载作业面选择。

重点消费点云模块已经给出的两块底板/第一作业面剩余空间规划，不重复实现其判定规则。
这里只负责把算法结果映射到当前第几块托盘应该放到哪一个作业面，以及是否进入垫板槽位。
"""
from __future__ import annotations

from typing import Any, Dict, Iterable, Mapping


class TruckBedPlanningService:
    def __init__(self, use_point_cloud_plan: bool = True) -> None:
        self.use_point_cloud_plan = bool(use_point_cloud_plan)

    @staticmethod
    def _board_map(point_cloud_result: Mapping[str, Any]) -> Dict[str, Dict[str, Any]]:
        result: Dict[str, Dict[str, Any]] = {}
        for board in point_cloud_result.get("boards") or []:
            if isinstance(board, Mapping) and board.get("board_label"):
                result[str(board["board_label"])] = dict(board)
        return result

    @staticmethod
    def _completed_count(completed_rounds: Iterable[Mapping[str, Any]], workface: str) -> int:
        count = 0
        for row in completed_rounds or []:
            if not isinstance(row, Mapping):
                continue
            plan = row.get("placement_plan") or row.get("target") or {}
            if isinstance(plan, Mapping) and str(plan.get("workface") or "") == str(workface):
                count += 1
        return count

    @staticmethod
    def _manual_target(cargo: Mapping[str, Any]) -> Dict[str, Any]:
        target = cargo.get("target")
        if isinstance(target, Mapping):
            return dict(target)
        result: Dict[str, Any] = {}
        if cargo.get("target_position") not in (None, "", "AUTO"):
            result["label"] = str(cargo.get("target_position"))
        for axis in ("x", "y", "z"):
            for suffix in ("mm", "m"):
                key = f"target_{axis}_{suffix}"
                if cargo.get(key) not in (None, ""):
                    result[key] = float(cargo[key])
        if cargo.get("target_yaw_deg") not in (None, ""):
            result["yaw_deg"] = float(cargo["target_yaw_deg"])
        return result

    def plan(
        self,
        point_cloud_result: Mapping[str, Any],
        completed_rounds: Iterable[Mapping[str, Any]],
        cargo: Mapping[str, Any],
    ) -> Dict[str, Any]:
        if not point_cloud_result.get("success"):
            raise RuntimeError("车板点云结果无效，不能生成放置规划")
        boards = self._board_map(point_cloud_result)
        if not boards:
            raise RuntimeError("车板点云结果中没有可用作业面")

        manual_target = self._manual_target(cargo)
        first_plan = point_cloud_result.get("first_workface_loading_plan")
        workface = None
        slot_index = 1
        pad_required = False
        compensation_mm = 0.0
        planning_basis = "detected_board_geometry"

        # 两块底板时优先使用点云模块对 label_2 已经计算好的第一作业面规划。
        if self.use_point_cloud_plan and isinstance(first_plan, Mapping) and "label_2" in boards:
            label2_used = self._completed_count(completed_rounds, "label_2")
            final_count = int(first_plan.get("final_pallet_count") or 0)
            normal_count = int(first_plan.get("normal_pallet_count") or 0)
            if final_count > 0 and label2_used < final_count:
                workface = "label_2"
                slot_index = label2_used + 1
                pad_required = bool(first_plan.get("pad_required")) and slot_index > normal_count
                compensation_mm = (
                    float(first_plan.get("remaining_workface_compensation_mm") or 0.0)
                    if pad_required else 0.0
                )
                planning_basis = "point_cloud_first_workface_loading_plan"
            elif "label_3" in boards:
                workface = "label_3"
                slot_index = self._completed_count(completed_rounds, "label_3") + 1
                planning_basis = "first_workface_finished_switch_to_label_3"

        if workface is None:
            # 单底板，或第一作业面规划不适用时，使用当前检测到的第一块有效底板。
            workface = next(iter(boards))
            slot_index = self._completed_count(completed_rounds, workface) + 1

        board = boards[workface]
        result = {
            "success": True,
            "source": "point_cloud_board_planning",
            "label": f"{workface}-slot-{slot_index}",
            "workface": workface,
            "slot_index": slot_index,
            "planning_basis": planning_basis,
            "board_mode": point_cloud_result.get("board_mode"),
            "board_corner_ids": list(board.get("corner_ids") or []),
            "board_corners_xyz_mm": list(board.get("corners_xyz_mm") or []),
            "board_length_mm": board.get("length_mm"),
            "board_width_mm": board.get("width_mm"),
            "board_height_mean_mm": board.get("height_mean_mm"),
            "board_tilt_angle_deg": board.get("tilt_angle_deg"),
            "board_offset_angle_deg": board.get("offset_angle_deg"),
            "pad_required": pad_required,
            "remaining_workface_compensation_mm": compensation_mm,
            "first_workface_loading_plan": dict(first_plan) if isinstance(first_plan, Mapping) else None,
            "manual_target": manual_target or None,
        }
        if pad_required:
            result["instruction"] = (
                f"label_2 最后剩余作业面达到阈值，需要垫板；"
                f"补偿距离 {compensation_mm:.3f} mm。"
            )
        else:
            result["instruction"] = f"使用 {workface} 第 {slot_index} 个托盘位置。"
        return result
