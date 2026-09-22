# -*- coding: utf-8 -*-
from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, List, Mapping

from services.placement_compensation_service import PlacementCompensationService


class SpaceManager:
    """相机几何驱动的可用空间管理。

    - 首轮：由 CameraBoardGeometryService 产生 regions；
    - 后续：不再跑雷达/角点，直接维护这些区域的 AVAILABLE/OCCUPIED 状态；
    - 8.2 先对 tentative target 的临近托盘拍照；
    - 8.3 再把邻近补偿 + 历史反馈写入本轮最终 target。
    """

    def __init__(self):
        self.regions: List[Dict[str, Any]] = []
        self.board_geometry: Dict[str, Any] = {}
        self.persistent_feedback = {"dx":0.0,"dy":0.0,"dz":0.0,"dyaw_deg":0.0}
        self.current_target: Dict[str, Any] = {}
        self.comp = PlacementCompensationService()

    def reset(self):
        self.regions=[]; self.board_geometry={}; self.persistent_feedback={"dx":0.0,"dy":0.0,"dz":0.0,"dyaw_deg":0.0}; self.current_target={}

    def initialize_from_camera_geometry(self, geometry: Mapping[str, Any]) -> Dict[str, Any]:
        if not geometry.get("success"):
            raise RuntimeError("相机车板几何结果无效")
        regions=deepcopy(geometry.get("regions") or [])
        if not regions:
            raise RuntimeError("相机车板几何未生成任何完整 1.2m 装载区域")
        for r in regions:
            r.setdefault("status","AVAILABLE"); r.setdefault("cargo_id",None)
        self.regions=regions; self.board_geometry=deepcopy(dict(geometry)); self.current_target={}
        return self.snapshot()

    def _next_region(self) -> Dict[str, Any]:
        region=next((r for r in self.regions if r.get("status")=="AVAILABLE"),None)
        if region is None:
            raise RuntimeError("当前相机规划车板已无可用装载区域")
        return region

    def peek_next_target(self, cargo: Mapping[str, Any] | None = None) -> Dict[str, Any]:
        r=self._next_region()
        c=r.get("center_world_xyz_mm") or [0,0,0]
        sections=self.board_geometry.get("sections") or []
        yaw=0.0
        if sections:
            # 区域所属 section 的相机几何偏航优先。
            section_name=r.get("section")
            match=next((x for x in sections if x.get("name")==section_name),None)
            if match: yaw=float(match.get("yaw_deg",0.0) or 0.0)
            else: yaw=float(sections[0].get("yaw_deg",0.0) or 0.0)
        board_comp={"dx":0.0,"dy":0.0,"dz":0.0,"dyaw_deg":0.0}
        if r.get("requires_support_block"):
            # 支撑块是区域状态要求，不把高度差重复加进货物Z；区域center已经是最终支撑面高度。
            board_comp["dz"]=0.0
        return {
            "region_id":r.get("region_id"),"blind_code":r.get("blind_code"),"section":r.get("section"),"column":r.get("column"),
            "requires_support_block":bool(r.get("requires_support_block")),
            "support_height_compensation_mm":float(r.get("support_height_compensation_mm",0.0) or 0.0),
            "nominal_world_pose":{"x_mm":float(c[0]),"y_mm":float(c[1]),"z_mm":float(c[2]),"roll_deg":0.0,"pitch_deg":0.0,"yaw_deg":yaw},
            "board_compensation_world_mm":board_comp,
        }

    def confirm_target(self, neighbor_compensation: Mapping[str,Any] | None = None, first: bool = False, cargo: Mapping[str,Any] | None = None) -> Dict[str, Any]:
        tentative=self.peek_next_target(cargo)
        combo=self.comp.combine_current_target(
            tentative["nominal_world_pose"],
            board_comp=tentative.get("board_compensation_world_mm"),
            persistent_feedback=self.persistent_feedback,
            neighbor_comp=(neighbor_compensation or {}).get("compensation_world_mm") if isinstance(neighbor_compensation,Mapping) else neighbor_compensation,
            first=first,
        )
        target={**tentative,**combo}
        self.current_target=deepcopy(target)
        return target

    def set_persistent_feedback(self, feedback: Mapping[str,Any] | None):
        f=feedback or {}
        self.persistent_feedback={"dx":float(f.get("dx",0.0) or 0.0),"dy":float(f.get("dy",0.0) or 0.0),"dz":float(f.get("dz",0.0) or 0.0),"dyaw_deg":float(f.get("dyaw_deg",0.0) or 0.0)}

    def occupy_current(self, cargo_id: str) -> Dict[str, Any]:
        rid=(self.current_target or {}).get("region_id")
        if not rid: raise RuntimeError("没有锁定的当前放置区域")
        for r in self.regions:
            if r.get("region_id")==rid:
                r["status"]="OCCUPIED"; r["cargo_id"]=str(cargo_id); return deepcopy(r)
        raise KeyError(rid)

    def occupy_region_if_passed(
        self,
        region_id: str,
        cargo_id: str,
        monitor_result: Mapping[str, Any],
    ) -> Dict[str, Any] | None:
        """实验室手动放货：只有四角 WORLD 判定通过才占用该区域。"""
        if not bool((monitor_result or {}).get("inside_planned_region")):
            return None
        target = str(region_id)
        for region in self.regions:
            if str(region.get("region_id")) == target:
                region["status"] = "OCCUPIED"
                region["cargo_id"] = str(cargo_id)
                return deepcopy(region)
        raise KeyError(target)

    def snapshot(self) -> Dict[str, Any]:
        return {
            "board_mode":self.board_geometry.get("board_mode","unknown"),
            "camera_board_geometry":deepcopy(self.board_geometry),
            "regions":deepcopy(self.regions),
            "available":[deepcopy(r) for r in self.regions if r.get("status")=="AVAILABLE"],
            "occupied":[deepcopy(r) for r in self.regions if r.get("status")=="OCCUPIED"],
            "current_target":deepcopy(self.current_target),
            "persistent_feedback_world_mm":deepcopy(self.persistent_feedback),
            "borrow_plan":deepcopy(self.board_geometry.get("borrow_plan")),
        }
