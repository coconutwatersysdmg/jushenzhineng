# -*- coding: utf-8 -*-
from __future__ import annotations
from copy import deepcopy
from typing import Any, Dict, Mapping


class PlacementCompensationService:
    """把区域名义放置点、历史反馈、8.2临近托盘补偿组合成当前PLC目标点。

    10.2 货物-托盘偏移是相机图像局部横向量，除非已经完成该相机的方向标定，
    不直接把这个标量硬加到 WORLD X/Y；它单独发送PLC，避免坐标方向错误。
    """

    @staticmethod
    def _vec(src: Mapping[str, Any] | None) -> Dict[str, float]:
        src=src or {}
        return {"dx":float(src.get("dx",0.0) or 0.0),"dy":float(src.get("dy",0.0) or 0.0),"dz":float(src.get("dz",0.0) or 0.0),"dyaw_deg":float(src.get("dyaw_deg",0.0) or 0.0)}

    def combine_current_target(self, nominal_pose: Mapping[str, Any], board_comp=None, persistent_feedback=None, neighbor_comp=None, first=False) -> Dict[str, Any]:
        base={k:float(nominal_pose.get(k,0.0) or 0.0) for k in ("x_mm","y_mm","z_mm","roll_deg","pitch_deg","yaw_deg")}
        b=self._vec(board_comp); p=self._vec({} if first else persistent_feedback); n=self._vec(neighbor_comp)
        final=deepcopy(base)
        final["x_mm"] += b["dx"]+p["dx"]+n["dx"]
        final["y_mm"] += b["dy"]+p["dy"]+n["dy"]
        final["z_mm"] += b["dz"]+p["dz"]+n["dz"]
        final["yaw_deg"] += b["dyaw_deg"]+p["dyaw_deg"]+n["dyaw_deg"]
        return {"nominal_world_pose":base,"board_compensation_world_mm":b,"persistent_feedback_world_mm":p,"neighbor_compensation_world_mm":n,"final_world_pose":final}

    def next_feedback_from_region(self, region_result: Mapping[str, Any]) -> Dict[str, float]:
        return self._vec(region_result.get("next_pallet_compensation_world_mm") or {})
