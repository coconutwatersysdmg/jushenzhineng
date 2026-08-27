# -*- coding: utf-8 -*-
from __future__ import annotations

"""基于相机最终 WORLD 角点判断车板几何、板型与装载区域。

最终板型/区域判断不读取雷达的 board_mode、loading_plan 等字段。
雷达只提供首轮 4/6 个粗搜索点；本模块只接收相机最终世界坐标。

角点拓扑：
- 4 点平板：P1/P2 为起始横边，P3/P4 为末端横边。
- 6 点候选高低板：P1/P2 为第一端，P3/P4 为高低交界，P5/P6 为第二端。
  是否确实为高低板由相机最终点的两段平均高度差判断，而不是由雷达判断。
"""

from copy import deepcopy
from math import atan2, degrees, floor
from typing import Any, Dict, Iterable, List, Mapping, Sequence

import numpy as np


def _p(v: Any) -> np.ndarray:
    if isinstance(v, Mapping):
        return np.asarray([
            float(v.get("x", v.get("x_mm", 0.0))),
            float(v.get("y", v.get("y_mm", 0.0))),
            float(v.get("z", v.get("z_mm", 0.0))),
        ], dtype=np.float64)
    return np.asarray(v[:3], dtype=np.float64)


def _json_point(p: np.ndarray) -> List[float]:
    return [round(float(x), 3) for x in p]


def _lerp(a: np.ndarray, b: np.ndarray, f: float) -> np.ndarray:
    return a + (b-a) * float(f)


class CameraBoardGeometryService:
    def __init__(
        self,
        row_length_mm: float = 1200.0,
        high_low_height_threshold_mm: float = 80.0,
        max_borrow_mm: float = 650.0,
        min_second_section_remaining_mm: float = 600.0,
    ):
        self.row_length_mm = float(row_length_mm)
        self.high_low_height_threshold_mm = float(high_low_height_threshold_mm)
        self.max_borrow_mm = float(max_borrow_mm)
        self.min_second_section_remaining_mm = float(min_second_section_remaining_mm)

    @staticmethod
    def _segment_geometry(start_left, start_right, end_left, end_right, name: str) -> Dict[str, Any]:
        sl, sr, el, er = map(_p, (start_left, start_right, end_left, end_right))
        left_len = float(np.linalg.norm(el-sl))
        right_len = float(np.linalg.norm(er-sr))
        length = 0.5*(left_len+right_len)
        width0 = float(np.linalg.norm(sr-sl))
        width1 = float(np.linalg.norm(er-el))
        width = 0.5*(width0+width1)
        front = 0.5*(sl+sr); rear = 0.5*(el+er)
        forward = rear-front
        horizontal = float(np.linalg.norm(forward[:2]))
        tilt = 0.0 if horizontal < 1e-9 else degrees(atan2(float(forward[2]), horizontal))
        # WORLD：X向右，Y车辆前进；相对 +Y 的偏航角。
        yaw = degrees(atan2(float(forward[0]), float(forward[1]))) if horizontal >= 1e-9 else 0.0
        points = np.stack([sl,sr,el,er])
        return {
            "name": name,
            "corners_world_xyz_mm": [_json_point(x) for x in points],
            "length_mm": round(length, 3),
            "width_mm": round(width, 3),
            "height_mean_mm": round(float(points[:,2].mean()), 3),
            "tilt_angle_deg": round(float(tilt), 6),
            "yaw_deg": round(float(yaw), 6),
            "_sl": sl, "_sr": sr, "_el": el, "_er": er,
        }

    def _section_regions(
        self,
        geom: Dict[str, Any],
        section_name: str,
        start_distance_mm: float = 0.0,
        end_distance_mm: float | None = None,
    ) -> Dict[str, Any]:
        L = float(geom["length_mm"])
        start_d = max(0.0, float(start_distance_mm))
        end_d = L if end_distance_mm is None else min(L, max(start_d, float(end_distance_mm)))
        usable = max(0.0, end_d-start_d)
        count = int(floor(usable/self.row_length_mm + 1e-9))
        sl, sr, el, er = geom["_sl"], geom["_sr"], geom["_el"], geom["_er"]
        regions = []
        for i in range(count):
            d0 = start_d + i*self.row_length_mm
            d1 = d0 + self.row_length_mm
            f0, f1 = d0/L, d1/L
            a0, b0 = _lerp(sl,el,f0), _lerp(sr,er,f0)
            a1, b1 = _lerp(sl,el,f1), _lerp(sr,er,f1)
            m0, m1 = 0.5*(a0+b0), 0.5*(a1+b1)
            for col, corners in (
                ("A", [a0,m0,a1,m1]),
                ("B", [m0,b0,m1,b1]),
            ):
                arr = np.stack(corners)
                regions.append({
                    "section": section_name,
                    "local_row_index": i+1,
                    "column": col,
                    "row_length_mm": self.row_length_mm,
                    "corners_world_xyz_mm": [_json_point(x) for x in arr],
                    "center_world_xyz_mm": _json_point(arr.mean(axis=0)),
                    "status": "AVAILABLE",
                    "cargo_id": None,
                    "requires_support_block": False,
                    "support_height_compensation_mm": 0.0,
                })
        remainder = usable-count*self.row_length_mm
        return {"regions": regions, "full_row_count": count, "remaining_length_mm": round(remainder,3), "usable_length_mm": round(usable,3)}

    def _borrow_plan(self, first: Dict[str, Any], second: Dict[str, Any]) -> Dict[str, Any]:
        L1, L2 = float(first["length_mm"]), float(second["length_mm"])
        n1 = int(floor(L1/self.row_length_mm + 1e-9))
        rem1 = L1-n1*self.row_length_mm
        if rem1 <= 1e-6:
            return {"enabled": False, "reason": "第一段无剩余长度", "remaining_first_mm": 0.0}
        need = self.row_length_mm-rem1
        if need > self.max_borrow_mm:
            return {"enabled": False, "reason": "最小借位超过配置上限", "remaining_first_mm": round(rem1,3), "required_borrow_mm": round(need,3), "max_borrow_mm": self.max_borrow_mm}
        if L2-need < self.min_second_section_remaining_mm:
            return {"enabled": False, "reason": "借位后第二段剩余空间过小", "remaining_first_mm": round(rem1,3), "required_borrow_mm": round(need,3), "second_remaining_after_borrow_mm": round(L2-need,3)}
        # 目标函数的实际结果：能新增一整排(左右两格)的前提下，取最小借位 need。
        return {
            "enabled": True,
            "optimization": "min_borrow_that_adds_one_full_1200mm_row",
            "remaining_first_mm": round(rem1,3),
            "required_borrow_mm": round(need,3),
            "borrowed_from_second_mm": round(need,3),
            "added_regions": 2,
            "support_height_compensation_mm": round(abs(float(first["height_mean_mm"])-float(second["height_mean_mm"])),3),
        }

    def _borrow_regions(self, first: Dict[str, Any], second: Dict[str, Any], borrow: Dict[str, Any]) -> List[Dict[str, Any]]:
        if not borrow.get("enabled"):
            return []
        L1 = float(first["length_mm"]); L2 = float(second["length_mm"])
        rem1 = float(borrow["remaining_first_mm"]); need = float(borrow["borrowed_from_second_mm"])
        # 起点位于第一段最后 rem1 的起点，终点位于第二段起点后 need。
        f0 = (L1-rem1)/L1
        sl1,sr1,el1,er1 = first["_sl"],first["_sr"],first["_el"],first["_er"]
        sl2,sr2,el2,er2 = second["_sl"],second["_sr"],second["_el"],second["_er"]
        a0,b0 = _lerp(sl1,el1,f0), _lerp(sr1,er1,f0)
        a1,b1 = _lerp(sl2,el2,need/L2), _lerp(sr2,er2,need/L2)
        m0,m1 = 0.5*(a0+b0),0.5*(a1+b1)
        z_target = max(float(first["height_mean_mm"]), float(second["height_mean_mm"]))
        support = float(borrow.get("support_height_compensation_mm",0.0))
        out=[]
        for col,corners in (("A",[a0,m0,a1,m1]),("B",[m0,b0,m1,b1])):
            arr=np.stack(corners)
            center=arr.mean(axis=0); center[2]=z_target
            out.append({
                "section":"BORROWED_BOUNDARY","local_row_index":1,"column":col,"row_length_mm":self.row_length_mm,
                "corners_world_xyz_mm":[_json_point(x) for x in arr],"center_world_xyz_mm":_json_point(center),
                "status":"AVAILABLE","cargo_id":None,"requires_support_block":True,
                "support_height_compensation_mm":round(support,3),
            })
        return out

    @staticmethod
    def _assign_blind_codes(regions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        # WORLD +Y is the truck-head direction. Loading must start at the head
        # and progress toward the tail, with A/B forming each transverse row.
        regions.sort(key=lambda r: (
            -float((r.get("center_world_xyz_mm") or [0, 0, 0])[1]),
            str(r.get("column") or ""),
        ))
        row_no = 0
        last_key = None
        for r in regions:
            center = r.get("center_world_xyz_mm") or [0, 0, 0]
            key = round(float(center[1]), 3)
            if key != last_key:
                row_no += 1; last_key = key
            r["blind_code"] = f"{r['column']}{row_no:02d}"
            r["region_id"] = r["blind_code"]
        return regions

    @staticmethod
    def _public_geom(g: Dict[str, Any]) -> Dict[str, Any]:
        return {k: deepcopy(v) for k,v in g.items() if not str(k).startswith("_")}

    def analyze(self, world_points: Dict[str, Any], corner_ids: Sequence[str]) -> Dict[str, Any]:
        ids = [str(x) for x in corner_ids]
        if len(ids) not in {4,6}:
            raise RuntimeError(f"相机板型判断只支持 4/6 点，当前 {ids}")
        if any(i not in world_points for i in ids):
            raise RuntimeError("相机最终世界角点不完整")

        if len(ids)==4:
            g = self._segment_geometry(world_points[ids[0]],world_points[ids[1]],world_points[ids[2]],world_points[ids[3]],"FLAT")
            div = self._section_regions(g,"FLAT")
            regions = self._assign_blind_codes(div["regions"])
            return {
                "success":True,"decision_source":"camera_world_corners","board_mode":"flat","is_high_low":False,
                "height_difference_mm":0.0,"sections":[self._public_geom(g)],"regions":regions,"borrow_plan":None,
                "remaining_length_mm":div["remaining_length_mm"],"row_length_mm":self.row_length_mm,
                "message":"相机最终 4 角点判定为平板并完成两列×1.2m区域划分",
            }

        # 6点拓扑：第一段 P1/P2->P3/P4，第二段 P3/P4->P5/P6。
        g1 = self._segment_geometry(world_points[ids[0]],world_points[ids[1]],world_points[ids[2]],world_points[ids[3]],"SECTION_1")
        g2 = self._segment_geometry(world_points[ids[2]],world_points[ids[3]],world_points[ids[4]],world_points[ids[5]],"SECTION_2")
        first_plateau_z = 0.5*(_p(world_points[ids[0]])[2] + _p(world_points[ids[1]])[2])
        second_plateau_z = 0.5*(_p(world_points[ids[4]])[2] + _p(world_points[ids[5]])[2])
        dz = abs(float(first_plateau_z-second_plateau_z))
        is_high_low = dz >= self.high_low_height_threshold_mm

        if not is_high_low:
            # 雷达给了 6 个搜索点，但视觉高度差不足，最终按相机结果合并为平板。
            gm = self._segment_geometry(world_points[ids[0]],world_points[ids[1]],world_points[ids[4]],world_points[ids[5]],"FLAT_MERGED")
            div = self._section_regions(gm,"FLAT")
            regions = self._assign_blind_codes(div["regions"])
            return {
                "success":True,"decision_source":"camera_world_corners","board_mode":"flat","is_high_low":False,
                "height_difference_mm":round(dz,3),"high_low_threshold_mm":self.high_low_height_threshold_mm,
                "sections":[self._public_geom(gm)],"regions":regions,"borrow_plan":None,
                "remaining_length_mm":div["remaining_length_mm"],"row_length_mm":self.row_length_mm,
                "message":f"雷达提供6个粗点，但相机最终两段高度差仅 {dz:.1f} mm，按平板处理",
            }

        # 高低板：第一/第二段谁高由相机高度决定。
        high = "SECTION_1" if float(first_plateau_z) >= float(second_plateau_z) else "SECTION_2"
        low = "SECTION_2" if high=="SECTION_1" else "SECTION_1"
        borrow = self._borrow_plan(g1,g2)
        if borrow.get("enabled"):
            borrow["support_height_compensation_mm"] = round(float(dz),3)
        div1 = self._section_regions(g1,"SECTION_1",0.0,float(g1["length_mm"])-float(borrow.get("remaining_first_mm",0.0)) if borrow.get("enabled") else None)
        regions = list(div1["regions"])
        regions.extend(self._borrow_regions(g1,g2,borrow))
        second_offset = float(borrow.get("borrowed_from_second_mm",0.0)) if borrow.get("enabled") else 0.0
        div2 = self._section_regions(g2,"SECTION_2",second_offset,None)
        regions.extend(div2["regions"])
        regions = self._assign_blind_codes(regions)
        return {
            "success":True,"decision_source":"camera_world_corners","board_mode":"high_low","is_high_low":True,
            "height_difference_mm":round(dz,3),"high_low_threshold_mm":self.high_low_height_threshold_mm,
            "high_section":high,"low_section":low,"sections":[self._public_geom(g1),self._public_geom(g2)],
            "regions":regions,"borrow_plan":borrow,"row_length_mm":self.row_length_mm,
            "message":f"相机最终6角点判定高低板，高度差 {dz:.1f} mm；区域由相机世界坐标生成",
        }
