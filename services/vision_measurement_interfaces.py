# -*- coding: utf-8 -*-
from __future__ import annotations

"""尚未提供正式视觉模型的两个测量接口。

8.2 临近托盘姿态、10.1 托盘-车板区域偏差目前只定义了系统接口与数据格式。
如果 UI/PLC 提供 debug_measurement，则直接消费真实/离线算法结果；没有结果时，
联调模式返回 0 补偿并明确标记 source=demo_zero，不伪装成视觉算法输出。
"""

from pathlib import Path
from typing import Any, Dict, Mapping


class NeighborPalletPoseService:
    def analyze(self, image_path: str, target: Mapping[str, Any], debug_measurement: Mapping[str, Any] | None = None) -> Dict[str, Any]:
        if image_path and not Path(image_path).is_file():
            raise RuntimeError(f"8.2 临近托盘姿态图片不存在：{image_path}")
        if debug_measurement:
            dx=float(debug_measurement.get("dx_mm",0.0)); dy=float(debug_measurement.get("dy_mm",0.0)); dz=float(debug_measurement.get("dz_mm",0.0)); yaw=float(debug_measurement.get("yaw_deg",0.0))
            return {"success":True,"source":"external_or_debug_measurement","image_path":image_path,"dx_mm":dx,"dy_mm":dy,"dz_mm":dz,"yaw_deg":yaw,"compensation_world_mm":{"dx":-dx,"dy":-dy,"dz":-dz,"dyaw_deg":-yaw},"message":"已依据临近托盘姿态计算当前放置补偿"}
        return {"success":True,"source":"demo_zero_no_neighbor_pose_algorithm","image_path":image_path,"dx_mm":0.0,"dy_mm":0.0,"dz_mm":0.0,"yaw_deg":0.0,"compensation_world_mm":{"dx":0.0,"dy":0.0,"dz":0.0,"dyaw_deg":0.0},"message":"8.2 视觉算法尚未接入，联调模式补偿为0"}


class PalletBoardRegionDeviationService:
    def analyze(self, image_path: str, target: Mapping[str, Any], debug_measurement: Mapping[str, Any] | None = None) -> Dict[str, Any]:
        if image_path and not Path(image_path).is_file():
            raise RuntimeError(f"10.1 托盘-车板区域偏差图片不存在：{image_path}")
        if debug_measurement:
            dx=float(debug_measurement.get("dx_mm",0.0)); dy=float(debug_measurement.get("dy_mm",0.0)); dz=float(debug_measurement.get("dz_mm",0.0)); yaw=float(debug_measurement.get("yaw_deg",0.0))
            return {"success":True,"source":"external_or_debug_measurement","image_path":image_path,"dx_mm":dx,"dy_mm":dy,"dz_mm":dz,"yaw_deg":yaw,"next_pallet_compensation_world_mm":{"dx":-dx,"dy":-dy,"dz":-dz,"dyaw_deg":-yaw},"message":"已计算托盘相对划分区域的偏差与下一托盘补偿"}
        return {"success":True,"source":"demo_zero_no_region_deviation_algorithm","image_path":image_path,"dx_mm":0.0,"dy_mm":0.0,"dz_mm":0.0,"yaw_deg":0.0,"next_pallet_compensation_world_mm":{"dx":0.0,"dy":0.0,"dz":0.0,"dyaw_deg":0.0},"message":"10.1 视觉算法尚未接入，联调模式补偿为0"}
