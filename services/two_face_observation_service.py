# -*- coding: utf-8 -*-
from __future__ import annotations
from copy import deepcopy


class TwoFaceObservationService:
    """WORLD: X向右，Y车辆前进，Z向上。

    唯一装载臂沿双侧龙门架到当前货物附近，随后从目标所在侧向车板内侧伸入，拍两个相邻面。
    相机固定挂在机械臂上，移动命令仍发送给机械臂/PLC。
    """
    def __init__(self, twin, robot, camera, inward_mm: float = 420.0, robot_id: str = "PICK_ARM", camera_id: str = "CAM_PICK"):
        self.twin=twin; self.robot=robot; self.camera=camera; self.inward_mm=float(inward_mm)
        self.robot_id=str(robot_id); self.camera_id=str(camera_id)

    def observe(self, cargo: dict, target: dict):
        pose=target.get("final_world_pose") or {}
        tx=float(pose.get("x_mm",0.0) or 0.0); ty=float(pose.get("y_mm",0.0) or 0.0); tz=float(pose.get("z_mm",0.0) or 0.0)
        column=str((target or {}).get("column") or "").strip().upper()
        truck_pose=((self.twin.snapshot().get("truck") or {}).get("pose") or {})
        truck_x=float(truck_pose.get("x_mm",0.0) or 0.0)
        right_side=(tx>truck_x) if ("x_mm" in pose or "x" in pose) else column=="B"
        outer_x,sign,yaw=(3500.0,-1.0,90.0) if right_side else (-3500.0,+1.0,-90.0)
        selected_side="RIGHT" if right_side else "LEFT"
        base={"x_mm":outer_x,"y_mm":ty,"z_mm":tz+500.0,"roll_deg":0,"pitch_deg":-10,"yaw_deg":yaw}
        self.robot.move_tool_world(self.robot_id,base,task="OBSERVE_FACE_A")
        a=self.camera.capture_rgb(self.camera_id,tag="face_a")
        inner=deepcopy(base); inner["x_mm"]=outer_x+sign*self.inward_mm
        self.robot.move_tool_world(self.robot_id,inner,task="EXTEND_INWARD_OBSERVE_FACE_B")
        b=self.camera.capture_rgb(self.camera_id,tag="face_b")
        return {"success":True,"robot_id":self.robot_id,"camera_id":self.camera_id,"selected_side":selected_side,"face_a":a,"face_b":b,"message":f"唯一装载臂在{'右' if right_side else '左'}侧车外完成两个相邻面观测；保持当前位置，第11步完成后再由第12步返回车尾"}
