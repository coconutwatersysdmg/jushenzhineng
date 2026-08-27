# -*- coding: utf-8 -*-
from __future__ import annotations
from abc import ABC, abstractmethod


class PLCAdapter(ABC):
    @abstractmethod
    def connect(self): ...
    @abstractmethod
    def send_command(self, command: str, payload: dict) -> dict: ...
    @abstractmethod
    def wait_ack(self, command_id: str, timeout_ms: int = 5000) -> dict: ...
    @abstractmethod
    def send_message(self, module: str, status: str, message: str, data: dict | None = None) -> dict: ...


class RobotAdapter(ABC):
    @abstractmethod
    def move_tool_world(self, robot_id: str, pose: dict, task: str = "") -> dict: ...
    @abstractmethod
    def retract(self, robot_id: str) -> dict: ...
    @abstractmethod
    def fork_pallet(self, pallet_result: dict) -> dict: ...
    @abstractmethod
    def place(self, cargo: dict, target: dict) -> dict: ...


class RadarAdapter(ABC):
    @abstractmethod
    def locate_truck(self, cargo: dict) -> dict: ...


class ArmCameraAdapter(ABC):
    """相机硬件接口。相机不拥有独立运动机构，位姿由父机械臂决定。"""
    @abstractmethod
    def capture_rgbd(self, camera_id: str, tag: str = "") -> dict: ...
    @abstractmethod
    def capture_rgb(self, camera_id: str, tag: str = "") -> dict: ...
