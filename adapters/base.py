# -*- coding: utf-8 -*-
"""精简循环装载所需设备接口。真实设备接入时替换 adapter 即可。"""
from abc import ABC, abstractmethod
from typing import Sequence


class RobotAdapter(ABC):
    @abstractmethod
    def check_status(self) -> bool: ...

    @abstractmethod
    def move_to_pallet(self) -> dict: ...

    @abstractmethod
    def align_with_pallet(self, pallet_result: dict) -> dict: ...

    @abstractmethod
    def move_to_cargo(self, cargo: dict) -> dict: ...

    @abstractmethod
    def load_cargo_to_pallet(self, cargo: dict) -> dict: ...

    @abstractmethod
    def fork_pallet(self, pallet_result: dict, deviation_result: dict) -> dict: ...

    @abstractmethod
    def place_cargo_on_truck(self, cargo: dict, target: dict, corner_result: dict) -> dict: ...

    @abstractmethod
    def return_for_next_round(self) -> dict: ...


class CameraAdapter(ABC):
    @abstractmethod
    def check_status(self) -> bool: ...

    @abstractmethod
    def capture_pallet_rgbd(self) -> dict: ...

    @abstractmethod
    def capture_corner_images(self, cargo: dict, expected_corner_ids: Sequence[str] | None = None) -> dict: ...


class RadarAdapter(ABC):
    @abstractmethod
    def check_status(self) -> bool: ...

    @abstractmethod
    def capture_truck_point_cloud(self, cargo: dict) -> dict: ...

    @abstractmethod
    def measure_pallet_cargo_deviation(self, cargo: dict, phase: str) -> dict: ...

    @abstractmethod
    def monitor_truck_pallets(self, loaded_pallets: list, phase: str, sample_count: int = 5) -> dict: ...
