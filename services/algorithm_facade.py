# -*- coding: utf-8 -*-
from __future__ import annotations
from copy import deepcopy

import config.feature_switches as feature_switches
from services.pallet_hole_recognition_service import PalletHoleRecognitionService
from services.pallet_cargo_offset_service import PalletCargoOffsetService
from services.point_cloud_processing_service import PointCloudProcessingService
from services.corner_recognition_service import CornerRecognitionService
from services.lab_lidar_adapter import process_lab_lidar_pcd
from services.lab_camera_adapter import LabCameraCornerService


class AlgorithmFacade:
    """已接入算法的统一门面。

    注意：雷达 point_cloud 的 board_mode / loading_plan 不再作为最终装载规划依据；
    它们即使由旧模块返回，也只保存在原始结果中。最终板型和区域由相机 WORLD 角点计算。

    实验室/现场算法由 config.feature_switches 两个独立开关控制。
    """
    def __init__(self):
        self.pallet_hole = PalletHoleRecognitionService()
        self.offset = PalletCargoOffsetService()
        self.point_cloud = PointCloudProcessingService()
        self.corner = CornerRecognitionService()
        self._lab_camera: LabCameraCornerService | None = None

    def _lab_camera_service(self) -> LabCameraCornerService:
        if self._lab_camera is None:
            self._lab_camera = LabCameraCornerService()
        return self._lab_camera

    def pre_pick_offset(self, image_path: str, cargo: dict):
        return self.offset.detect(image_path, phase="pre_pick", cargo=cargo, result_tag=cargo.get("instance_id", "cargo"))

    def post_place_offset(self, image_path: str, cargo: dict):
        return self.offset.detect(image_path, phase="post_place", cargo=cargo, result_tag=cargo.get("instance_id", "cargo"))

    def pallet_hole_recognize(self, rgb_path: str, depth_path: str, cargo: dict):
        return self.pallet_hole.recognize(rgb_path, depth_path, result_tag=cargo.get("instance_id", "cargo"))

    def radar_process(self, locate_result: dict, cargo: dict):
        if locate_result.get("pcd_path"):
            if feature_switches.USE_LAB_LIDAR_ALGO:
                result = process_lab_lidar_pcd(
                    locate_result["pcd_path"],
                    result_tag=cargo.get("instance_id", "cargo"),
                )
                result["radar_raw_board_analysis_reference_only"] = {
                    "board_mode": "lab_flat_4_corners",
                    "length_mm": result.get("length_mm"),
                    "width_mm": result.get("width_mm"),
                }
                return result
            result = self.point_cloud.process_file(locate_result["pcd_path"], result_tag=cargo.get("instance_id", "cargo"))
            result["radar_raw_board_analysis_reference_only"] = {
                "board_mode": result.get("board_mode"),
                "boards": deepcopy(result.get("boards")),
                "first_workface_loading_plan": deepcopy(result.get("first_workface_loading_plan")),
            }
            return result
        return deepcopy(locate_result)

    def corner_image_recognize(self, image_paths_by_point: dict, corner_ids: list[str], cargo: dict):
        return self.corner.recognize_visual_only(
            image_source=image_paths_by_point,
            expected_corner_ids=corner_ids,
            result_tag=cargo.get("instance_id", "cargo"),
        )

    def lab_corner_world_recognize(
        self,
        corner_ids: list[str],
        capture_meta: dict,
        group_captures: dict | None = None,
    ):
        """实验室相机：YOLO+深度+外参，直接输出 WORLD。"""
        return self._lab_camera_service().locate_from_capture_meta(
            corner_ids=corner_ids,
            capture_meta=capture_meta,
            group_captures=group_captures,
        )

    def lab_pallet_hole_world_recognize(self, capture: dict, plc_pose: dict, cargo: dict):
        """实验室插孔：复用 cam_yolo_lab 双目标检测，直接输出 WORLD。"""
        return self._lab_camera_service().locate_pallet_holes(
            capture["rgb_path"],
            capture["depth_path"],
            plc_pose,
            depth_scale_mm=float(capture.get("depth_scale_mm", 1.0) or 1.0),
        )

    def lab_camera_transform(self):
        """Return the lab-only camera/PLC/WORLD transform used by visit planning."""
        return self._lab_camera_service().transform
