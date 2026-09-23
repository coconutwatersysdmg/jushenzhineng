from pathlib import Path

import numpy as np

from services.lab_dynamic_box_monitor_service import (
    LabDynamicBoxMonitorService,
    evaluate_selected_box_world,
    judge_box_world_region,
)
from services.module_evidence_service import ModuleEvidenceService
from services.sensor_calibration_service import SensorCalibrationService


def _region():
    return {
        "region_id": "B1",
        # Planner order is start-left, start-right, end-left, end-right.
        "corners_world_xyz_mm": [
            [0.0, 0.0, 0.0],
            [1000.0, 0.0, 0.0],
            [0.0, 1200.0, 0.0],
            [1000.0, 1200.0, 0.0],
        ],
    }


def test_lab_dynamic_monitor_has_separate_module_evidence_entries():
    module_ids = {row["module_id"] for row in ModuleEvidenceService().snapshot()}

    assert {"LAB_DYNAMIC_PRE_PLACE", "LAB_DYNAMIC_POST_PLACE"} <= module_ids


def test_world_region_requires_all_four_box_corners_inside():
    passed = judge_box_world_region(
        [[100, 100, 300], [900, 100, 300], [900, 1100, 300], [100, 1100, 300]],
        _region(),
    )
    failed = judge_box_world_region(
        [[100, 100, 300], [1050, 100, 300], [900, 1100, 300], [100, 1100, 300]],
        _region(),
    )

    assert passed["plan_check_status"] == "PASS"
    assert passed["inside_planned_region"] is True
    assert failed["plan_check_status"] == "OUT_OF_REGION"
    assert failed["inside_planned_region"] is False
    assert failed["outside_corners"] == ["P2"]


def test_selected_uv_corners_are_converted_to_world_before_region_judgement():
    selected = {
        "center_uv": [50.0, 60.0],
        "corners_uv": [[10.0, 10.0], [90.0, 10.0], [90.0, 110.0], [10.0, 110.0]],
    }
    world_by_uv = {
        (10.0, 10.0): [100.0, 100.0, 250.0],
        (90.0, 10.0): [900.0, 100.0, 250.0],
        (90.0, 110.0): [900.0, 1100.0, 250.0],
        (10.0, 110.0): [100.0, 1100.0, 250.0],
    }

    result = evaluate_selected_box_world(
        selected,
        _region(),
        lambda u, v: {
            "camera_xyz_mm": [u, v, 1000.0],
            "world_xyz_mm": world_by_uv[(u, v)],
            "depth_value": 1000.0,
        },
    )

    assert result["inside_planned_region"] is True
    assert result["corners_world_xyz_mm"][0] == [100.0, 100.0, 250.0]
    assert result["corners_camera_xyz_mm"][2] == [90.0, 110.0, 1000.0]


def test_capture_analysis_uses_rgb_detection_depth_and_world_region(tmp_path):
    rgb = tmp_path / "rgb.jpg"
    depth = tmp_path / "depth.png"
    rgb.write_bytes(b"rgb")
    depth.write_bytes(b"depth")

    def detector(_image, _config, planned_region_uv=None):
        assert planned_region_uv is None
        selected = {
            "center_uv": [50.0, 60.0],
            "corners_uv": [[10.0, 10.0], [90.0, 10.0], [90.0, 110.0], [10.0, 110.0]],
        }
        return (
            {"status": "OK", "box_count": 1, "selection_rule": "single_box", "all_boxes": [selected], "selected_box": selected},
            np.zeros((120, 100, 3), dtype=np.uint8),
            np.zeros((120, 100), dtype=np.uint8),
        )

    def writer(path, _image):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_bytes(b"artifact")
        return True

    service = LabDynamicBoxMonitorService(
        calibration=SensorCalibrationService(),
        detector=detector,
        image_reader=lambda _path: np.zeros((120, 100, 3), dtype=np.uint8),
        depth_sampler=lambda _path, _u, _v, _window: 1000.0,
        image_writer=writer,
        result_root=tmp_path / "results",
    )
    capture = {
        "success": True,
        "camera_id": "CAM_PICK",
        "rgb_path": str(rgb),
        "depth_path": str(depth),
        "depth_scale_mm": 1.0,
        "intrinsics": {"fx": 100.0, "fy": 100.0, "cx": 0.0, "cy": 0.0},
        "camera_world_pose": {
            "x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0,
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
        },
    }

    result = service.analyze_capture(capture, _region(), phase="post_place", result_tag="B1")

    assert result["success"] is True
    assert result["inside_planned_region"] is True
    assert result["planned_region_id"] == "B1"
    assert result["phase"] == "post_place"
    assert result["rgb_path"] == str(rgb)
    assert result["depth_path"] == str(depth)
    assert Path(result["result_image_path"]).is_file()
    assert Path(result["mask_image_path"]).is_file()


def test_capture_analysis_keeps_annotated_result_when_box_corner_depth_is_invalid(tmp_path):
    rgb = tmp_path / "rgb.jpg"
    depth = tmp_path / "depth.png"
    rgb.write_bytes(b"rgb")
    depth.write_bytes(b"depth")

    selected = {
        "center_uv": [50.0, 60.0],
        "corners_uv": [[10.0, 10.0], [90.0, 10.0], [90.0, 110.0], [10.0, 110.0]],
    }

    def detector(_image, _config, planned_region_uv=None):
        return (
            {"status": "OK", "box_count": 1, "selection_rule": "single_box", "all_boxes": [selected], "selected_box": selected},
            np.zeros((120, 100, 3), dtype=np.uint8),
            np.zeros((120, 100), dtype=np.uint8),
        )

    def writer(path, _image):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_bytes(b"artifact")
        return True

    service = LabDynamicBoxMonitorService(
        calibration=SensorCalibrationService(),
        detector=detector,
        image_reader=lambda _path: np.zeros((120, 100, 3), dtype=np.uint8),
        depth_sampler=lambda *_args: (_ for _ in ()).throw(RuntimeError("角点周围没有有效深度")),
        image_writer=writer,
        result_root=tmp_path / "results",
    )
    capture = {
        "success": True,
        "camera_id": "CAM_PICK",
        "rgb_path": str(rgb),
        "depth_path": str(depth),
        "depth_scale_mm": 1.0,
        "intrinsics": {"fx": 100.0, "fy": 100.0, "cx": 0.0, "cy": 0.0},
        "camera_world_pose": {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0},
    }

    result = service.analyze_capture(capture, _region(), phase="post_place", result_tag="B1")

    assert result["success"] is False
    assert result["status"] == "DEPTH_INVALID"
    assert result["corners_uv"] == selected["corners_uv"]
    assert "没有有效深度" in result["message"]
    assert Path(result["result_image_path"]).is_file()
