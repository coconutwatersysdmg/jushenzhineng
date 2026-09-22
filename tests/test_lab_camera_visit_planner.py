from pathlib import Path

import pytest

from algorithm_modules.lab.cam_yolo_lab.camera_world_module import CameraWorldTransform
from services.lab_camera_visit_planner import LabCameraVisitPlanner


ROOT = Path(__file__).resolve().parents[1]
CAMERA_EXTRINSIC = ROOT / "config" / "camera_extrinsic.json"


def test_lab_visit_plan_uses_radar_centers_for_plc_xy_and_keeps_planning_r():
    planner = LabCameraVisitPlanner(CameraWorldTransform.from_json(CAMERA_EXTRINSIC))
    world_points = {
        "P1": {"x": 100, "y": 800, "z": 1200},
        "P2": {"x": 300, "y": 800, "z": 1200},
        "P3": {"x": 500, "y": 200, "z": 1200},
        "P4": {"x": 700, "y": 200, "z": 1200},
    }
    targets = planner.build_pair_targets(
        world_points,
        {"x": 0, "y": 0, "z": 380, "r": -75},
    )

    assert [item["pair"] for item in targets] == [("P3", "P4"), ("P1", "P2")]
    expected_x = []
    for pair in (("P3", "P4"), ("P1", "P2")):
        center = tuple(
            (world_points[pair[0]][axis] + world_points[pair[1]][axis]) / 2.0
            for axis in ("x", "y", "z")
        )
        expected_x.append(
            planner.transform.plc_xy_for_camera_axis_target(
                center, {"x": 0.0, "y": 0.0, "z": 380.0, "r": -75.0}
            )["X"]
        )
    assert [item["plc_command"]["X"] for item in targets] == pytest.approx(expected_x)
    assert targets[0]["plc_command"]["X"] != pytest.approx(230.0)
    assert targets[0]["plc_command"]["X"] != pytest.approx(targets[1]["plc_command"]["X"])
    assert all(item["plc_command"]["Z"] == 380.0 for item in targets)
    assert all(item["plc_command"]["R"] == -75.0 for item in targets)


def test_camera_axis_inverse_rejects_target_behind_camera():
    transform = CameraWorldTransform.from_json(CAMERA_EXTRINSIC)

    with pytest.raises(ValueError, match="光轴后方"):
        transform.plc_xy_for_camera_axis_target(
            (0, 0, -10000), {"x": 0, "y": 0, "z": 380, "r": -80}
        )


def test_lab_region_visit_targets_geometric_center_and_keeps_startup_r():
    planner = LabCameraVisitPlanner(CameraWorldTransform.from_json(CAMERA_EXTRINSIC))

    target = planner.build_world_target(
        [500.0, 600.0, 1450.0],
        {"x": 0.0, "y": 0.0, "z": 380.0, "r": -73.5},
        task="LAB_REGION_B1",
    )

    expected = planner.transform.plc_xy_for_camera_axis_target(
        (500.0, 600.0, 1450.0),
        {"x": 0.0, "y": 0.0, "z": 380.0, "r": -73.5},
    )
    assert target["target_world"] == [500.0, 600.0, 1450.0]
    assert target["plc_command"]["X"] == pytest.approx(expected["X"])
    assert target["plc_command"]["Y"] == pytest.approx(expected["Y"])
    assert target["plc_command"]["Z"] == 380.0
    assert target["plc_command"]["R"] == -73.5
