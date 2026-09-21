from pathlib import Path

import pytest

from algorithm_modules.lab.cam_yolo_lab.camera_world_module import CameraWorldTransform
from services.lab_camera_visit_planner import LabCameraVisitPlanner


ROOT = Path(__file__).resolve().parents[1]
CAMERA_EXTRINSIC = ROOT / "config" / "camera_extrinsic.json"


def test_lab_visit_plan_uses_p3p4_then_p1p2_and_fixed_camera_pose():
    planner = LabCameraVisitPlanner(CameraWorldTransform.from_json(CAMERA_EXTRINSIC))
    targets = planner.build_pair_targets(
        {
            "P1": {"x": 100, "y": 800, "z": 1200},
            "P2": {"x": 300, "y": 800, "z": 1200},
            "P3": {"x": 100, "y": 200, "z": 1200},
            "P4": {"x": 300, "y": 200, "z": 1200},
        },
        {"x": 0, "y": 0, "z": 380, "r": -80},
    )

    assert [item["pair"] for item in targets] == [("P3", "P4"), ("P1", "P2")]
    assert all(item["plc_command"]["X"] == 230.0 for item in targets)
    assert all(item["plc_command"]["Z"] == 380.0 for item in targets)
    assert all(item["plc_command"]["R"] == -80.0 for item in targets)


def test_camera_axis_inverse_rejects_target_behind_camera():
    transform = CameraWorldTransform.from_json(CAMERA_EXTRINSIC)

    with pytest.raises(ValueError, match="光轴后方"):
        transform.plc_xy_for_camera_axis_target(
            (0, 0, -10000), {"x": 0, "y": 0, "z": 380, "r": -80}
        )
