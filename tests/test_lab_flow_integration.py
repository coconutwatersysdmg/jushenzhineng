from pathlib import Path

from config import feature_switches
from services.lab_camera_visit_planner import merge_lab_corner_points


def test_lab_final_points_prefer_camera_and_fallback_per_point():
    final = merge_lab_corner_points(
        {"P1": {"x": 1, "y": 2, "z": 3}, "P2": {"x": 4, "y": 5, "z": 6}},
        {"P1": {"x": 10, "y": 20, "z": 30}},
    )

    assert final["P1"]["source"] == "camera_yolo"
    assert final["P1"]["x"] == 10
    assert final["P2"]["source"] == "lidar_fallback"
    assert final["P2"]["x"] == 4


def test_lab_flow_keeps_hole_recognition_non_forking():
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    lab_method = source[source.index("def _lab_pick_recognize"):source.index("def _lab_sense")]

    assert '"fork_skipped": True' in lab_method
    assert '"world_coordinate_frame"' in lab_method
    assert "self.robot.fork_pallet" not in lab_method


def test_lab_corner_shell_is_real_camera_flow_and_round_reuse_is_present():
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    shell = source[source.index("def _lab_corner_shell"):source.index("def _lab_overhead_pose")]
    advance = source[source.index("def _advance_lab_round"):source.index("def _parallel_locate")]

    assert 'group = "".join(pair)' in shell
    assert 'f"LAB_CORNER_{group}"' in shell
    assert "lab_corner_world_recognize" in shell
    assert "shell_only" not in shell
    assert "preserved_camera" in advance
    assert "preserved_final" in advance


def test_non_lab_profile_does_not_load_lab_corner_visit():
    previous = feature_switches.RUN_PROFILE
    try:
        feature_switches.apply_run_profile("field", persist=False)
        assert feature_switches.RUN_PROFILE == "field"
        source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
        assert "def _lab_corner_shell" in source
    finally:
        feature_switches.apply_run_profile(previous, persist=False)
