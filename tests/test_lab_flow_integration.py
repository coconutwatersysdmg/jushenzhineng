from pathlib import Path

from config import feature_switches
from services.lab_space_planner import (
    build_lab_space_plan,
    lab_b_region_for_round,
)
from services.lab_camera_visit_planner import merge_lab_corner_points
from services.lab_cycle_policy import lab_steps_for_round
from services.module_evidence_service import ModuleEvidenceService
from services.space_manager import SpaceManager


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
    assert '"PALLET_HOLE_COORDINATES"' in lab_method
    assert "self.robot.fork_pallet" not in lab_method
    assert "self.robot.move_tool_world" not in lab_method


def test_lab_hole_recognition_is_registered_as_traditional_not_yolo():
    rows = {row["module_id"]: row for row in ModuleEvidenceService().snapshot()}
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    lab_method = source[source.index("def _lab_pick_recognize"):source.index("def _lab_sense")]

    assert "LAB_PALLET_HOLE_TRADITIONAL" in rows
    assert "传统" in rows["LAB_PALLET_HOLE_TRADITIONAL"]["category"]
    assert '"LAB_PALLET_HOLE_TRADITIONAL"' in lab_method
    assert '"PALLET_HOLE_YOLO"' not in lab_method


def test_lab_corner_shell_is_real_camera_flow_and_round_reuse_is_present():
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    shell = source[source.index("def _lab_corner_shell"):source.index("def _lab_region_camera_move")]
    advance = source[source.index("def _advance_lab_round"):source.index("def _parallel_locate")]

    assert 'group = "".join(pair)' in shell
    assert 'f"LAB_CORNER_{group}"' in shell
    assert "lab_corner_world_recognize" in shell
    assert "build_lab_space_plan(" in shell
    assert '"lab_space_plan"' in shell
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


def test_lab_backend_grid_plan_uses_final_world_corners_without_ui_grid_payload():
    final_points = {
        "P1": {"x": 0.0, "y": 0.0, "z": 0.0},
        "P2": {"x": 1000.0, "y": 0.0, "z": 0.0},
        "P3": {"x": 0.0, "y": 2400.0, "z": 0.0},
        "P4": {"x": 1000.0, "y": 2400.0, "z": 0.0},
    }

    result = build_lab_space_plan(final_points, ["P1", "P2", "P3", "P4"])

    assert result["success"] is True
    assert result["geometry"]["decision_source"] == "camera_world_corners"
    assert len(result["space"]["regions"]) == 4
    assert result["ui_display"] is False


def test_lab_grid_numbers_from_p1_p2_end_and_only_loads_b_column():
    final_points = {
        "P1": {"x": 0.0, "y": 0.0, "z": 0.0},
        "P2": {"x": 1000.0, "y": 0.0, "z": 0.0},
        "P3": {"x": 0.0, "y": 3600.0, "z": 0.0},
        "P4": {"x": 1000.0, "y": 3600.0, "z": 0.0},
    }

    result = build_lab_space_plan(final_points, ["P1", "P2", "P3", "P4"])

    assert [r["region_id"] for r in result["space"]["regions"]] == [
        "A1", "B1", "A2", "B2", "A3", "B3",
    ]
    assert result["loading_order"] == ["B1", "B2", "B3"]
    assert lab_b_region_for_round(result["space"], 0)["region_id"] == "B1"
    assert lab_b_region_for_round(result["space"], 1)["region_id"] == "B2"
    assert lab_b_region_for_round(result["space"], 2)["region_id"] == "B3"


def test_lab_repeat_flow_monitors_previous_b_region_before_post_place_check():
    assert [code for code, _name in lab_steps_for_round(1)] == [
        "LAB_RETURN_ORIGIN",
        "LAB_SENSE",
        "LAB_PRE_PLACE_MONITOR",
        "LAB_PLACE_VERIFY",
    ]


def test_only_passed_post_place_check_occupies_current_b_region():
    final_points = {
        "P1": {"x": 0.0, "y": 0.0, "z": 0.0},
        "P2": {"x": 1000.0, "y": 0.0, "z": 0.0},
        "P3": {"x": 0.0, "y": 2400.0, "z": 0.0},
        "P4": {"x": 1000.0, "y": 2400.0, "z": 0.0},
    }
    manager = SpaceManager()
    plan = build_lab_space_plan(
        final_points,
        ["P1", "P2", "P3", "P4"],
        space_manager=manager,
    )

    assert manager.occupy_region_if_passed("B1", "cargo-1", {"inside_planned_region": False}) is None
    occupied = manager.occupy_region_if_passed("B1", "cargo-1", {"inside_planned_region": True})

    assert occupied["region_id"] == "B1"
    assert occupied["status"] == "OCCUPIED"
    snapshot = manager.snapshot()
    assert [r["region_id"] for r in snapshot["occupied"]] == ["B1"]
    assert "A1" in [r["region_id"] for r in snapshot["available"]]
    assert plan["loading_order"] == ["B1", "B2"]
