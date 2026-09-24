from pathlib import Path

import pytest

from config import feature_switches
from services.lab_space_planner import (
    build_lab_space_plan,
    lab_b_region_for_round,
)
from services.lab_camera_visit_planner import merge_lab_corner_points
from services.lab_cycle_policy import lab_steps_for_round
from services.lab_dynamic_box_monitor_service import (
    LabDynamicBoxMonitorService,
    judge_box_world_region,
)
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


def test_lab_sense_failure_stops_at_step_two_for_retry_instead_of_reaching_corner_capture():
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    sense = source[source.index("def _lab_sense"):source.index("def _lab_held_r_deg")]
    dispatch = source[source.index('elif code=="LAB_SENSE":'):source.index('elif code=="LAB_CORNER_SHELL":')]

    assert '"continue_anyway": False' in sense
    assert 'raise RuntimeError(data.get("message") or "实验室插孔识别或雷达四角失败")' in dispatch


def test_lab_hole_recognition_is_registered_as_traditional_not_yolo():
    rows = {row["module_id"]: row for row in ModuleEvidenceService().snapshot()}
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    lab_method = source[source.index("def _lab_pick_recognize"):source.index("def _lab_sense")]

    assert "LAB_PALLET_HOLE_TRADITIONAL" in rows
    assert "传统" in rows["LAB_PALLET_HOLE_TRADITIONAL"]["category"]
    assert '"LAB_PALLET_HOLE_TRADITIONAL"' in lab_method
    assert '"PALLET_HOLE_YOLO"' not in lab_method


def test_lab_hole_world_coordinates_reuse_the_lab_camera_plc_calibration():
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    lab_method = source[source.index("def _lab_pick_recognize"):source.index("def _lab_sense")]

    assert "self.algorithms.lab_camera_transform()" in lab_method
    assert ".camera_to_world(" in lab_method
    assert "dynamic_camera_world_matrix" not in lab_method


def test_lab_ui_text_describes_xy_only_motion():
    policy = Path("services/lab_cycle_policy.py").read_text(encoding="utf-8")
    controller = Path("controllers/flow_controller.py").read_text(encoding="utf-8")

    assert "仅 XYZ" not in policy
    assert "仅下发 XYZ" not in controller


def test_lab_profile_disables_corner_review_and_plc_confirmation_dialog():
    lab = feature_switches.get_run_profile("lab")
    window = Path("ui/main_window.py").read_text(encoding="utf-8")
    presenter = window[window.index("def _present_next_plc_command"):window.index("def _push_plc_batch")]

    assert lab["switches"]["CORNER_REVIEW_ENABLED"] is False
    assert "if self._is_lab_ui():" in presenter
    assert "self._push_plc_batch(batch)" in presenter
    assert "dlg.apply_batch(batch" not in presenter.split("if self._is_lab_ui():", 1)[1].split("auto =", 1)[0]


def test_step_failure_dialog_offers_retry_skip_and_stop_for_every_lab_step():
    window = Path("ui/main_window.py").read_text(encoding="utf-8")
    controller = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    dialog = window[window.index("def _ask_continue_after_failure"):window.index("def _handle_step_failure")]
    handler = window[window.index("def _handle_step_failure"):window.index("def _next")]
    continue_method = controller[
        controller.index("def continue_after_step_failure"):controller.index("def reset")
    ]

    assert 'box.addButton("重新尝试"' in dialog
    assert 'box.addButton("继续跳过"' in dialog
    assert 'box.addButton("停止"' in dialog
    assert "QTimer.singleShot(0, self._retry_current_step)" in handler
    assert '"retry_required"' not in continue_method


def test_lab_corner_shell_recognizes_p3p4_while_plc_moves_to_p1p2():
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    shell = source[source.index("def _lab_corner_shell"):source.index("def _lab_region_camera_move")]
    advance = source[source.index("def _advance_lab_round"):source.index("def _parallel_locate")]

    assert 'group = "".join(pair)' in shell
    assert 'f"LAB_CORNER_{group}"' in shell
    assert "lab_corner_world_recognize" in shell
    assert "p34_future = corner_pool.submit(" in shell
    assert 'if group == "P3P4":' in shell
    assert 'if group == "P1P2" and p34_future is not None:' in shell
    assert "p34_future.done()" in shell
    assert shell.index("p34_future.done()") < shell.index('cap = self._capture_rgbd(camera_id, f"LAB_CORNER_{group}")')
    assert "p34_future.result()" in shell
    recognize_group = shell[shell.index("def recognize_group"):shell.index("def show_group_result")]
    assert 'show_group_result(group, result)' in recognize_group
    assert recognize_group.index('show_group_result(group, result)') < recognize_group.index('return result')
    assert 'title="第3步：角点 YOLO 识别结果"' not in shell
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
        "P1": {"x": 0.0, "y": 0.0, "z": 0.0, "source": "camera_yolo"},
        "P2": {"x": 1000.0, "y": 0.0, "z": 0.0, "source": "camera_yolo"},
        "P3": {"x": 0.0, "y": 2400.0, "z": 0.0, "source": "camera_yolo"},
        "P4": {"x": 1000.0, "y": 2400.0, "z": 0.0, "source": "camera_yolo"},
    }

    result = build_lab_space_plan(final_points, ["P1", "P2", "P3", "P4"])

    assert result["success"] is True
    assert result["geometry"]["decision_source"] == "camera_world_corners"
    assert len(result["space"]["regions"]) == 10
    assert result["geometry"]["lab_equal_row_count"] == 5
    assert result["ui_display"] is False


def test_lab_grid_uses_world_corners_to_make_two_columns_and_five_equal_rows():
    final_points = {
        "P1": {"x": 100.0, "y": 200.0, "z": 1000.0, "source": "camera_yolo"},
        "P2": {"x": 1300.0, "y": 400.0, "z": 1020.0, "source": "camera_yolo"},
        "P3": {"x": 700.0, "y": 6200.0, "z": 1300.0, "source": "camera_yolo"},
        "P4": {"x": 1900.0, "y": 6400.0, "z": 1320.0, "source": "camera_yolo"},
    }

    result = build_lab_space_plan(final_points, ["P1", "P2", "P3", "P4"])

    assert [r["region_id"] for r in result["space"]["regions"]] == [
        "A1", "B1", "A2", "B2", "A3", "B3",
        "A4", "B4", "A5", "B5",
    ]
    assert result["loading_order"] == ["B1", "B2", "B3", "B4", "B5"]
    regions = {region["region_id"]: region for region in result["space"]["regions"]}
    assert regions["A1"]["corners_world_xyz_mm"] == [
        [100.0, 200.0, 1000.0],
        [700.0, 300.0, 1010.0],
        [220.0, 1400.0, 1060.0],
        [820.0, 1500.0, 1070.0],
    ]
    assert regions["B5"]["corners_world_xyz_mm"][3] == [1900.0, 6400.0, 1320.0]
    assert lab_b_region_for_round(result["space"], 0)["region_id"] == "B1"
    assert lab_b_region_for_round(result["space"], 4)["region_id"] == "B5"


def test_lab_grid_rejects_lidar_fallback_corner():
    final_points = {
        "P1": {"x": 0.0, "y": 0.0, "z": 0.0, "source": "camera_yolo"},
        "P2": {"x": 1000.0, "y": 0.0, "z": 0.0, "source": "camera_yolo"},
        "P3": {"x": 0.0, "y": 6000.0, "z": 0.0, "source": "lidar_fallback"},
        "P4": {"x": 1000.0, "y": 6000.0, "z": 0.0, "source": "camera_yolo"},
    }

    with pytest.raises(RuntimeError, match="相机.*WORLD.*完整"):
        build_lab_space_plan(final_points, ["P1", "P2", "P3", "P4"])


def test_lab_repeat_flow_monitors_previous_b_region_before_post_place_check():
    assert [code for code, _name in lab_steps_for_round(1)] == [
        "LAB_RETURN_ORIGIN",
        "LAB_SENSE",
        "LAB_PRE_PLACE_MONITOR",
        "LAB_WAIT_MANUAL_PLACE",
        "LAB_PLACE_VERIFY",
    ]


def test_lab_manual_place_confirmation_directly_starts_current_region_check():
    assert [code for code, _name in lab_steps_for_round(0)] == [
        "DEVICE_CHECK",
        "LAB_SENSE",
        "LAB_CORNER_SHELL",
        "LAB_WAIT_MANUAL_PLACE",
        "LAB_PLACE_VERIFY",
    ]
    controller = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    window = Path("ui/main_window.py").read_text(encoding="utf-8")
    wait_branch = controller[
        controller.index('elif code=="LAB_WAIT_MANUAL_PLACE":'):
        controller.index('elif code=="LAB_PLACE_VERIFY":')
    ]

    assert "_lab_manual_place_confirmed" in wait_branch
    assert "return self.execute_next()" in wait_branch
    assert 'self.next_btn.setText("确认已放好，开始拍照检测")' in window
    assert 'self.controller.current_step[0] == "LAB_WAIT_MANUAL_PLACE"' in window
    continue_method = controller[
        controller.index("def continue_after_step_failure"):controller.index("def _save")
    ]
    assert "if self.is_lab_profile() and code == \"LAB_PLACE_VERIFY\":" in continue_method
    assert "self._advance_lab_round()" in continue_method


def test_lab_post_place_verdict_records_warning_and_starts_next_manual_cycle():
    source = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    block = source[source.index('elif code=="LAB_PLACE_VERIFY":'):source.index('elif code=="PRE_PICK_OFFSET":')]

    assert 'raise RuntimeError(data.get("message") or "当前 B 区放货后检测未通过")' not in block
    assert "advance=self._advance_lab_round()" in block


def test_lab_automatic_motion_uses_xy_only_and_dialog_does_not_show_r_target():
    controller = Path("controllers/flow_controller.py").read_text(encoding="utf-8")
    dialog = Path("ui/plc_motion_dialog.py").read_text(encoding="utf-8")

    assert 'axes=("X", "Y")' in controller
    assert '"z_axis_untouched": True' in controller
    assert 'self.xyzr_table.setHorizontalHeaderLabels(["段/任务", "X", "Y", "Z"])' in dialog
    assert '"R": float(self.xyzr_table.item(row, 4).text())' not in dialog


def test_lab_box_region_decision_uses_xy_even_when_board_is_sloped():
    region = {
        "region_id": "B1",
        "corners_world_xyz_mm": [
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 40.0],
            [0.0, 100.0, 80.0],
            [100.0, 100.0, 120.0],
        ],
    }
    # Z 故意远离底板；只要 X/Y 在 B1 内，纸箱仍应通过。
    box = [[20.0, 20.0, 5000.0], [80.0, 20.0, 6000.0], [80.0, 80.0, 7000.0], [20.0, 80.0, 8000.0]]

    result = judge_box_world_region(box, region)

    assert result["inside_planned_region"] is True
    assert result["corner_inside"] == [True, True, True, True]


def test_lab_box_depth_sampling_expands_after_a_zero_depth_corner():
    calls = []

    class _Calibration:
        @staticmethod
        def dynamic_camera_world_matrix(_pose):
            import numpy as np
            return np.eye(4)

        @staticmethod
        def transform_point(_matrix, point):
            return point

    def sample_depth(_path, _u, _v, window):
        calls.append(window)
        return 0.0 if window == 5 else 1000.0

    service = LabDynamicBoxMonitorService(
        calibration=_Calibration(),
        depth_sampler=sample_depth,
    )
    result = None
    try:
        result = service._pixel_to_world(
            {
                "depth_path": "depth.png",
                "depth_scale_mm": 1.0,
                "intrinsics": {"fx": 1000.0, "fy": 1000.0, "cx": 0.0, "cy": 0.0},
            },
            10.0,
            20.0,
        )
    except RuntimeError:
        pass

    assert calls == [5, 11]
    assert result["depth_value"] == 1000.0


def test_lab_box_world_conversion_reuses_lab_camera_plc_transform():
    class _Calibration:
        @staticmethod
        def dynamic_camera_world_matrix(_pose):
            raise AssertionError("实验室动态监测不应改用数字孪生相机外参")

    class _LabTransform:
        def __init__(self):
            self.calls = []

        def camera_to_world(self, point, plc_pose):
            self.calls.append((tuple(point), dict(plc_pose)))
            return (111.0, 222.0, 333.0)

    transform = _LabTransform()
    service = LabDynamicBoxMonitorService(
        calibration=_Calibration(),
        depth_sampler=lambda *_args: 1000.0,
    )
    service.lab_world_transform = transform

    result = service._pixel_to_world(
        {
            "depth_path": "depth.png",
            "depth_scale_mm": 1.0,
            "intrinsics": {"fx": 1000.0, "fy": 1000.0, "cx": 0.0, "cy": 0.0},
            "plc_pose": {"x": 10.0, "y": 20.0, "z": 380.0, "r": -90.0},
        },
        10.0,
        20.0,
    )

    assert transform.calls
    assert result["world_xyz_mm"] == [111.0, 222.0, 333.0]
    assert result["world_transform"] == "lab_camera_plc_extrinsic"


def test_only_passed_post_place_check_occupies_current_b_region():
    final_points = {
        "P1": {"x": 0.0, "y": 0.0, "z": 0.0, "source": "camera_yolo"},
        "P2": {"x": 1000.0, "y": 0.0, "z": 0.0, "source": "camera_yolo"},
        "P3": {"x": 0.0, "y": 2400.0, "z": 0.0, "source": "camera_yolo"},
        "P4": {"x": 1000.0, "y": 2400.0, "z": 0.0, "source": "camera_yolo"},
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
    assert plan["loading_order"] == ["B1", "B2", "B3", "B4", "B5"]
