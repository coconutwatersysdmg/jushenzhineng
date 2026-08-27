# -*- coding: utf-8 -*-
from pathlib import Path

from controllers.flow_controller import FlowController


controller = FlowController()
left = controller._side_for_target({"column": "A", "final_world_pose": {"x_mm": -900}})
right = controller._side_for_target({"column": "B", "final_world_pose": {"x_mm": 900}})
assert left[0] == right[0] == "PICK_ARM"
assert left[2:] == (-3500.0, -90.0), left
assert right[2:] == (3500.0, 90.0), right

controller.set_plan([{"cargo_code":"SIDE","cargo_name":"换侧验证","quantity":1,"length_mm":1200,"width_mm":1000,"height_mm":900}])
controller.start()
right_move = controller._move_gantry_to_target_side(
    {"column": "A", "final_world_pose": {"x_mm": 900, "y_mm": 9000, "z_mm": 1800}},
    2600,
    "VERIFY_RIGHT_SIDE",
)
assert right_move["selected_side"] == "RIGHT", right_move
assert right_move["final_pose"]["x_mm"] == 3500.0, right_move
assert any(move["pose"]["x_mm"] == 3500.0 for move in right_move["moves"]), right_move

qml = Path("ui/qml/TwinScene.qml").read_text(encoding="utf-8")
for token in (
    "id: gantryFrame",
    "property real leftX: -3.5",
    "property real rightX: 3.5",
    "id: movingCrossbeam",
    "The single loading arm remains suspended from the moving crossbeam",
):
    assert token in qml, token

print("V8_GANTRY_SIDES_OK", left[2:], right[2:])
