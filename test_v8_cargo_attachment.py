# -*- coding: utf-8 -*-
from controllers.flow_controller import FlowController


def close_pose(actual, expected, tolerance=1e-5):
    for key in ("x_mm", "y_mm", "z_mm", "roll_deg", "pitch_deg", "yaw_deg"):
        assert abs(float(actual.get(key, 0.0)) - float(expected.get(key, 0.0))) <= tolerance, (key, actual, expected)


events = []
controller = FlowController()
controller.set_state_listener(
    lambda snapshot, motion: events.append(
        {
            "robot_id": motion["robot_id"],
            "task": motion["task"],
            "cargo": snapshot["twin"]["cargo"],
            "robot_pose": snapshot["twin"]["devices"]["PICK_ARM"]["pose"],
        }
    )
)
controller.set_plan([{
    "cargo_code": "FOLLOW",
    "cargo_name": "机械臂随动验证货物",
    "quantity": 1,
    "length_mm": 1200,
    "width_mm": 1000,
    "height_mm": 900,
    "pallet_reference_width_mm": 1200,
}])
controller.start()
topology = controller.twin.snapshot()
assert [k for k,v in topology["devices"].items() if v.get("kind") == "robot"] == ["PICK_ARM"], topology["devices"]
assert list(topology["cameras"]) == ["CAM_PICK"], topology["cameras"]
assert topology["devices"]["PICK_ARM"]["pose"]["x_mm"] == -3500, topology["devices"]["PICK_ARM"]
assert topology["devices"]["PICK_ARM"]["pose"]["y_mm"] == 1000, topology["devices"]["PICK_ARM"]

# Step 2 then first-round parallel step 3.1/3.2.
controller.execute_next()
controller.execute_next()
picked = controller.twin.snapshot()["cargo"]
assert picked["status"] == "CARRIED", picked
assert picked["attached_to"] == "PICK_ARM", picked
assert picked["attachment"]["parent_device_id"] == "PICK_ARM", picked
close_pose(
    picked["pose"],
    {"x_mm": -2400, "y_mm": 1000, "z_mm": 850, "roll_deg": 0, "pitch_deg": 0, "yaw_deg": -90},
)

# Any subsequent PICK_ARM motion must carry the pallet/cargo rigidly with it.
before = picked["pose"]
controller.robot.move_tool_world(
    "PICK_ARM",
    {"x_mm": -3500, "y_mm": 18000, "z_mm": 1700, "roll_deg": 0, "pitch_deg": 0, "yaw_deg": -90},
    task="VERIFY_CARGO_FOLLOW",
)
following = controller.twin.snapshot()["cargo"]
assert following["attached_to"] == "PICK_ARM", following
assert following["pose"] != before, (before, following)

# Finish mapping/planning.  The arm must still be at the photographed head
# endpoint before the newly inserted 8.4 pre-place monitor begins.
while controller.current_step[0] != "PRE_PLACE_MONITOR":
    controller.execute_next()
assignments = controller.round_data["corner_assignments"]
assert assignments and all(
    value["robot_id"] == "PICK_ARM" and value["camera_id"] == "CAM_PICK"
    and value["target_robot_pose_world"]["x_mm"] == -3500
    for value in assignments.values()
), assignments
assert {value["capture_group"] for value in assignments.values()} == {"TAIL", "HEAD"}
capture_events = [e for e in events if e["task"] in {"CAPTURE_TAIL", "CAPTURE_HEAD"}]
assert capture_events and all(e["cargo"].get("attached_to") == "PICK_ARM" for e in capture_events), capture_events
assert all(float(e["robot_pose"]["x_mm"]) == -3500.0 for e in capture_events), capture_events
assert all(abs(float(e["cargo"]["pose"]["pitch_deg"])) < 1e-6 for e in capture_events), capture_events
head_y=max(value["target_robot_pose_world"]["y_mm"] for value in assignments.values() if value["capture_group"]=="HEAD")
assert controller.twin.snapshot()["devices"]["PICK_ARM"]["pose"]["y_mm"] == head_y
controller.execute_next()
assert controller.current_step[0] == "PLACE"
target = controller.round_data["placement_target"]["final_world_pose"]
controller.execute_next()
placed = controller.twin.snapshot()["cargo"]
place_record = [r for r in controller.results if r["step_code"] == "PLACE"][-1]
assert place_record["data"]["selected_side"] == "LEFT", place_record
assert placed["status"] == "PLACED", placed
assert placed.get("attached_to") is None, placed
assert placed.get("attachment") is None, placed
close_pose(placed["pose"], target)

carry_events = [e for e in events if e["task"].startswith("CARRY_TO_PLACEMENT_SIDE_") or e["task"] in {"EXTEND_CARGO_FROM_OUTSIDE_TO_TARGET", "LOWER_CARGO_TO_TARGET"}]
carry_tasks=[]
for event in carry_events:
    if not carry_tasks or carry_tasks[-1] != event["task"]:
        carry_tasks.append(event["task"])
assert carry_tasks[-2:] == ["EXTEND_CARGO_FROM_OUTSIDE_TO_TARGET", "LOWER_CARGO_TO_TARGET"], carry_events
assert any(task.endswith("LOWER_LEFT") for task in carry_tasks), carry_tasks
assert all(e["cargo"].get("attached_to") == "PICK_ARM" for e in carry_events), carry_events
assert all(float(e["robot_pose"]["x_mm"]) == -3500.0 for e in carry_events), carry_events
close_pose(carry_events[-1]["cargo"]["pose"], target)

# Observation and arm return after release must not drag the placed cargo away.
while not controller.finished:
    controller.execute_next()
close_pose(controller.twin.snapshot()["cargo"]["pose"], target)
assert controller.twin.snapshot()["devices"]["PICK_ARM"]["pose"]["y_mm"] == 1000.0

print("V8_CARGO_ATTACHMENT_OK", carry_tasks, len(carry_events), placed["status"], target)
