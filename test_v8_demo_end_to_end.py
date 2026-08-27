# -*- coding: utf-8 -*-
from pathlib import Path

from controllers.flow_controller import FlowController


motions=[]
controller=FlowController()
controller.set_state_listener(lambda snapshot,motion: motions.append({
    "robot_id":motion["robot_id"],"task":motion["task"],"step":snapshot["step_code"],
    "x_mm":float(snapshot["twin"]["devices"]["PICK_ARM"]["pose"]["x_mm"]),
}))
controller.set_plan([{
    "cargo_code":"AUTO-DEMO","cargo_name":"自动补齐联调货物","quantity":2,
    "length_mm":1200,"width_mm":1000,"height_mm":900,"pallet_reference_width_mm":1200,
}])
controller.start()

step_count=0
while not controller.finished and step_count < 40:
    controller.execute_next(); step_count += 1

assert controller.finished is True
assert step_count == 27, step_count
assert all(record["status"] == "success" for record in controller.results), controller.results
capture=next(record for record in controller.results if record["step_code"] == "CAPTURE_CORNERS")
assert capture["data"]["physical_capture_count"] == 2, capture
assert set(capture["data"]["capture_groups"]) == {"TAIL", "HEAD"}, capture
assert len(set(capture["data"]["image_paths_by_point"].values())) == 4, capture
for pid,meta in capture["data"]["capture_meta"].items():
    assert meta["source"] == "camera_or_debug_input", meta
    assert Path(meta["rgb_path"]).is_file(), meta
    assert Path(meta["depth_path"]).is_file(), meta
world=next(record for record in controller.results if record["step_code"] == "CAMERA_TO_WORLD")
assert world["data"]["source"] == "example_rgbd_scene_aligned", world
assert world["data"]["raw_example_conversion"]["source"] == "camera_pixel_depth_world", world
assert len(world["data"]["world_points"]) == 4, world
assert {point["x"] for point in world["data"]["world_points"].values()} == {-1250.0,1250.0}, world
plan=next(record for record in controller.results if record["step_code"] == "INITIAL_SPACE_PLAN")
regions=plan["data"]["geometry"]["regions"]
assert regions and {region["column"] for region in regions} == {"A","B"}, regions
assert all(-1250.0 <= region["center_world_xyz_mm"][0] <= 1250.0 for region in regions), regions
assert len(motions) >= 12, motions
capture_tasks=[]
for motion in motions:
    if motion["task"] in {"CAPTURE_TAIL","CAPTURE_HEAD"} and (not capture_tasks or capture_tasks[-1] != motion["task"]):
        capture_tasks.append(motion["task"])
assert capture_tasks == ["CAPTURE_TAIL","CAPTURE_HEAD"], capture_tasks
capture_motions=[motion for motion in motions if motion["task"] in {"CAPTURE_TAIL","CAPTURE_HEAD"}]
assert capture_motions and all(motion["x_mm"] == -3500.0 for motion in capture_motions), capture_motions
place_records=[record for record in controller.results if record["step_code"] == "PLACE"]
for record in place_records:
    target_x=float(record["data"]["target"]["final_world_pose"]["x_mm"])
    expected="RIGHT" if target_x > 0.0 else "LEFT"
    assert record["data"]["selected_side"] == expected, record
retract_motions=[motion for motion in motions if motion["task"]=="RETRACT"]
assert retract_motions and all(motion["step"]=="RETURN" for motion in retract_motions), retract_motions
assert controller.twin.snapshot()["devices"]["PICK_ARM"]["pose"]["y_mm"] == 1000.0
assert controller.twin.snapshot()["alarm"] is None
modules={item["module_id"]:item for item in controller.snapshot()["module_evidence"]}
for module_id in ("PALLET_HOLE_YOLO","POINTNET_TRUCK","CORNER_YOLO"):
    assert modules[module_id]["model_invoked"] is True, modules[module_id]
for module_id in ("DYNAMIC_PRE_PLACE","DYNAMIC_POST_PLACE"):
    assert modules[module_id]["call_count"] == 2, modules[module_id]
print("V8_DEMO_END_TO_END_OK",step_count,len(motions),capture_tasks,sorted(world["data"]["world_points"]))
