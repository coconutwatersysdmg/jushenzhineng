# -*- coding: utf-8 -*-
from core.digital_twin_state import DigitalTwinState
from core.geometry import Pose6D
from devices.mock_devices import MockPLCAdapter, MockRobotAdapter


def main():
    twin=DigitalTwinState(); plc=MockPLCAdapter(twin); plc.connect(); robot=MockRobotAdapter(twin,plc)
    before=twin.camera_world_pose("CAM_PICK")
    robot.move_tool_world("PICK_ARM", {"x_mm":7000,"y_mm":-2500,"z_mm":1700,"roll_deg":0,"pitch_deg":0,"yaw_deg":0}, "TEST")
    after=twin.camera_world_pose("CAM_PICK")
    assert before != after
    assert twin.snapshot()["cameras"]["CAM_PICK"]["parent_robot_id"] == "PICK_ARM"
    assert list(twin.snapshot()["cameras"]) == ["CAM_PICK"]
    print("V7 CORE TEST PASSED")
    print("CAM_PICK before:", before)
    print("CAM_PICK after :", after)

if __name__ == "__main__": main()
