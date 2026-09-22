from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from controllers.flow_controller import FlowController
from services.lab_camera_adapter import LabCameraCornerService
from utils.cv_io import write_image


class _FakeLabLocalizer:
    def __init__(self):
        self.calls = []

    def detect_pair(self, frame, plc_pose, pair_names):
        self.calls.append((frame, dict(plc_pose), tuple(pair_names)))
        return {
            "left": [101.0, 202.0, 303.0],
            "right": [404.0, 505.0, 606.0],
        }


class CamYoloLabPalletHoleTests(unittest.TestCase):
    def test_cam_yolo_lab_locates_two_pallet_holes_in_world(self):
        with TemporaryDirectory() as folder:
            rgb_path = Path(folder) / "pallet_rgb.jpg"
            depth_path = Path(folder) / "pallet_depth.png"
            self.assertTrue(write_image(rgb_path, np.zeros((12, 16, 3), dtype=np.uint8)))
            self.assertTrue(write_image(depth_path, np.full((12, 16), 1000, dtype=np.uint16)))

            localizer = _FakeLabLocalizer()
            service = LabCameraCornerService()
            service._localizer = localizer

            result = service.locate_pallet_holes(
                rgb_path,
                depth_path,
                {"x": 12.0, "y": 34.0, "z": 380.0, "r": -80.0},
                depth_scale_mm=1.0,
            )

            self.assertTrue(result["success"])
            self.assertEqual(result["recognition_source"], "cam_yolo_lab_pallet_holes")
            self.assertEqual(result["left_world_xyz_mm"], [101.0, 202.0, 303.0])
            self.assertEqual(result["right_world_xyz_mm"], [404.0, 505.0, 606.0])
            self.assertEqual(result["result_image_path"], str(rgb_path.resolve()))
            self.assertEqual(localizer.calls[0][1], {"x": 12.0, "y": 34.0, "z": 380.0, "r": -80.0})
            self.assertEqual(localizer.calls[0][2], ("left", "right"))


class _Twin:
    def __init__(self, cargo):
        self.cargo = cargo
        self.parallel = {}

    def snapshot(self):
        return {"cargo": self.cargo}

    def set_parallel(self, name, status, message, result=None):
        self.parallel[name] = {"status": status, "message": message, "result": result}


class _Robot:
    def __init__(self):
        self.command_emitter = object()
        self.moves = []

    def move_tool_world(self, robot_id, pose, task=""):
        self.moves.append(task)
        return {
            "success": True,
            "gantry_xyzr": {"X": 12.0, "Y": 34.0, "Z": 380.0, "R": -80.0},
        }


class _Camera:
    def __init__(self, rgb_path, depth_path):
        self.rgb_path = str(rgb_path)
        self.depth_path = str(depth_path)

    def capture_rgbd(self, camera_id, tag=""):
        return {
            "success": True,
            "camera_id": camera_id,
            "tag": tag,
            "rgb_path": self.rgb_path,
            "depth_path": self.depth_path,
            "depth_scale_mm": 1.0,
            "source": "realsense_d435i",
        }


class _Algorithms:
    def __init__(self):
        self.lab_calls = 0
        self.old_calls = 0

    def lab_pallet_hole_world_recognize(self, capture, plc_pose, cargo):
        self.lab_calls += 1
        return {
            "success": True,
            "recognition_source": "cam_yolo_lab_pallet_holes",
            "left_world_xyz_mm": [101.0, 202.0, 303.0],
            "right_world_xyz_mm": [404.0, 505.0, 606.0],
            "result_image_path": capture["rgb_path"],
            "message": "cam_yolo_lab 插孔识别完成",
        }

    def pallet_hole_recognize(self, rgb_path, depth_path, cargo):
        self.old_calls += 1
        return {"success": False, "message": "旧 pallet_hole_best.pt 不应在实验室模式调用"}


class _PLC:
    def send_message(self, *args, **kwargs):
        return {"success": True}


def _controller_for_lab_hole(rgb_path: Path, depth_path: Path):
    cargo = {
        "instance_id": "CARGO-TEST-001",
        "pose": {
            "x_mm": -2400.0,
            "y_mm": 1000.0,
            "z_mm": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": -90.0,
        },
    }
    controller = FlowController.__new__(FlowController)
    controller.round_data = {}
    controller.round_index = 0
    controller.queue = [cargo]
    controller.device_config = {"robots": {}, "camera_roles": {"pallet_hole": "CAM_PICK"}}
    controller.twin = _Twin(cargo)
    controller.robot = _Robot()
    controller.camera = _Camera(rgb_path, depth_path)
    controller.algorithms = _Algorithms()
    controller.plc = _PLC()
    controller.allow_demo = False
    controller._evidence = lambda *args, **kwargs: None
    return controller


class LabPalletHoleFlowTests(unittest.TestCase):
    def test_lab_step_two_routes_pallet_holes_to_cam_yolo_lab(self):
        with TemporaryDirectory() as folder:
            rgb_path = Path(folder) / "live_rgb.jpg"
            depth_path = Path(folder) / "live_depth.png"
            rgb_path.write_bytes(b"rgb")
            depth_path.write_bytes(b"depth")
            controller = _controller_for_lab_hole(rgb_path, depth_path)

            result = controller._lab_pick_recognize()

            self.assertTrue(result["success"])
            self.assertEqual(result["image_path"], str(rgb_path))
            self.assertEqual(result["hole_result"]["recognition_source"], "cam_yolo_lab_pallet_holes")
            self.assertEqual(controller.algorithms.lab_calls, 1)
            self.assertEqual(controller.algorithms.old_calls, 0)
            self.assertIn("PALLET_HOLE_LEFT", controller.robot.moves)
            self.assertIn("PALLET_HOLE_RIGHT", controller.robot.moves)

    def test_lab_step_two_keeps_live_photo_when_cam_yolo_lab_fails(self):
        with TemporaryDirectory() as folder:
            rgb_path = Path(folder) / "live_rgb.jpg"
            depth_path = Path(folder) / "live_depth.png"
            rgb_path.write_bytes(b"rgb")
            depth_path.write_bytes(b"depth")
            controller = _controller_for_lab_hole(rgb_path, depth_path)

            def fail(*args, **kwargs):
                raise RuntimeError("cam_yolo_lab 未检测到两个目标")

            controller.algorithms.lab_pallet_hole_world_recognize = fail

            result = controller._lab_pick_recognize()

            self.assertFalse(result["success"])
            self.assertEqual(result["image_path"], str(rgb_path))
            self.assertEqual(result["capture"]["source"], "realsense_d435i")
            self.assertIn("cam_yolo_lab 未检测到两个目标", result["message"])
