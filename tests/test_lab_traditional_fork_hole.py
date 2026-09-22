from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from controllers.flow_controller import FlowController
from utils.cv_io import write_image


class TraditionalForkHoleAlgorithmTests(unittest.TestCase):
    def test_two_dark_holes_return_camera_xyz(self):
        try:
            from algorithm_modules.lab.fork_hole_lab.fork_hole_locator import locate_fork_holes
        except ImportError:
            self.fail("fork_hole_final 传统算法尚未集成")

        image = np.full((720, 1280, 3), 210, dtype=np.uint8)
        image[350:450, 330:470] = 10
        image[350:450, 810:950] = 10
        depth_mm = np.full((720, 1280), 1000.0, dtype=np.float32)
        intrinsics = {"fx": 1000.0, "fy": 1000.0, "cx": 640.0, "cy": 360.0}

        result = locate_fork_holes(image, depth_mm, intrinsics)

        self.assertEqual(result["left_hole_xyz_mm"], [-240.0, 40.0, 1000.0])
        self.assertEqual(result["right_hole_xyz_mm"], [240.0, 40.0, 1000.0])

    def test_service_reads_saved_rgbd_and_normalizes_output(self):
        try:
            from services.traditional_fork_hole_service import TraditionalForkHoleService
        except ImportError:
            self.fail("传统插孔识别服务尚未集成")

        with TemporaryDirectory() as folder:
            rgb_path = Path(folder) / "rgb.jpg"
            depth_path = Path(folder) / "depth.png"
            image = np.full((720, 1280, 3), 210, dtype=np.uint8)
            image[350:450, 330:470] = 10
            image[350:450, 810:950] = 10
            self.assertTrue(write_image(rgb_path, image))
            self.assertTrue(write_image(depth_path, np.full((720, 1280), 1000, dtype=np.uint16)))
            capture = {
                "rgb_path": str(rgb_path),
                "depth_path": str(depth_path),
                "depth_scale_mm": 1.0,
                "intrinsics": {"fx": 1000.0, "fy": 1000.0, "cx": 640.0, "cy": 360.0},
            }

            result = TraditionalForkHoleService().recognize_capture(capture)

            self.assertTrue(result["success"])
            self.assertEqual(result["recognition_source"], "traditional_fork_hole_rgbd")
            self.assertEqual(result["coordinate_frame"], "camera")
            self.assertEqual(result["left_xyz_mm"], [-240.0, 40.0, 1000.0])
            self.assertEqual(result["right_xyz_mm"], [240.0, 40.0, 1000.0])
            self.assertEqual(result["result_image_path"], str(rgb_path.resolve()))


class _Twin:
    def __init__(self, cargo):
        self.cargo = cargo
        self.parallel = {}

    def snapshot(self):
        return {"cargo": self.cargo}

    def set_parallel(self, name, status, message, result=None):
        self.parallel[name] = {"status": status, "message": message, "result": result}

    def camera_world_pose(self, camera_id):
        return {
            "x_mm": 0.0,
            "y_mm": 0.0,
            "z_mm": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
        }


class _Robot:
    def __init__(self):
        self.command_emitter = object()
        self.moves = []

    def move_tool_world(self, robot_id, pose, task=""):
        self.moves.append((task, dict(pose)))
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
            "intrinsics": {"fx": 904.0, "fy": 905.0, "cx": 658.0, "cy": 374.0},
            "camera_world_pose": {},
            "source": "realsense_d435i",
        }


class _Algorithms:
    def __init__(self):
        self.traditional_calls = 0
        self.cam_yolo_calls = 0

    def lab_pallet_hole_recognize(self, capture, cargo):
        self.traditional_calls += 1
        return {
            "success": True,
            "recognition_source": "traditional_fork_hole_rgbd",
            "coordinate_frame": "camera",
            "left_xyz_mm": [1.0, 2.0, 3.0],
            "right_xyz_mm": [4.0, 5.0, 6.0],
            "result_image_path": capture["rgb_path"],
            "message": "传统算法插孔识别完成",
        }

    def lab_pallet_hole_world_recognize(self, capture, plc_pose, cargo):
        self.cam_yolo_calls += 1
        return {"success": False, "message": "实验室插孔不应再调用 cam_yolo_lab"}


class _Calibration:
    def dynamic_camera_world_matrix(self, camera_pose):
        return "camera_to_world"

    def transform_point(self, matrix, xyz):
        self.last_matrix = matrix
        return [float(xyz[0]) + 10.0, float(xyz[1]) + 20.0, float(xyz[2]) + 30.0]


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
    controller.calibration = _Calibration()
    controller.plc = _PLC()
    controller.allow_demo = False
    controller._evidence = lambda *args, **kwargs: None
    return controller


class LabTraditionalForkHoleFlowTests(unittest.TestCase):
    def test_lab_step_two_uses_traditional_algorithm_then_transforms_to_world(self):
        with TemporaryDirectory() as folder:
            rgb_path = Path(folder) / "live_rgb.jpg"
            depth_path = Path(folder) / "live_depth.png"
            rgb_path.write_bytes(b"rgb")
            depth_path.write_bytes(b"depth")
            controller = _controller_for_lab_hole(rgb_path, depth_path)

            result = controller._lab_pick_recognize()

            self.assertTrue(result["success"])
            self.assertEqual(result["hole_result"]["recognition_source"], "traditional_fork_hole_rgbd")
            self.assertEqual(result["hole_result"]["left_world_xyz_mm"], [11.0, 22.0, 33.0])
            self.assertEqual(result["hole_result"]["right_world_xyz_mm"], [14.0, 25.0, 36.0])
            self.assertEqual(controller.algorithms.traditional_calls, 1)
            self.assertEqual(controller.algorithms.cam_yolo_calls, 0)
            self.assertEqual(result["image_path"], str(rgb_path))

    def test_lab_step_two_keeps_live_photo_when_traditional_algorithm_fails(self):
        with TemporaryDirectory() as folder:
            rgb_path = Path(folder) / "live_rgb.jpg"
            depth_path = Path(folder) / "live_depth.png"
            rgb_path.write_bytes(b"rgb")
            depth_path.write_bytes(b"depth")
            controller = _controller_for_lab_hole(rgb_path, depth_path)

            def fail(*args, **kwargs):
                raise RuntimeError("传统算法没有识别到两个插孔")

            controller.algorithms.lab_pallet_hole_recognize = fail

            result = controller._lab_pick_recognize()

            self.assertFalse(result["success"])
            self.assertEqual(result["image_path"], str(rgb_path))
            self.assertEqual(result["capture"]["source"], "realsense_d435i")
            self.assertIn("传统算法没有识别到两个插孔", result["message"])
