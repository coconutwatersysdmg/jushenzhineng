from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from config import feature_switches
from controllers.flow_controller import FlowController
from core.digital_twin_state import DigitalTwinState
from devices.real_arm_camera_adapter import RealArmCameraAdapter
from devices.real_livox_radar_adapter import RealLivoxRadarAdapter
from services.livox_service import LivoxCaptureResult


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class LabLivePalletHoleCaptureTests(unittest.TestCase):
    def test_real_lab_policy_prevents_offline_rgbd_from_short_circuiting_live_capture(self):
        twin = DigitalTwinState()
        camera = RealArmCameraAdapter(
            twin,
            {"backend": "disabled", "allow_file_inputs": False},
        )
        controller = FlowController.__new__(FlowController)
        controller.device_mode = "real"
        controller.system_config = {"devices": {"livox": {"use_live_capture": False}}}
        controller.twin = twin
        controller.camera = camera
        controller.radar = object()

        rgb = PROJECT_ROOT / "examples" / "pallet_rgbd" / "pallet1.jpg"
        depth = PROJECT_ROOT / "examples" / "pallet_rgbd" / "pallet1_depth.png"
        self.assertTrue(rgb.is_file())
        self.assertTrue(depth.is_file())

        controller.set_debug_inputs(
            {
                "pallet_rgb": str(rgb),
                "pallet_depth": str(depth),
                "images": {},
                "depths": {"pallet_hole": str(depth)},
            }
        )

        capture = camera.capture_rgbd("CAM_PICK", tag="pallet_hole")

        self.assertFalse(capture["success"])
        self.assertNotEqual(capture.get("source"), "manual_file")
        self.assertEqual(controller.debug_inputs["pallet_rgb"], "")
        self.assertEqual(controller.debug_inputs["pallet_depth"], "")
        self.assertEqual(controller.debug_inputs["depths"]["pallet_hole"], "")

    def test_live_radar_policy_clears_non_example_offline_pcd_too(self):
        twin = DigitalTwinState()
        camera = RealArmCameraAdapter(twin, {"backend": "disabled"})

        class _Radar:
            def __init__(self):
                self.path = None

            def set_point_cloud_path(self, path):
                self.path = path

        radar = _Radar()
        controller = FlowController.__new__(FlowController)
        controller.device_mode = "real"
        controller.system_config = {"devices": {"livox": {"use_live_capture": True}}}
        controller.twin = twin
        controller.camera = camera
        controller.radar = radar

        controller.set_debug_inputs(
            {
                "point_cloud_path": "C:/data/recorded_board.pcd",
                "images": {},
                "depths": {},
            }
        )

        self.assertEqual(controller.debug_inputs["point_cloud_path"], "")
        self.assertEqual(radar.path, "")

    def test_lab_real_mode_clears_all_offline_camera_debug_inputs(self):
        previous = feature_switches.RUN_PROFILE
        try:
            feature_switches.apply_run_profile("lab", persist=False)
            twin = DigitalTwinState()
            camera = RealArmCameraAdapter(twin, {"backend": "disabled"})
            controller = FlowController.__new__(FlowController)
            controller.device_mode = "real"
            controller.system_config = {"devices": {"livox": {"use_live_capture": True}}}
            controller.twin = twin
            controller.camera = camera
            controller.radar = object()

            controller.set_debug_inputs(
                {
                    "pallet_rgb": "offline_pallet.jpg",
                    "pallet_depth": "offline_pallet.png",
                    "images": {"neighbor_pose": "offline_neighbor.jpg"},
                    "depths": {"lab_place_verify": "offline_place.png"},
                    "corner_images": {"P1": "offline_corner.jpg"},
                    "corner_depths": {"P1": "offline_corner.png"},
                }
            )

            self.assertEqual(controller.debug_inputs["pallet_rgb"], "")
            self.assertEqual(controller.debug_inputs["pallet_depth"], "")
            for bucket in ("images", "depths", "corner_images", "corner_depths"):
                self.assertTrue(all(not value for value in controller.debug_inputs[bucket].values()))
        finally:
            feature_switches.apply_run_profile(previous, persist=False)

    def test_real_camera_adapter_does_not_use_tagged_offline_rgbd(self):
        twin = DigitalTwinState()
        with TemporaryDirectory() as folder:
            rgb = Path(folder) / "offline_rgb.jpg"
            depth = Path(folder) / "offline_depth.png"
            rgb.write_bytes(b"rgb")
            depth.write_bytes(b"depth")
            camera = RealArmCameraAdapter(
                twin,
                {"backend": "disabled", "allow_file_inputs": False},
            )
            camera.set_tagged_image("pallet_hole", str(rgb))
            camera.set_tagged_depth("pallet_hole", str(depth))

            capture = camera.capture_rgbd("CAM_PICK", tag="pallet_hole")

            self.assertFalse(capture["success"])
            self.assertNotEqual(capture.get("source"), "manual_file")

    def test_live_livox_adapter_ignores_any_override_pcd(self):
        twin = DigitalTwinState()
        with TemporaryDirectory() as folder:
            root = Path(folder)
            override = root / "offline.pcd"
            live = root / "live.pcd"
            exe = root / "livox.exe"
            override.write_text("POINTS 1\nDATA ascii\n0 0 0\n", encoding="ascii")
            live.write_text("POINTS 1\nDATA ascii\n0 0 0\n", encoding="ascii")
            exe.write_bytes(b"exe")
            radar = RealLivoxRadarAdapter(twin, {"use_live_capture": True})

            class _LiveService:
                exe_path = exe

                @staticmethod
                def capture_once():
                    return LivoxCaptureResult(True, live, 1, 0, "", "")

            radar.service = _LiveService()
            radar.set_point_cloud_path(str(override))

            result = radar.locate_truck({})

            self.assertTrue(result["success"])
            self.assertEqual(result["source"], "livox_mid360")
            self.assertEqual(result["pcd_path"], str(live.resolve()))

    def test_lab_rgbd_capture_rejects_manual_source(self):
        previous = feature_switches.RUN_PROFILE
        try:
            feature_switches.apply_run_profile("lab", persist=False)

            class _ManualCamera:
                @staticmethod
                def capture_rgbd(camera_id, tag=""):
                    return {
                        "success": True,
                        "camera_id": camera_id,
                        "tag": tag,
                        "source": "manual_file",
                        "rgb_path": "offline.jpg",
                        "depth_path": "offline.png",
                    }

            controller = FlowController.__new__(FlowController)
            controller.device_mode = "real"
            controller.camera = _ManualCamera()

            with self.assertRaisesRegex(RuntimeError, "D435i.*实拍"):
                controller._capture_rgbd("CAM_PICK", "pallet_hole")
        finally:
            feature_switches.apply_run_profile(previous, persist=False)

    def test_lab_device_check_does_not_advance_when_any_sensor_is_offline(self):
        previous = feature_switches.RUN_PROFILE
        try:
            feature_switches.apply_run_profile("lab", persist=False)

            class _Twin:
                def set_phase(self, *_args, **_kwargs):
                    pass

                def set_alarm(self, *_args, **_kwargs):
                    pass

                @staticmethod
                def snapshot():
                    return {"parallel": {}}

            controller = FlowController.__new__(FlowController)
            controller.finished = False
            controller.running = True
            controller.round_index = 0
            controller.step_index = 0
            controller.queue = [{"instance_id": "cargo-1"}]
            controller.twin = _Twin()
            controller._device_check = lambda: {
                "success": False,
                "all_online": False,
                "message": "CAM_PICK 离线",
            }
            controller._record = lambda *_args, **_kwargs: None
            controller._save = lambda: None
            controller.snapshot = lambda: {}

            with self.assertRaisesRegex(RuntimeError, "设备.*在线"):
                controller.execute_next()

            self.assertEqual(controller.step_index, 0)
            controller.continue_after_step_failure("传感器仍离线")
            self.assertEqual(controller.step_index, 0)
        finally:
            feature_switches.apply_run_profile(previous, persist=False)

    def test_lab_corner_step_does_not_advance_without_four_camera_world_corners(self):
        previous = feature_switches.RUN_PROFILE
        try:
            feature_switches.apply_run_profile("lab", persist=False)

            class _Twin:
                def set_phase(self, *_args, **_kwargs):
                    pass

                def set_alarm(self, *_args, **_kwargs):
                    pass

                @staticmethod
                def snapshot():
                    return {"parallel": {}}

            controller = FlowController.__new__(FlowController)
            controller.finished = False
            controller.running = True
            controller.round_index = 0
            controller.step_index = 2
            controller.queue = [{"instance_id": "cargo-1"}]
            controller.twin = _Twin()
            controller._lab_corner_shell = lambda: {
                "success": False,
                "message": "相机四个 WORLD 角点不完整",
            }
            controller._record = lambda *_args, **_kwargs: None
            controller._save = lambda: None
            controller.snapshot = lambda: {}

            with self.assertRaisesRegex(RuntimeError, "相机.*WORLD.*不完整"):
                controller.execute_next()

            self.assertEqual(controller.step_index, 2)
        finally:
            feature_switches.apply_run_profile(previous, persist=False)
