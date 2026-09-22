from pathlib import Path
import unittest

from controllers.flow_controller import FlowController
from core.digital_twin_state import DigitalTwinState
from devices.real_arm_camera_adapter import RealArmCameraAdapter


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class LabLivePalletHoleCaptureTests(unittest.TestCase):
    def test_real_lab_policy_prevents_offline_rgbd_from_short_circuiting_live_capture(self):
        twin = DigitalTwinState()
        camera = RealArmCameraAdapter(twin, {"backend": "disabled"})
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
