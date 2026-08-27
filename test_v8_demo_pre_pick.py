# -*- coding: utf-8 -*-
from pathlib import Path
from tempfile import TemporaryDirectory

from controllers.flow_controller import FlowController
from utils.demo_assets import ensure_demo_pre_pick_image


with TemporaryDirectory() as td:
    image_path = ensure_demo_pre_pick_image(Path(td) / "demo_pre_pick_offset.jpg")
    assert image_path.is_file() and image_path.stat().st_size > 0

controller = FlowController()
controller.set_plan([{
    "cargo_code": "DEMO-001", "cargo_name": "示例货物", "quantity": 1,
    "length_mm": 1200, "width_mm": 1000, "height_mm": 900,
    "pallet_reference_width_mm": 1200,
}])
controller.start()
snapshot = controller.execute_next()
result = controller.round_data["pre_pick_offset"]

assert snapshot["step_code"] == "PARALLEL_LOCATE", snapshot
assert controller.results[-1]["status"] == "success", controller.results[-1]
assert result["should_fork"] is True, result
assert result.get("demo_input") is not True, result
assert result["algorithm"] == "pallet_overhang_detection_module", result
assert Path(result["image_path"]).is_file(), result

# If the auto-loaded example is explicitly removed, demo mode still provides
# the historical generated fallback.
fallback = FlowController()
fallback.camera.tagged_images.pop("pre_pick_offset", None)
captured_fallback = fallback._capture_rgb("CAM_PICK", "pre_pick_offset", required=True)
assert captured_fallback["success"] is True and captured_fallback.get("demo"), captured_fallback

# A user-selected/camera-provided image must always win over the demo fallback.
explicit = FlowController()
explicit.set_debug_inputs({"images": {"pre_pick_offset": str(Path(result["image_path"]))}})
captured = explicit._capture_rgb("CAM_PICK", "pre_pick_offset", required=True)
assert captured["success"] is True and not captured.get("demo"), captured

# Production mode remains strict and never injects synthetic data.
strict = FlowController()
strict.allow_demo = False
strict.camera.set_demo_enabled(False)
strict.camera.tagged_images.pop("pre_pick_offset", None)
try:
    strict._capture_rgb("CAM_PICK", "pre_pick_offset", required=True)
except RuntimeError as exc:
    assert "未取得 pre_pick_offset JPG" in str(exc), exc
else:
    raise AssertionError("production mode unexpectedly used a demo image")

# A successful retry clears the previous alarm instead of leaving a stale red banner.
retry = FlowController()
retry.camera.tagged_images.pop("pre_pick_offset", None)
retry.set_plan([{"cargo_code":"RETRY","quantity":1,"pallet_reference_width_mm":1200}])
retry.start(); retry.allow_demo=False; retry.camera.set_demo_enabled(False)
try:
    retry.execute_next()
except RuntimeError:
    pass
else:
    raise AssertionError("missing production image should fail before retry")
assert retry.twin.snapshot()["alarm"]
retry.allow_demo=True; retry.camera.set_demo_enabled(True); retry.execute_next()
assert retry.twin.snapshot()["alarm"] is None

print("DEMO_PRE_PICK_FALLBACK_OK", result["overhang_percent"], captured_fallback["image_path"])
