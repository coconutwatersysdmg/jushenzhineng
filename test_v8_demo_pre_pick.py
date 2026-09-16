# -*- coding: utf-8 -*-
from pathlib import Path
from tempfile import TemporaryDirectory

from config.feature_switches import apply_run_profile
from controllers.flow_controller import FlowController
from utils.demo_assets import ensure_demo_pre_pick_image

# 本用例验证模拟路径；避免本机残留 field/lab 配置连真机。
apply_run_profile("sim", persist=False)

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
assert controller.current_step[0] == "DEVICE_CHECK", controller.current_step
snapshot = controller.execute_next()
assert snapshot["step_code"] == "PRE_PICK_OFFSET", snapshot
assert controller.results[-1]["step_code"] == "DEVICE_CHECK", controller.results[-1]
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
# 真机缺图时步骤2改为软继续（warning），不再抛异常阻断流程。
retry = FlowController()
retry.camera.tagged_images.pop("pre_pick_offset", None)
retry.set_plan([{"cargo_code":"RETRY","quantity":1,"pallet_reference_width_mm":1200}])
retry.start(); retry.allow_demo=False; retry.camera.set_demo_enabled(False)
assert retry.execute_next()["step_code"] == "PRE_PICK_OFFSET"
snap = retry.execute_next()
assert snap["step_code"] == "PARALLEL_LOCATE", snap
assert retry.results[-1]["status"] == "warning", retry.results[-1]
assert retry.round_data["pre_pick_offset"].get("capture_success") is False
assert retry.round_data["pre_pick_offset"].get("continue_anyway") is True
msgs = " ".join(str(m.get("message") or "") for m in (retry.twin.snapshot().get("messages") or []))
assert "拍照失败" in msgs or "未取得" in msgs, msgs
assert retry.twin.snapshot()["alarm"] is None
retry.allow_demo=True; retry.camera.set_demo_enabled(True)
# 下一轮前重置到步骤2再试成功路径：直接再跑一次 capture 验证 demo 可恢复。
captured_ok = retry._capture_rgb("CAM_PICK", "pre_pick_offset", required=True)
assert captured_ok.get("success") and captured_ok.get("demo"), captured_ok

print("DEMO_PRE_PICK_FALLBACK_OK", result["overhang_percent"], captured_fallback["image_path"])
