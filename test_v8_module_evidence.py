# -*- coding: utf-8 -*-
from pathlib import Path
from tempfile import TemporaryDirectory

from controllers.flow_controller import FlowController
from services.dynamic_monitoring_service import DynamicMonitoringService
from services.module_evidence_service import ModuleEvidenceService


catalog = ModuleEvidenceService()
modules = {item["module_id"]: item for item in catalog.snapshot()}
assert len(modules) == 14, modules.keys()
for module_id in ("PALLET_HOLE_YOLO", "POINTNET_TRUCK", "CORNER_YOLO"):
    assert Path(modules[module_id]["model_path"]).is_file(), modules[module_id]
for module_id in ("DYNAMIC_PRE_PLACE", "DYNAMIC_POST_PLACE"):
    for path in modules[module_id]["example_inputs"].values():
        assert Path(path).is_file(), (module_id, path)

first = [code for code, _ in FlowController.FIRST_STEPS]
assert first.index("TARGET_CONFIRM") < first.index("PRE_PLACE_MONITOR") < first.index("PLACE")
assert first.index("PLACE") < first.index("POST_PLACE_BOTTOM") < first.index("POST_REGION")

inputs = modules["DYNAMIC_PRE_PLACE"]["example_inputs"]
with TemporaryDirectory() as folder:
    service = DynamicMonitoringService(Path(folder))
    pre = service.analyze(inputs["rgb_path"], inputs["depth_path"], inputs["camera_path"], "pre_place", "test")
    post = service.analyze(inputs["rgb_path"], inputs["depth_path"], inputs["camera_path"], "post_place_bottom_pallet", "test")
    for result in (pre, post):
        assert result["success"] is True, result
        assert len(result["corner_xyz_m"]) == 3, result
        assert Path(result["result_image_path"]).is_file(), result
        assert Path(result["result_json_path"]).is_file(), result

print("V8_MODULE_EVIDENCE_OK", len(modules), round(pre["offset_deg"], 3), first)
