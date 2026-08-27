# -*- coding: utf-8 -*-
"""不依赖 Open3D 的点云结果结构/第一作业面规划测试。"""
from algorithm_modules.point_cloud_segment_module.point_cloud_pipeline import compute_first_workface_loading_plan
from services.point_cloud_processing_service import PointCloudProcessingService
from services.truck_bed_planning_service import TruckBedPlanningService


def main():
    loading = compute_first_workface_loading_plan(4380.54, 1200.0, 0.5)
    raw = {
        "label_2": {
            "corner_ids": ["P1", "P2", "P3", "P4"],
            "corners_xyz_mm": [[0,0,1400],[0,3000,1400],[4380,0,1430],[4380,3000,1430]],
            "length_mm": 4380.54,
            "width_mm": 3000,
            "height_mean_mm": 1415,
            "tilt_angle_deg": 0.39,
            "offset_angle_deg": 2.28,
            "loading_plan": loading,
        },
        "label_3": {
            "corner_ids": ["P5", "P6", "P7", "P8"],
            "corners_xyz_mm": [[0,0,1200],[0,3000,1200],[13000,0,1210],[13000,3000,1210]],
            "length_mm": 13000,
            "width_mm": 3000,
            "height_mean_mm": 1205,
            "tilt_angle_deg": 0.04,
            "offset_angle_deg": 0.7,
        },
    }
    normalized = PointCloudProcessingService.normalize_result(raw)
    assert normalized["board_count"] == 2
    assert normalized["corner_ids"] == [f"P{i}" for i in range(1, 9)]
    assert normalized["first_workface_loading_plan"]["pad_required"] is True

    planner = TruckBedPlanningService()
    completed = []
    plans = []
    for i in range(5):
        plan = planner.plan(normalized, completed, {"cargo_code": f"C{i+1}"})
        plans.append(plan)
        completed.append({"placement_plan": plan})
    assert [p["workface"] for p in plans] == ["label_2", "label_2", "label_2", "label_2", "label_3"]
    assert plans[3]["pad_required"] is True
    print("POINT CLOUD INTEGRATION TEST PASSED")
    print(loading)
    print([p["label"] for p in plans])


if __name__ == "__main__":
    main()
