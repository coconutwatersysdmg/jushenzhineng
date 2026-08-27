# -*- coding: utf-8 -*-
"""不加载真实 .pt / PCD 的整流程逻辑测试。"""
from services.loading_cycle_service import LoadingCycleService


def main():
    svc = LoadingCycleService(
        allow_pallet_demo_fallback=True,
        allow_point_cloud_demo_fallback=True,
        allow_corner_demo_fallback=True,
    )
    svc.set_plan([
        {
            "cargo_code": "T-A",
            "cargo_name": "测试货物A",
            "quantity": 5,
            "length_mm": 1200,
            "width_mm": 900,
            "height_mm": 700,
        },
    ])
    svc.start()
    guard = 0
    while not svc.finished and guard < 100:
        svc.execute_next_step()
        guard += 1

    assert svc.finished
    assert svc.completed_count == 5
    assert guard == 60, guard

    # 示例高低板：label_2 规划 3 个正常槽 + 1 个垫板槽，第 5 件切到 label_3。
    plans = [row["placement_plan"] for row in svc.completed]
    assert [p["workface"] for p in plans[:4]] == ["label_2"] * 4
    assert plans[3]["pad_required"] is True
    assert plans[3]["remaining_workface_compensation_mm"] == 420.0
    assert plans[4]["workface"] == "label_3"

    for item in svc.completed:
        assert item["radar_result"]["board_count"] == 2
        assert len(item["radar_result"]["corner_ids"]) == 8
        assert item["pre_pick_deviation"]["success"]
        assert item["post_place_deviation"]["success"]
        assert item["post_place_monitor"]["success"]

    print("FLOW TEST PASSED", svc.completed_count, "rounds", guard, "steps")
    print("WORKFACES", [p["label"] for p in plans])


if __name__ == "__main__":
    main()
