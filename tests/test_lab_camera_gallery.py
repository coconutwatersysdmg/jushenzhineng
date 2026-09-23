from services.lab_camera_gallery import build_lab_camera_gallery_entries


def test_gallery_contains_each_lab_camera_capture_and_its_recognition_summary():
    entries = build_lab_camera_gallery_entries(
        {
            "pick_result": {
                "image_path": "pick.jpg",
                "message": "插孔识别完成",
            },
            "lab_corner_shell": {
                "capture_log": [
                    {"group": "P3P4", "pair": ["P3", "P4"], "rgb_path": "p34.jpg", "capture_success": True},
                    {"group": "P1P2", "pair": ["P1", "P2"], "rgb_path": "p12.jpg", "capture_success": True},
                ],
                "camera_result": {
                    "world_points": {
                        "P1": {"x": 1, "y": 2, "z": 3},
                        "P2": {"x": 4, "y": 5, "z": 6},
                        "P3": {"x": 7, "y": 8, "z": 9},
                        "P4": {"x": 10, "y": 11, "z": 12},
                    }
                },
            },
            "lab_place_verify": {
                "planned_region_id": "B1",
                "result_image_path": "b1_result.jpg",
                "status": "DEPTH_INVALID",
                "message": "角点周围没有有效深度",
            },
        },
        round_index=1,
    )

    assert [entry["image_path"] for entry in entries] == ["pick.jpg", "p34.jpg", "p12.jpg", "b1_result.jpg"]
    assert entries[1]["label"] == "第1轮 · 底板角点 P3/P4"
    assert "P3=(7.0,8.0,9.0)" in entries[1]["summary"]
    assert entries[-1]["label"] == "第1轮 · B1 放货后检测"
    assert "DEPTH_INVALID" in entries[-1]["summary"]
