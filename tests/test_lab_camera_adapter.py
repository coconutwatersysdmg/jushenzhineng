from services import lab_camera_adapter as adapter
from services.lab_camera_adapter import LabCameraCornerService


def test_corner_service_recognizes_a_p3p4_capture_group_without_waiting_for_p1p2(monkeypatch):
    class FakeLocalizer:
        last_image_points = {}

        def detect_pair(self, _frame, _pose, pair):
            self.last_image_points = {pair[0]: [10.0, 20.0], pair[1]: [30.0, 40.0]}
            return {pair[0]: [1.0, 2.0, 3.0], pair[1]: [4.0, 5.0, 6.0]}

    monkeypatch.setattr(adapter, "build_frame_from_paths", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(
        LabCameraCornerService,
        "_save_annotated_corners",
        staticmethod(lambda _rgb, _points: "p34_annotated.png"),
    )
    service = LabCameraCornerService()
    service._localizer = FakeLocalizer()
    meta = {
        point: {
            "capture_group": "P3P4",
            "rgb_path": "p34.jpg",
            "depth_path": "p34.png",
            "depth_scale_mm": 1.0,
            "plc_pose": {"x": 1.0, "y": 2.0, "z": 380.0, "r": -90.0},
        }
        for point in ("P3", "P4")
    }

    result = service.locate_from_capture_meta(["P3", "P4"], meta, {"P3P4": meta})

    assert result["success"] is True
    assert result["world_points"] == {
        "P3": {"x": 1.0, "y": 2.0, "z": 3.0},
        "P4": {"x": 4.0, "y": 5.0, "z": 6.0},
    }
    assert result["details"]["P3P4"]["annotated_image_path"] == "p34_annotated.png"
