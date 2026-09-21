from pathlib import Path


def test_lab_docs_describe_camera_final_world_points():
    root = Path(__file__).resolve().parents[1]
    camera_doc = (root / "docs/loading_steps/07_CAMERA_TO_WORLD.md").read_text(encoding="utf-8")
    corner_doc = (root / "docs/loading_steps/06_CORNER_RECOGNITION.md").read_text(encoding="utf-8")

    assert "P3/P4、P1/P2" in camera_doc
    assert "相机 WORLD 角点优先" in corner_doc

