# -*- coding: utf-8 -*-
"""Deterministic demo assets used only when demo-device mode is enabled."""
from __future__ import annotations

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEMO_PRE_PICK_IMAGE = PROJECT_ROOT / "examples" / "demo_pre_pick_offset.jpg"
DEMO_AUTO_ROOT = PROJECT_ROOT / "examples" / "demo_auto"


def ensure_demo_pre_pick_image(path: Path = DEMO_PRE_PICK_IMAGE) -> Path:
    """Create a synthetic, algorithm-compatible pallet/cargo JPG if absent."""
    path = Path(path).resolve()
    if path.is_file() and path.stat().st_size > 0:
        return path

    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("生成步骤2联调示例图需要 opencv-python 和 numpy") from exc

    width, height = 1000, 800
    image = np.full((height, width, 3), (28, 34, 42), dtype=np.uint8)
    pallet_blue = (255, 100, 0)  # BGR; HSV hue falls inside the detector's blue range.
    cargo_white = (232, 232, 232)

    # Two separated blue pallets and two aligned light cargo blocks. The upper
    # object stays within the bottom pallet, so the expected decision is ACCEPT.
    cv2.rectangle(image, (200, 330), (800, 390), pallet_blue, -1)
    cv2.rectangle(image, (200, 680), (800, 740), pallet_blue, -1)
    cv2.rectangle(image, (220, 70), (780, 320), cargo_white, -1)
    cv2.rectangle(image, (220, 400), (780, 670), cargo_white, -1)

    for y in (355, 705):
        for x in (270, 470, 670):
            cv2.rectangle(image, (x, y), (x + 60, y + 20), (18, 48, 68), -1)
    cv2.putText(image, "DEMO PRE-PICK OFFSET", (235, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (120, 210, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "SYNTHETIC INPUT - NOT CAMERA CAPTURE", (205, 780),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 170, 220), 2, cv2.LINE_AA)

    path.parent.mkdir(parents=True, exist_ok=True)
    from utils.cv_io import write_image
    if not write_image(path, image, [cv2.IMWRITE_JPEG_QUALITY, 94]):
        raise RuntimeError(f"步骤2联调示例图写入失败：{path}")
    return path


def _safe_tag(tag: str) -> str:
    text = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(tag or "capture"))
    return text.strip("_-") or "capture"


def ensure_demo_rgb_image(tag: str, path: Path | None = None) -> Path:
    """Create a visibly synthetic camera frame for non-algorithm demo steps."""
    safe = _safe_tag(tag)
    path = Path(path or (DEMO_AUTO_ROOT / f"{safe}.jpg")).resolve()
    if path.is_file() and path.stat().st_size > 0:
        return path

    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("生成联调相机示例图需要 opencv-python 和 numpy") from exc

    width, height = 1280, 720
    image = np.full((height, width, 3), (25, 34, 46), dtype=np.uint8)
    cv2.rectangle(image, (120, 120), (1160, 640), (54, 78, 98), 3)
    cv2.rectangle(image, (250, 300), (1030, 570), (210, 216, 220), -1)
    cv2.line(image, (640, 160), (640, 620), (50, 210, 255), 3)
    cv2.line(image, (360, 435), (920, 435), (50, 210, 255), 3)
    cv2.drawMarker(image, (640, 435), (40, 70, 255), cv2.MARKER_CROSS, 42, 4)
    cv2.putText(image, f"DEMO CAMERA / {safe.upper()}", (125, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 1.05, (120, 215, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "AUTO-FILLED SYNTHETIC INPUT", (125, 690),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, (100, 170, 220), 2, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    from utils.cv_io import write_image
    if not write_image(path, image, [cv2.IMWRITE_JPEG_QUALITY, 93]):
        raise RuntimeError(f"联调相机示例图写入失败：{path}")
    return path


def ensure_demo_corner_rgbd(tag: str) -> tuple[Path, Path]:
    """Create an aligned synthetic corner RGB/depth pair for P1-P6."""
    safe = _safe_tag(tag)
    rgb_path = DEMO_AUTO_ROOT / f"{safe}.jpg"
    depth_path = DEMO_AUTO_ROOT / f"{safe}_depth.png"
    if rgb_path.is_file() and depth_path.is_file() and rgb_path.stat().st_size > 0 and depth_path.stat().st_size > 0:
        return rgb_path.resolve(), depth_path.resolve()

    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("生成角点联调 RGB-D 需要 opencv-python 和 numpy") from exc

    width, height = 1280, 720
    image = np.full((height, width, 3), (18, 28, 38), dtype=np.uint8)
    cx, cy = 658, 374
    cv2.line(image, (80, 590), (1200, 590), (155, 165, 170), 12)
    cv2.line(image, (cx, 120), (cx, 650), (50, 205, 255), 3)
    cv2.line(image, (260, cy), (1050, cy), (50, 205, 255), 3)
    cv2.circle(image, (cx, cy), 24, (35, 70, 255), 5)
    cv2.putText(image, f"DEMO CORNER {safe.upper()}", (65, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (125, 220, 255), 3, cv2.LINE_AA)
    cv2.putText(image, "RGB + ALIGNED UINT16 DEPTH", (65, 690),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (100, 170, 220), 2, cv2.LINE_AA)
    depth = np.full((height, width), 1500, dtype=np.uint16)
    depth[:80, :] = 0

    rgb_path.parent.mkdir(parents=True, exist_ok=True)
    from utils.cv_io import write_image
    if not write_image(rgb_path, image, [cv2.IMWRITE_JPEG_QUALITY, 93]):
        raise RuntimeError(f"角点联调 RGB 写入失败：{rgb_path}")
    if not write_image(depth_path, depth):
        raise RuntimeError(f"角点联调深度写入失败：{depth_path}")
    return rgb_path.resolve(), depth_path.resolve()
