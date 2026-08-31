# -*- coding: utf-8 -*-
"""OpenCV helpers that work with non-ASCII Windows paths.

cv2.imread / cv2.imwrite fail when the path contains characters such as Chinese
folder names.  Use numpy fromfile / tofile through imdecode / imencode instead.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np


def read_image(path: str | Path, flags: int = cv2.IMREAD_COLOR):
    """Read an image from a possibly non-ASCII path. Returns None on failure."""
    file_path = Path(path)
    try:
        data = np.fromfile(str(file_path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, flags)


def write_image(path: str | Path, image, params: Sequence[Any] | None = None) -> bool:
    """Write an image to a possibly non-ASCII path."""
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    suffix = file_path.suffix.lower() or ".png"
    ok, encoded = cv2.imencode(suffix, image, params or [])
    if not ok:
        return False
    try:
        encoded.tofile(str(file_path))
    except OSError:
        return False
    return file_path.is_file() and file_path.stat().st_size > 0
