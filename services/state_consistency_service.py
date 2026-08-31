# -*- coding: utf-8 -*-
from __future__ import annotations
from pathlib import Path
from typing import Optional


class StateConsistencyService:
    def __init__(self, mean_absdiff_threshold: float = 18.0):
        self.threshold = float(mean_absdiff_threshold)

    def compare(self, previous_image: str, current_image: str):
        p, c = Path(str(previous_image or "")), Path(str(current_image or ""))
        if not p.is_file() or not c.is_file():
            return {"success": True, "same_state": True, "demo": True, "score": 0.0, "message": "未提供完整前后状态图，联调按状态一致处理"}
        try:
            import cv2
            import numpy as np
            from utils.cv_io import read_image
        except ImportError:
            return {"success": False, "same_state": False, "message": "状态一致性检测需要 opencv-python/numpy"}
        a = read_image(p, cv2.IMREAD_GRAYSCALE); b = read_image(c, cv2.IMREAD_GRAYSCALE)
        if a is None or b is None:
            return {"success": False, "same_state": False, "message": "状态图读取失败"}
        b = cv2.resize(b, (a.shape[1], a.shape[0]))
        score = float(np.mean(cv2.absdiff(a, b)))
        same = score <= self.threshold
        return {"success": True, "same_state": same, "score": round(score, 3), "threshold": self.threshold, "message": "当前托盘状态与前一状态一致" if same else "当前托盘状态与前一状态不一致，需要更新可用空间"}
