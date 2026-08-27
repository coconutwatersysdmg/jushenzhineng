"""
托盘堆垛横向超限检测模块

功能
----
- 最底层蓝色托盘作为基准；
- 上方比较：上层白色货物 / 中间蓝色托盘 / 下层白色货物；
- 左右两侧分别取最外侧边界；
- 最大超出比例 > 5%：REJECT，否则 ACCEPT；
- 不输出可视化图片；
- JSON 仅保留：
    {
      "overhang_percent": 1.23,
      "decision": "ACCEPT"
    }

系统集成
--------
可直接导入：
    from pallet_overhang_detection_module import analyze, analyze_image

    result = analyze(frame_bgr)          # 输入 OpenCV BGR 图像
    result = analyze_image("test.png")   # 输入图片路径

命令行测试
----------
支持单张图片或文件夹批处理，只生成 JSON。
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class Config:
    # 判定阈值
    MAX_OVERHANG_PERCENT: float = 5.0

    # 蓝色托盘 HSV
    BLUE_H_MIN: int = 80
    BLUE_H_MAX: int = 115
    BLUE_S_MIN: int = 40
    BLUE_S_MAX: int = 255
    BLUE_V_MIN: int = 40
    BLUE_V_MAX: int = 255

    # 蓝色托盘候选过滤
    MIN_BLUE_COMPONENT_AREA_RATIO: float = 0.008
    MIN_PALLET_WIDTH_RATIO: float = 0.25
    PALLET_CENTER_TOLERANCE_RATIO: float = 0.32
    PALLET_BOUND_QUANTILE: float = 0.005

    # 白色货物
    WHITE_S_MAX: int = 42
    WHITE_V_MIN: int = 135
    GOODS_SIDE_SEARCH_RATIO: float = 0.20
    WHITE_COLUMN_OCCUPANCY_THRESHOLD: float = 0.42
    COLUMN_SMOOTH_KERNEL: int = 11
    GOODS_VERTICAL_MARGIN_PX: int = 8
    TOP_SEARCH_START_RATIO: float = 0.04


DEFAULT_CONFIG = Config()

IMAGE_SUFFIXES = {
    ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"
}


def _make_blue_mask(image: np.ndarray, cfg: Config) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(
        hsv,
        np.array(
            [cfg.BLUE_H_MIN, cfg.BLUE_S_MIN, cfg.BLUE_V_MIN],
            dtype=np.uint8,
        ),
        np.array(
            [cfg.BLUE_H_MAX, cfg.BLUE_S_MAX, cfg.BLUE_V_MAX],
            dtype=np.uint8,
        ),
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
    )
    return mask


def _detect_blue_pallets(image: np.ndarray, cfg: Config) -> list[dict]:
    mask = _make_blue_mask(image, cfg)
    h, w = mask.shape

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask,
        connectivity=8,
    )

    min_area = h * w * cfg.MIN_BLUE_COMPONENT_AREA_RATIO
    min_width = w * cfg.MIN_PALLET_WIDTH_RATIO
    image_center_x = w / 2.0
    center_tolerance = w * cfg.PALLET_CENTER_TOLERANCE_RATIO
    q = cfg.PALLET_BOUND_QUANTILE

    pallets = []

    for label_id in range(1, count):
        _, _, bw, _, area = stats[label_id]
        cx, cy = centroids[label_id]

        if area < min_area:
            continue
        if bw < min_width:
            continue
        if abs(cx - image_center_x) > center_tolerance:
            continue

        ys, xs = np.nonzero(labels == label_id)
        if len(xs) == 0:
            continue

        left = int(round(np.quantile(xs, q)))
        right = int(round(np.quantile(xs, 1.0 - q)))
        top = int(round(np.quantile(ys, q)))
        bottom = int(round(np.quantile(ys, 1.0 - q)))

        pallets.append({
            "left": left,
            "right": right,
            "top": top,
            "bottom": bottom,
            "width": max(1, right - left),
            "center_y": float(cy),
        })

    pallets.sort(key=lambda p: p["center_y"])
    return pallets


def _make_white_mask(image: np.ndarray, cfg: Config) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = (
        (hsv[:, :, 1] <= cfg.WHITE_S_MAX)
        & (hsv[:, :, 2] >= cfg.WHITE_V_MIN)
    ).astype(np.uint8)

    return cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
    )


def _smooth(values: np.ndarray, kernel_size: int) -> np.ndarray:
    k = max(3, int(kernel_size))
    if k % 2 == 0:
        k += 1
    return np.convolve(
        values,
        np.ones(k, dtype=np.float64) / k,
        mode="same",
    )


def _goods_side_x(
    white_mask: np.ndarray,
    expected_x: int,
    y0: int,
    y1: int,
    side: str,
    search_radius: int,
    cfg: Config,
) -> int | None:
    h, w = white_mask.shape

    y0 = max(0, min(h - 1, int(y0)))
    y1 = max(y0 + 1, min(h, int(y1)))

    if y1 - y0 < 20:
        return None

    x0 = max(0, expected_x - search_radius)
    x1 = min(w - 1, expected_x + search_radius)

    if x1 <= x0:
        return None

    occupancy = np.mean(
        white_mask[y0:y1, :] > 0,
        axis=0,
    ).astype(np.float64)

    occupancy = _smooth(
        occupancy,
        cfg.COLUMN_SMOOTH_KERNEL,
    )

    threshold = cfg.WHITE_COLUMN_OCCUPANCY_THRESHOLD

    if side == "left":
        candidates = [
            x
            for x in range(x0 + 1, x1 + 1)
            if occupancy[x] >= threshold
            and occupancy[x - 1] < threshold
        ]

        if candidates:
            return min(candidates, key=lambda x: abs(x - expected_x))

        valid = np.flatnonzero(
            occupancy[x0:x1 + 1] >= threshold
        )
        return None if len(valid) == 0 else int(x0 + valid[0])

    candidates = [
        x
        for x in range(x0, x1)
        if occupancy[x] >= threshold
        and occupancy[x + 1] < threshold
    ]

    if candidates:
        return min(candidates, key=lambda x: abs(x - expected_x))

    valid = np.flatnonzero(
        occupancy[x0:x1 + 1] >= threshold
    )
    return None if len(valid) == 0 else int(x0 + valid[-1])


def _detect_goods_pair(
    white_mask: np.ndarray,
    middle_pallet: dict,
    y0: int,
    y1: int,
    cfg: Config,
) -> tuple[int | None, int | None]:
    search_radius = max(
        30,
        int(round(
            middle_pallet["width"] * cfg.GOODS_SIDE_SEARCH_RATIO
        )),
    )

    left = _goods_side_x(
        white_mask,
        middle_pallet["left"],
        y0,
        y1,
        "left",
        search_radius,
        cfg,
    )

    right = _goods_side_x(
        white_mask,
        middle_pallet["right"],
        y0,
        y1,
        "right",
        search_radius,
        cfg,
    )

    return left, right


def analyze(
    image_bgr: np.ndarray,
    cfg: Config = DEFAULT_CONFIG,
) -> dict:
    """
    系统集成主接口。

    参数
    ----
    image_bgr:
        OpenCV BGR 图像。

    返回
    ----
    {
        "overhang_percent": float,
        "decision": "ACCEPT" | "REJECT"
    }
    """
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("输入图像为空")

    h = image_bgr.shape[0]

    pallets = _detect_blue_pallets(image_bgr, cfg)
    if len(pallets) < 2:
        raise RuntimeError(
            f"有效蓝色托盘不足：只检测到 {len(pallets)} 个"
        )

    # 最下面：待插取托盘
    bottom = pallets[-1]

    # 底层托盘上方距离最近的蓝色托盘
    middle = pallets[-2]

    white_mask = _make_white_mask(image_bgr, cfg)
    margin = cfg.GOODS_VERTICAL_MARGIN_PX

    top_left, top_right = _detect_goods_pair(
        white_mask,
        middle,
        int(round(h * cfg.TOP_SEARCH_START_RATIO)),
        middle["top"] - margin,
        cfg,
    )

    lower_left, lower_right = _detect_goods_pair(
        white_mask,
        middle,
        middle["bottom"] + margin,
        bottom["top"] - margin,
        cfg,
    )

    left_candidates = [
        x
        for x in (top_left, middle["left"], lower_left)
        if x is not None
    ]
    right_candidates = [
        x
        for x in (top_right, middle["right"], lower_right)
        if x is not None
    ]

    if not left_candidates or not right_candidates:
        raise RuntimeError("上部对象左右边界检测失败")

    upper_left = min(left_candidates)
    upper_right = max(right_candidates)

    bottom_width = bottom["right"] - bottom["left"]
    if bottom_width <= 1:
        raise RuntimeError("底层托盘像素宽度异常")

    left_overhang = max(0, bottom["left"] - upper_left)
    right_overhang = max(0, upper_right - bottom["right"])
    max_overhang = max(left_overhang, right_overhang)

    overhang_percent = 100.0 * max_overhang / bottom_width

    return {
        "overhang_percent": round(float(overhang_percent), 3),
        "decision": (
            "REJECT"
            if overhang_percent > cfg.MAX_OVERHANG_PERCENT
            else "ACCEPT"
        ),
    }


def analyze_image(
    image_path: str | Path,
    cfg: Config = DEFAULT_CONFIG,
) -> dict:
    """读取一张图片并返回检测结果，不写文件。"""
    image_path = Path(image_path)
    image = cv2.imread(str(image_path))

    if image is None:
        raise RuntimeError(f"图片读取失败：{image_path}")

    return analyze(image, cfg)


def process_path(
    input_path: str | Path,
    output_dir: str | Path,
    cfg: Config = DEFAULT_CONFIG,
) -> None:
    """
    测试/离线批处理接口。
    输入单图或文件夹，只保存 JSON，不保存图片。
    """
    input_path = Path(input_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if input_path.is_file():
        image_paths = [input_path]
    elif input_path.is_dir():
        image_paths = sorted(
            p
            for p in input_path.iterdir()
            if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
        )
    else:
        raise FileNotFoundError(f"输入路径不存在：{input_path}")

    if not image_paths:
        raise RuntimeError(f"没有找到可处理图片：{input_path}")

    for image_path in image_paths:
        result = analyze_image(image_path, cfg)

        json_path = output_dir / f"{image_path.stem}.json"
        json_path.write_text(
            json.dumps(
                result,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

        print(
            f"[OK] {image_path.name} | "
            f"{result['overhang_percent']:.3f}% | "
            f"{result['decision']}"
        )


def _main() -> int:
    parser = argparse.ArgumentParser(
        description="托盘横向超限检测，只输出 JSON"
    )
    parser.add_argument(
        "input",
        help="单张图片或图片文件夹",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default="output",
        help="JSON 输出目录",
    )
    parser.add_argument(
        "--threshold-percent",
        type=float,
        default=DEFAULT_CONFIG.MAX_OVERHANG_PERCENT,
        help="最大允许超出比例，默认 5%%",
    )

    args = parser.parse_args()

    cfg = Config(
        MAX_OVERHANG_PERCENT=args.threshold_percent
    )

    process_path(
        args.input,
        args.output_dir,
        cfg,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
