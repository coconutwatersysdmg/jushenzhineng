from __future__ import annotations

import argparse
import json
from pathlib import Path

from camera_world_module import D435iCamera, YoloD435iWorldLocalizer


POINT_KEYS = ("P1", "P2", "P3", "P4")


def parse_pose(values):
    x, y, z, r = (float(v) for v in values)
    return {"x": x, "y": y, "z": z, "r": r}


def write_result(path: str | Path, result):
    clean = {name: [float(v) for v in result[name]] for name in POINT_KEYS}
    Path(path).write_text(json.dumps(clean, ensure_ascii=False, indent=2), encoding="utf-8")
    return clean


def build_parser():
    parser = argparse.ArgumentParser(
        description="D435i + YOLO 双位置角点精定位：最终只输出 P1-P4 世界坐标(mm)"
    )
    parser.add_argument(
        "--p3p4-pose", nargs=4, type=float, metavar=("X", "Y", "Z", "R"), required=True,
        help="拍摄 P3/P4 时的 PLC 位姿，单位 mm/mm/mm/deg",
    )
    parser.add_argument(
        "--p1p2-pose", nargs=4, type=float, metavar=("X", "Y", "Z", "R"), required=True,
        help="拍摄 P1/P2 时的 PLC 位姿，单位 mm/mm/mm/deg",
    )
    parser.add_argument("--output", default="result.json", help="结果 JSON 路径")
    parser.add_argument("--serial", default=None, help="可选：指定 D435i 序列号")
    return parser


def main():
    args = build_parser().parse_args()
    p3p4_pose = parse_pose(args.p3p4_pose)
    p1p2_pose = parse_pose(args.p1p2_pose)
    localizer = YoloD435iWorldLocalizer()

    print("正在连接 D435i ...")
    with D435iCamera(serial=args.serial) as camera:
        input("请确认叉车臂已到 P3/P4 拍摄位姿，然后按 Enter 采集 ...")
        p3p4_frame = camera.capture()
        p3p4 = localizer.detect_pair(p3p4_frame, p3p4_pose, ("P3", "P4"))
        print("P3/P4 识别完成。")

        input("请确认叉车臂已到 P1/P2 拍摄位姿，然后按 Enter 采集 ...")
        p1p2_frame = camera.capture()
        p1p2 = localizer.detect_pair(p1p2_frame, p1p2_pose, ("P1", "P2"))
        print("P1/P2 识别完成。")

    result = {
        "P1": p1p2["P1"],
        "P2": p1p2["P2"],
        "P3": p3p4["P3"],
        "P4": p3p4["P4"],
    }
    clean = write_result(args.output, result)
    print(json.dumps(clean, ensure_ascii=False, indent=2))
    print(f"结果已保存：{Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
