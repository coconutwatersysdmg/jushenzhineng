# -*- coding: utf-8 -*-
"""梅卡采图 worker：在 runtime_mecheye（Python 3.11）里运行，向 stdout 打一行 JSON。"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="")
    parser.add_argument("--tag", default="pallet")
    parser.add_argument("--out-dir", default="")
    args = parser.parse_args()

    # 保证能 import 项目 devices（._pth 含 .. 时一般已有）
    root = Path(__file__).resolve().parents[1]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

    from devices.mechmind_capture import capture_rgbd_inprocess

    result = capture_rgbd_inprocess(
        ip=str(args.ip or "").strip(),
        save_dir=args.out_dir or None,
        tag=str(args.tag or "pallet"),
    )
    sys.stdout.write(json.dumps(result, ensure_ascii=False))
    sys.stdout.flush()
    return 0 if result.get("success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
