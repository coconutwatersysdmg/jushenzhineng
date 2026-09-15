from __future__ import annotations

import argparse
import json
from pathlib import Path

from lidar_module import process_pcd


def main() -> int:
    parser = argparse.ArgumentParser(description="实验室雷达点云处理算法")
    parser.add_argument("pcd", help="输入 PCD 文件")
    parser.add_argument("-o", "--output", help="输出 JSON 文件；不填写则仅在终端显示")
    args = parser.parse_args()

    result = process_pcd(args.pcd, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
