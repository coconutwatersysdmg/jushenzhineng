# -*- coding: utf-8 -*-
"""独立 PLC 控制台：接收主系统 JSON 指令并驱动龙门架。

启动：
  python -m plc_console
  或双击 / 运行 plc_console/main.py
"""

from .main import main

__all__ = ["main"]
