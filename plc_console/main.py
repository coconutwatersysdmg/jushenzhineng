# -*- coding: utf-8 -*-
"""plc_console 入口。"""
from __future__ import annotations

import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def main() -> int:
    os.environ.setdefault("PYTHONUTF8", "1")
    from PySide6.QtWidgets import QApplication
    from plc_console.ui.main_window import PlcConsoleWindow

    app = QApplication(sys.argv)
    app.setApplicationName("PLC Motion Console")
    window = PlcConsoleWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
