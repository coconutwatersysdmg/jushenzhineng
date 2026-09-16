# -*- coding: utf-8 -*-
"""Two-process IPC smoke test for plc_console bridge."""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PY = ROOT / "runtime" / "python.exe"
EXE = str(PY if PY.is_file() else sys.executable)

SERVER = r"""
import sys
from pathlib import Path
ROOT = Path(r"%s")
sys.path.insert(0, str(ROOT))
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from plc_console.bridge import PlcBridgeServer

app = QApplication([])
got = []
srv = PlcBridgeServer()
srv.commandReceived.connect(lambda c: (got.append(c), print("GOT", c.get("task"), flush=True)))
print("START", srv.start(), flush=True)
QTimer.singleShot(6000, app.quit)
app.exec()
print("COUNT", len(got), flush=True)
""" % str(ROOT)


def main() -> int:
    proc = subprocess.Popen([EXE, "-c", SERVER], cwd=str(ROOT))
    time.sleep(1.5)
    sys.path.insert(0, str(ROOT))
    from contracts.plc_motion_cmd import build_motion_command
    from services.plc_motion_publisher import PlcMotionPublisher

    cmd = build_motion_command(
        robot_id="PICK_ARM",
        world_pose={"x_mm": 0, "y_mm": 0, "z_mm": 0},
        task="IPC2",
        gantry_xyzr={"X": 1, "Y": 2, "Z": 3, "R": 4},
    )
    res = PlcMotionPublisher().push(cmd)
    print("CLIENT", res)
    rc = proc.wait(timeout=10)
    ok = bool(res.get("success")) and rc == 0
    print("RESULT", "OK" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
