from core.digital_twin_state import DigitalTwinState
from devices.real_plc_adapter import RealPlcAdapter
from plc_console.motion.runner import PlcMotionRunner


class _FakeMotion:
    connected = True

    def __init__(self, current_r):
        self.current_r = float(current_r)
        self.targets = None

    def move_absolute(self, targets, **kwargs):
        self.targets = dict(targets)
        return {"success": True, "targets": dict(targets), "message": "ok"}

    def read_positions(self):
        return {"X": 10.0, "Y": 20.0, "Z": 30.0, "R": self.current_r}


def test_real_plc_motion_keeps_physical_r_instead_of_requested_r():
    plc = RealPlcAdapter(DigitalTwinState(), {"hold_r_axis": True})
    motion = _FakeMotion(current_r=37.5)
    plc.connected = True
    plc._motion = motion

    result = plc.move_absolute_xyzr(
        {"X": 100.0, "Y": 200.0, "Z": 300.0, "R": 90.0},
    )

    assert result["success"] is True
    assert motion.targets["R"] == 37.5
    assert motion.targets["X"] == 100.0
    assert motion.targets["Y"] == 200.0
    assert motion.targets["Z"] == 300.0

    motion.current_r = 12.0
    plc.move_absolute_xyzr({"X": 110.0, "Y": 210.0, "Z": 310.0, "R": 0.0})
    assert motion.targets["R"] == 37.5


def test_plc_console_locks_r_for_the_whole_session():
    runner = PlcMotionRunner({"plc": {"ip": "127.0.0.1"}, "gantry": {"hold_r_axis": True}})
    motion = _FakeMotion(current_r=41.0)
    runner._motion = motion

    runner.execute_command({"gantry_xyzr": {"X": 1, "Y": 2, "Z": 3, "R": 90}})
    motion.current_r = -15.0
    result = runner.execute_command({"gantry_xyzr": {"X": 4, "Y": 5, "Z": 6, "R": 0}})

    assert result["targets"]["R"] == 41.0
    assert result["locked_r_deg"] == 41.0
