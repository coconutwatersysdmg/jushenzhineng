# -*- coding: utf-8 -*-
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QSG_RHI_BACKEND", "software")

from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ui.main_window import MainWindow


app = QApplication.instance() or QApplication([])
window = MainWindow()
window.resize(1400, 850)
window.show()
app.processEvents()
quick = window.quick
root = quick.rootObject()

# Left-button drag must update the orbit immediately and release capture.
yaw0 = float(root.property("orbitYaw"))
pitch0 = float(root.property("orbitPitch"))
start = QPoint(160, quick.height() - 180)
end = QPoint(280, quick.height() - 140)
QTest.mousePress(quick, Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier, start)
QTest.mouseMove(quick, end, 80)
QTest.mouseRelease(quick, Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier, end)
app.processEvents()
assert float(root.property("orbitYaw")) - yaw0 > 20.0
assert abs(float(root.property("orbitPitch")) - pitch0) > 5.0
assert bool(root.property("orbitDragging")) is False
assert float(root.property("wheelZoomSensitivity")) >= 0.030

# Click the four fixed-view buttons: left/right and head/tail must not be reversed.
panel_width = min(
    360 if quick.width() < 800 else max(400, quick.width() * 0.54),
    quick.width() - 20,
)
panel_x = quick.width() - 10 - panel_width
button_width = (panel_width - 66) / 6
actual = []
for index in (2, 3, 4, 5):
    point = QPoint(int(panel_x + 8 + index * (button_width + 5) + button_width / 2), 34)
    QTest.mouseClick(quick, Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier, point)
    QTest.qWait(380)
    actual.append(round(float(root.property("orbitYaw"))))
assert actual == [-90, 90, 0, 180], actual

window.close()
print("V8_SCENE_CONTROLS_OK", actual, root.property("wheelZoomSensitivity"))
