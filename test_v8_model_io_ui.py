# -*- coding: utf-8 -*-
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from ui.main_window import MainWindow


app=QApplication.instance() or QApplication([])
window=MainWindow(); window.show(); app.processEvents()

# Future steps are not exposed before the live workflow reaches them.
assert window.right_tabs.tabText(2) == "模型输入/输出"
assert window.module_table.rowCount() == 0
assert "未执行步骤不提前显示" in window.module_summary.text()

sample=Path("examples/demo_pre_pick_offset.jpg").resolve()
window.controller.module_evidence.record(
    "PALLET_OVERHANG_PRE",
    {"image_path":str(sample),"threshold_percent":5.0,"cargo_id":"UI-TEST"},
    {"success":True,"result_image_path":str(sample),"overhang_percent":0.0,"decision":"ACCEPT"},
    12.5,
)
window._refresh(); app.processEvents()
assert window.module_table.rowCount() == 1
assert "放置前货物-托盘横向偏移" in window.current_module_title.text()
assert "threshold_percent" in window.module_input_data.toPlainText()
assert "overhang_percent" in window.module_output_data.toPlainText()
assert window.module_input_preview.pixmap() and not window.module_input_preview.pixmap().isNull()
assert window.module_output_preview.pixmap() and not window.module_output_preview.pixmap().isNull()
window.close(); app.processEvents()
print("V8_MODEL_IO_UI_OK",window.module_table.rowCount())
