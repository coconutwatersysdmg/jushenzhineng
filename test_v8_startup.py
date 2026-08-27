# -*- coding: utf-8 -*-
import os
from uuid import uuid4

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import QApplication

from main import (
    _acquire_instance_guard, _make_splash, _release_instance_guard,
    _send_activation, _start_instance_server,
)
from ui.main_window import MainWindow


app = QApplication.instance() or QApplication([])
splash = _make_splash()
window = MainWindow()
window.show()
app.processEvents()
assert window.quick.status() == QQuickWidget.Status.Ready, [e.toString() for e in window.quick.errors()]
assert int(window.quick.rootObject().property("cargoVisualCount")) == 3
assert len(window.flow_nodes) == 12
window.resize(1100, 700)
app.processEvents()
main_sizes=window.main_splitter.sizes()
assert main_sizes[1] >= 350, main_sizes
assert main_sizes[2] >= 240, main_sizes
assert window.message.height() >= 105, window.message.height()
assert min(node.width() for node in window.flow_nodes) >= 90, [node.width() for node in window.flow_nodes]
splash.finish(window)

server_name=f"jushenzhineng-startup-test-{uuid4().hex}"
server=_start_instance_server(server_name)
assert server
assert _send_activation(server_name,500) is True
app.processEvents()
assert server.hasPendingConnections() is True
while server.hasPendingConnections():
    connection=server.nextPendingConnection(); connection.disconnectFromServer()
server.close()

mutex_name=f"Local\\JushenzhinengStartupTest{uuid4().hex}"
first_handle,first_exists=_acquire_instance_guard(mutex_name)
second_handle,second_exists=_acquire_instance_guard(mutex_name)
assert first_handle and first_exists is False
assert second_handle and second_exists is True
_release_instance_guard(second_handle); _release_instance_guard(first_handle)

window.close()
app.processEvents()
print("V8_STARTUP_OK", window.quick.status(), main_sizes, window.message.height())
