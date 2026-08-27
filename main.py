# -*- coding: utf-8 -*-
from __future__ import annotations

import faulthandler
import ctypes
import os
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
VENV_PYTHON = PROJECT_ROOT / "venv" / "Scripts" / "python.exe"
STARTUP_LOG = PROJECT_ROOT / "runtime" / "startup.log"
FAULT_LOG = PROJECT_ROOT / "runtime" / "startup_fault.log"
INSTANCE_SERVER_NAME = "jushenzhineng-v8-main-window"
INSTANCE_MUTEX_NAME = "Local\\JushenzhinengV8MainWindow"
_FAULT_STREAM = None


def _log_startup(message: str) -> None:
    try:
        STARTUP_LOG.parent.mkdir(parents=True, exist_ok=True)
        with STARTUP_LOG.open("a", encoding="utf-8") as stream:
            stream.write(f"[{datetime.now().isoformat(timespec='milliseconds')}] pid={os.getpid()} {message}\n")
    except Exception:
        pass


def _redirect_to_project_python() -> None:
    """Make ``python main.py`` and double-click use the project environment.

    Windows may associate ``.py`` files (and the ``python`` command) with a
    global interpreter that does not contain PySide6.  Replace that process
    with the checked project interpreter before importing any GUI dependency.
    """
    if os.name != "nt" or not VENV_PYTHON.is_file():
        return
    if os.environ.get("JUSHEN_VENV_BOOTSTRAPPED") == "1":
        return
    try:
        current = Path(sys.executable).resolve()
        target = VENV_PYTHON.resolve()
        if current == target:
            return
        _log_startup(f"BOOTSTRAP_REDIRECT from={current} to={target}")
        environment = os.environ.copy()
        environment["JUSHEN_VENV_BOOTSTRAPPED"] = "1"
        environment.setdefault("PYTHONUTF8", "1")
        os.chdir(PROJECT_ROOT)
        os.execve(
            str(target),
            [str(target), str(Path(__file__).resolve()), *sys.argv[1:]],
            environment,
        )
    except Exception:
        _log_startup("BOOTSTRAP_REDIRECT_FAILED\n" + traceback.format_exc().rstrip())


def _enable_fault_log() -> None:
    global _FAULT_STREAM
    try:
        FAULT_LOG.parent.mkdir(parents=True, exist_ok=True)
        _FAULT_STREAM = FAULT_LOG.open("a", encoding="utf-8")
        faulthandler.enable(file=_FAULT_STREAM, all_threads=True)
    except Exception:
        _FAULT_STREAM = None


def _remove_legacy_lock() -> None:
    """Remove only the obsolete lock created before local activation IPC."""
    try:
        from tempfile import gettempdir
        legacy = Path(gettempdir()).resolve() / "jushenzhineng_v8.lock"
        if legacy.is_file():
            legacy.unlink()
            _log_startup(f"LEGACY_LOCK_REMOVED path={legacy}")
    except Exception as exc:
        _log_startup(f"LEGACY_LOCK_REMOVE_WARNING {exc}")


def _send_activation(server_name: str = INSTANCE_SERVER_NAME, timeout_ms: int = 350) -> bool:
    from PySide6.QtNetwork import QLocalSocket

    socket = QLocalSocket()
    socket.connectToServer(server_name)
    if not socket.waitForConnected(timeout_ms):
        socket.abort()
        return False
    socket.write(b"ACTIVATE\n")
    socket.flush()
    socket.waitForBytesWritten(timeout_ms)
    socket.disconnectFromServer()
    return True


def _acquire_instance_guard(name: str = INSTANCE_MUTEX_NAME):
    """Return (handle, already_exists); Windows releases the guard on any process exit."""
    if os.name != "nt":
        return None, False
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    create_mutex = kernel32.CreateMutexW
    create_mutex.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p]
    create_mutex.restype = ctypes.c_void_p
    # CreateMutexW only guarantees LastError when the mutex already exists.
    # Clear a stale ERROR_ALREADY_EXISTS left by an unrelated Win32 call first,
    # otherwise a cold launch can be mistaken for a duplicate instance.
    ctypes.set_last_error(0)
    handle = create_mutex(None, False, name)
    already_exists = ctypes.get_last_error() == 183  # ERROR_ALREADY_EXISTS
    return handle, already_exists


def _release_instance_guard(handle) -> None:
    if handle and os.name == "nt":
        ctypes.windll.kernel32.CloseHandle(ctypes.c_void_p(handle))


def _raise_existing_window_windows() -> bool:
    if os.name != "nt":
        return False
    user32 = ctypes.windll.user32
    found = []
    callback_type = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)

    def visit(hwnd, _):
        length = user32.GetWindowTextLengthW(hwnd)
        if length <= 0:
            return True
        buffer = ctypes.create_unicode_buffer(length + 1)
        user32.GetWindowTextW(hwnd, buffer, length + 1)
        # Match the stable ASCII product version as well as the Chinese title;
        # this remains reliable under mixed Windows code-page environments.
        title = buffer.value.casefold()
        if "v8" in title:
            user32.ShowWindow(hwnd, 9)  # SW_RESTORE
            user32.BringWindowToTop(hwnd)
            user32.SetForegroundWindow(hwnd)
            user32.FlashWindow(hwnd, True)
            found.append(hwnd)
            return False
        return True

    user32.EnumWindows(callback_type(visit), 0)
    return bool(found)


def _activate_existing_with_retry(timeout_s: float = 4.0) -> bool:
    deadline = time.perf_counter() + max(0.2, float(timeout_s))
    server_accepted = False
    while time.perf_counter() < deadline:
        # Raise the HWND directly first; a successful socket write only proves
        # the pipe accepted bytes, not that a busy GUI event loop processed them.
        if _raise_existing_window_windows():
            _send_activation(timeout_ms=80)
            return True
        server_accepted = _send_activation(timeout_ms=220) or server_accepted
        time.sleep(0.12)
    return server_accepted


def _activate_existing_native_retry(timeout_s: float = 6.0) -> bool:
    """Activate without importing/creating a second Qt application."""
    deadline = time.perf_counter() + max(0.2, float(timeout_s))
    while time.perf_counter() < deadline:
        if _raise_existing_window_windows():
            return True
        time.sleep(0.10)
    return False


def _start_instance_server(server_name: str = INSTANCE_SERVER_NAME):
    from PySide6.QtNetwork import QLocalServer

    server = QLocalServer()
    if server.listen(server_name):
        return server
    _log_startup(f"INSTANCE_SERVER_DISABLED error={server.errorString()}")
    return False


def _make_splash():
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QColor, QFont, QPainter, QPixmap
    from PySide6.QtWidgets import QSplashScreen

    pixmap = QPixmap(680, 260)
    pixmap.fill(QColor("#07111f"))
    painter = QPainter(pixmap)
    painter.setPen(QColor("#74d8ff"))
    painter.setFont(QFont("Microsoft YaHei UI", 22, QFont.Weight.Bold))
    painter.drawText(36, 90, "具身智能装载数字孪生 v8")
    painter.setPen(QColor("#dcecff"))
    painter.setFont(QFont("Microsoft YaHei UI", 12))
    painter.drawText(38, 140, "正在加载相机、算法服务与三维场景，请稍候……")
    painter.setPen(QColor("#6f8ca5"))
    painter.drawText(38, 195, "再次运行会自动唤醒已有窗口，无需连续重复点击。")
    painter.end()
    splash = QSplashScreen(pixmap, Qt.WindowType.WindowStaysOnTopHint)
    splash.show()
    return splash


def main() -> int:
    _redirect_to_project_python()
    started = time.perf_counter()
    _enable_fault_log()
    _log_startup("BOOT begin")
    _remove_legacy_lock()
    instance_guard, already_running = _acquire_instance_guard()
    _log_startup(f"INSTANCE_GUARD already_running={already_running}")
    if already_running and os.name == "nt":
        activated = _activate_existing_native_retry()
        _log_startup(f"ACTIVATION_EXISTING_NATIVE success={activated}")
        _release_instance_guard(instance_guard)
        return 0
    try:
        from PySide6.QtCore import QTimer
        from PySide6.QtWidgets import QApplication, QMessageBox
    except Exception:
        detail = traceback.format_exc()
        _log_startup("PYSIDE_IMPORT_FAILED\n" + detail.rstrip())
        print(detail, file=sys.stderr)
        _release_instance_guard(instance_guard)
        return 1

    app = QApplication(sys.argv)
    app.setApplicationName("Embodied Loading Digital Twin v8")
    _log_startup(f"QT_READY platform={app.platformName()} elapsed={time.perf_counter()-started:.3f}s")

    if already_running:
        activated = _activate_existing_with_retry()
        _log_startup(f"ACTIVATION_EXISTING success={activated}")
        _release_instance_guard(instance_guard)
        return 0
    instance_server = _start_instance_server()

    splash = _make_splash()
    app.processEvents()
    _log_startup("WINDOW_IMPORT begin")
    try:
        from ui.main_window import MainWindow

        window = MainWindow()

        def activate_existing_window():
            if instance_server:
                while instance_server.hasPendingConnections():
                    connection = instance_server.nextPendingConnection()
                    connection.waitForReadyRead(100)
                    connection.readAll()
                    connection.disconnectFromServer()
            window.showNormal()
            window.raise_()
            window.activateWindow()
            QApplication.alert(window, 1600)
            _log_startup("ACTIVATED existing window raised")

        if instance_server:
            instance_server.newConnection.connect(activate_existing_window)
            if instance_server.hasPendingConnections():
                QTimer.singleShot(0, activate_existing_window)

        window.show()
        window.raise_()
        window.activateWindow()
        splash.finish(window)
        elapsed = time.perf_counter() - started
        screen = window.screen().availableGeometry()
        _log_startup(f"READY main window shown elapsed={elapsed:.3f}s screen={screen.width()}x{screen.height()}")

        smoke_ms = int(os.environ.get("JUSHEN_STARTUP_SMOKE_MS", "0") or 0)
        if smoke_ms > 0:
            QTimer.singleShot(smoke_ms, app.quit)
        rc = app.exec()
        _log_startup(f"EXIT code={rc}")
        _release_instance_guard(instance_guard)
        return rc
    except Exception:
        detail = traceback.format_exc()
        _log_startup("FAILED\n" + detail.rstrip())
        splash.close()
        QMessageBox.critical(None, "启动失败", f"程序启动失败，详细信息已写入：\n{STARTUP_LOG}\n\n{detail}")
        _release_instance_guard(instance_guard)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
