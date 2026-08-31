import json
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import QEvent, QEventLoop, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QButtonGroup,
    QDialog,
    QDoubleSpinBox,
    QFrame,
    QGraphicsScene,
    QGraphicsView,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QTextEdit,
    QToolTip,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

try:
    from pymodbus.client import ModbusTcpClient
except Exception:
    try:
        from pymodbus.client.sync import ModbusTcpClient
    except Exception:
        ModbusTcpClient = None


DEVICE_ID = 1

MODE_ADDRESS = 5
ESTOP_ADDRESS = 7

AXIS_CONFIG = {
    "X": {
        "jog_address": 12,
        "home_address": 10,
        "home_done_address": 37868,
        "reset_address": 113,
        "pause_address": 129,
        "absolute_trigger_address": 18,
        "absolute_done_address": 117,
        "absolute_position_address": 41208,
        "absolute_speed_address": 41210,
        "position_address": 47234,
        "actual_speed_address": 29680,
        "alarm_address": 119,
    },
    "Y": {
        "jog_address": 22,
        "home_address": 20,
        "home_done_address": 37888,
        "reset_address": 213,
        "pause_address": 229,
        "absolute_trigger_address": 28,
        "absolute_done_address": 217,
        "absolute_position_address": 41308,
        "absolute_speed_address": 41310,
        "position_address": 47238,
        "actual_speed_address": 29700,
        "alarm_address": 219,
    },
    "Z": {
        "jog_address": 32,
        "home_address": 30,
        "home_done_address": 37928,
        "reset_address": 313,
        "pause_address": 329,
        "absolute_trigger_address": 38,
        "absolute_done_address": 317,
        "absolute_position_address": 41408,
        "absolute_speed_address": 41410,
        "position_address": 47246,
        "actual_speed_address": 29740,
        "alarm_address": 319,
    },
    "R": {
        "jog_address": 42,
        "home_address": 40,
        "home_done_address": 37948,
        "reset_address": 413,
        "pause_address": 429,
        "absolute_trigger_address": 48,
        "absolute_done_address": 417,
        "absolute_position_address": 41508,
        "absolute_speed_address": 41510,
        "position_address": 47250,
        "actual_speed_address": 29760,
        "alarm_address": 419,
    },
}

MOTION_CLEAR_KEYS = (
    "jog_address",
    "home_address",
    "absolute_trigger_address",
    "pause_address",
)


AXIS_UNITS = {
    "X": ("mm", "mm/s"),
    "Y": ("mm", "mm/s"),
    "Z": ("mm", "mm/s"),
    "R": ("°", "°/s"),
}

SOFT_LIMITS = {
    "X": (0.0, 500.0),
    "Y": (0.0, 1300.0),
    "Z": (0.0, 380.0),
    "R": (-180.0, 180.0),
}

DEFAULT_SETTINGS = {
    "plc_ip": "192.168.6.6",
    "plc_port": "502",
    "default_speed": 30.0,
    "axis_default_speeds": {
        "X": 30.0,
        "Y": 50.0,
        "Z": 30.0,
        "R": 30.0,
    },
    "axis_step": {
        "XYZ": 50.0,
        "R": 10.0,
    },
    "soft_limits": {
        "X": [0.0, 500.0],
        "Y": [0.0, 1300.0],
        "Z": [0.0, 380.0],
        "R": [-180.0, 180.0],
    },
    "r_forbidden_zones_enabled": False,
    "r_forbidden_zones": [],
    "work_origin": {
        "X": 0.0,
        "Y": 0.0,
        "Z": 120.0,
        "R": 0.0,
    },
}


@dataclass
class AxisState:
    name: str
    position: float = 0.0
    target: float = 0.0
    speed: float = 30.0
    jog_direction: int = 0
    positioning: bool = False
    alarm: bool = False
    motion_kind: str = ""
    start_position: float = 0.0
    motion_start_time: float = 0.0


class ArrowDoubleSpinBox(QDoubleSpinBox):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setButtonSymbols(QAbstractSpinBox.NoButtons)

        self._up_button = QToolButton(self)
        self._down_button = QToolButton(self)

        for button, arrow_type in (
            (self._up_button, Qt.UpArrow),
            (self._down_button, Qt.DownArrow),
        ):
            button.setObjectName("SpinArrowButton")
            button.setArrowType(arrow_type)
            button.setFocusPolicy(Qt.NoFocus)
            button.setAutoRepeat(True)
            button.setCursor(Qt.ArrowCursor)

        self._up_button.clicked.connect(lambda: self.stepBy(1))
        self._down_button.clicked.connect(lambda: self.stepBy(-1))

    def resizeEvent(self, event):
        super().resizeEvent(event)

        button_width = 24
        button_height = max(14, (self.height() - 4) // 2)
        x = self.width() - button_width - 2
        self._up_button.setGeometry(
            x,
            2,
            button_width,
            button_height,
        )
        self._down_button.setGeometry(
            x,
            2 + button_height,
            button_width,
            button_height,
        )


class SafeDoubleSpinBox(ArrowDoubleSpinBox):
    """
    防误触数字输入框。

    默认状态：
    1. 单击不能修改；
    2. 鼠标滚轮不能修改；
    3. 上下箭头始终显示；
    4. 双击输入框后解锁；
    5. 按 Enter 或失去焦点后重新锁定。
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self._unlocked = False
        self._wheel_enabled_after_unlock = True

        self.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.setAccelerated(True)
        self.setKeyboardTracking(False)

        self.setProperty("editingUnlocked", False)

        self.lineEdit().setReadOnly(True)
        self.lineEdit().installEventFilter(self)

        self.editingFinished.connect(self.lock_editing)

        self.setToolTip(
            "双击输入框后才能修改；解锁后可使用滚轮和右侧上下按钮。"
        )

    def eventFilter(self, watched, event):
        if watched is self.lineEdit():

            if event.type() == QEvent.Wheel:
                return not (
                    self._unlocked
                    and self._wheel_enabled_after_unlock
                )

            if event.type() == QEvent.MouseButtonDblClick:
                self.unlock_editing()
                return True

            if not self._unlocked and event.type() in (
                QEvent.MouseButtonPress,
                QEvent.KeyPress,
                QEvent.ShortcutOverride,
                QEvent.InputMethod,
            ):
                return True

        return super().eventFilter(watched, event)

    def mousePressEvent(self, event):
        if not self._unlocked:
            event.accept()
            return

        super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event):
        self.unlock_editing()
        event.accept()

    def wheelEvent(self, event):
        if (
            self._unlocked
            and self._wheel_enabled_after_unlock
        ):
            super().wheelEvent(event)
            return

        event.ignore()

    def keyPressEvent(self, event):
        if not self._unlocked:
            event.ignore()
            return

        super().keyPressEvent(event)

        if event.key() in (
            Qt.Key_Return,
            Qt.Key_Enter,
        ):
            QTimer.singleShot(
                0,
                self.lock_editing,
            )

    def stepBy(self, steps):
        # 上下调节按钮只有在双击解锁后才生效
        if not self._unlocked:
            QToolTip.showText(
                self.mapToGlobal(
                    self.rect().bottomLeft()
                ),
                "请先双击输入框解锁。",
                self,
            )
            return

        super().stepBy(steps)

    def focusOutEvent(self, event):
        super().focusOutEvent(event)
        self.lock_editing()

    def unlock_editing(self):
        self._unlocked = True

        self.lineEdit().setReadOnly(False)

        self.setProperty(
            "editingUnlocked",
            True,
        )

        self.refresh_style()

        self.lineEdit().setFocus(
            Qt.MouseFocusReason
        )

        self.lineEdit().selectAll()

        QToolTip.showText(
            self.mapToGlobal(
                self.rect().bottomLeft()
            ),
            "已解锁：可输入数值或使用右侧上下按钮。",
            self,
        )

    def lock_editing(self):
        if not self._unlocked:
            return

        self._unlocked = False

        self.lineEdit().setReadOnly(True)

        self.setProperty(
            "editingUnlocked",
            False,
        )

        self.refresh_style()

    def refresh_style(self):
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()


class SettingsDoubleSpinBox(ArrowDoubleSpinBox):
    def wheelEvent(self, event):
        event.ignore()


class AxisCard(QGroupBox):
    jog_started = pyqtSignal(str, int)
    jog_stopped = pyqtSignal(str)

    home_clicked = pyqtSignal(str)
    reset_clicked = pyqtSignal(str)
    stop_clicked = pyqtSignal(str)

    absolute_clicked = pyqtSignal(
        str,
        float,
        float,
    )

    relative_clicked = pyqtSignal(
        str,
        float,
        float,
    )

    def __init__(self, axis_name, default_speed=30.0):
        super().__init__(
            f"{axis_name} 轴控制"
        )

        self.axis_name = axis_name

        position_unit, speed_unit = (
            AXIS_UNITS[axis_name]
        )

        self.position_label = QLabel(
            f"0.00 {position_unit}"
        )

        self.position_label.setObjectName(
            "AxisPosition"
        )

        self.position_label.setAlignment(
            Qt.AlignRight
            | Qt.AlignVCenter
        )

        self.status_label = QLabel("待机")

        self.status_label.setObjectName(
            "StatusNormal"
        )

        self.status_label.setAlignment(
            Qt.AlignCenter
        )
        self.status_label.setMinimumWidth(190)

        self.value_spin = self.make_spin(
            position_unit,
            0.0,
            -999999.0,
        )

        self.speed_spin = self.make_spin(
            speed_unit,
            float(default_speed),
            1.0,
        )

        self.jog_negative_button = QPushButton(
            "点动 −"
        )

        self.jog_positive_button = QPushButton(
            "点动 ＋"
        )

        self.home_button = QPushButton(
            "回原"
        )

        self.reset_button = QPushButton(
            "复位"
        )

        self.absolute_button = QPushButton(
            "绝对定位"
        )

        self.relative_button = QPushButton(
            "相对定位"
        )

        self.stop_button = QPushButton(
            "停止"
        )

        self.blank_button = QPushButton("")

        self.blank_button.setEnabled(False)

        self.blank_button.setObjectName(
            "BlankAxisButton"
        )

        self.axis_buttons = [
            self.jog_negative_button,
            self.jog_positive_button,
            self.home_button,
            self.reset_button,
            self.absolute_button,
            self.relative_button,
            self.stop_button,
            self.blank_button,
        ]

        for button in self.axis_buttons:
            button.setFixedHeight(38)

            button.setSizePolicy(
                QSizePolicy.Expanding,
                QSizePolicy.Fixed,
            )

            if button is not self.blank_button:
                button.setObjectName(
                    "AxisButton"
                )

        self.jog_negative_button.pressed.connect(
            lambda: self.jog_started.emit(
                axis_name,
                -1,
            )
        )

        self.jog_positive_button.pressed.connect(
            lambda: self.jog_started.emit(
                axis_name,
                1,
            )
        )

        self.jog_negative_button.released.connect(
            lambda: self.jog_stopped.emit(
                axis_name
            )
        )

        self.jog_positive_button.released.connect(
            lambda: self.jog_stopped.emit(
                axis_name
            )
        )

        self.home_button.clicked.connect(
            lambda: self.home_clicked.emit(
                axis_name
            )
        )

        self.reset_button.clicked.connect(
            lambda: self.reset_clicked.emit(
                axis_name
            )
        )

        self.stop_button.clicked.connect(
            lambda: self.stop_clicked.emit(
                axis_name
            )
        )

        self.absolute_button.clicked.connect(
            lambda: self.absolute_clicked.emit(
                axis_name,
                float(
                    self.value_spin.value()
                ),
                float(
                    self.speed_spin.value()
                ),
            )
        )

        self.relative_button.clicked.connect(
            lambda: self.relative_clicked.emit(
                axis_name,
                float(
                    self.value_spin.value()
                ),
                float(
                    self.speed_spin.value()
                ),
            )
        )

        root_layout = QVBoxLayout(self)

        root_layout.setContentsMargins(
            16,
            19,
            16,
            14,
        )

        root_layout.setSpacing(9)

        info_layout = QGridLayout()

        info_layout.setHorizontalSpacing(10)
        info_layout.setVerticalSpacing(9)

        info_layout.addWidget(
            QLabel("当前位置"),
            0,
            0,
        )

        info_layout.addWidget(
            self.position_label,
            0,
            1,
        )

        info_layout.addWidget(
            QLabel("运行状态"),
            0,
            2,
        )

        info_layout.addWidget(
            self.status_label,
            0,
            3,
        )

        info_layout.addWidget(
            QLabel("目标 / 位移"),
            1,
            0,
        )

        info_layout.addWidget(
            self.value_spin,
            1,
            1,
        )

        info_layout.addWidget(
            QLabel("运行速度"),
            1,
            2,
        )

        info_layout.addWidget(
            self.speed_spin,
            1,
            3,
        )

        info_layout.setColumnStretch(
            1,
            1,
        )

        info_layout.setColumnStretch(
            3,
            1,
        )

        button_layout = QGridLayout()

        button_layout.setHorizontalSpacing(9)
        button_layout.setVerticalSpacing(8)

        button_layout.addWidget(
            self.jog_negative_button,
            0,
            0,
        )

        button_layout.addWidget(
            self.jog_positive_button,
            0,
            1,
        )

        button_layout.addWidget(
            self.home_button,
            0,
            2,
        )

        button_layout.addWidget(
            self.reset_button,
            0,
            3,
        )

        button_layout.addWidget(
            self.absolute_button,
            1,
            0,
        )

        button_layout.addWidget(
            self.relative_button,
            1,
            1,
        )

        button_layout.addWidget(
            self.stop_button,
            1,
            2,
        )

        button_layout.addWidget(
            self.blank_button,
            1,
            3,
        )

        for column in range(4):
            button_layout.setColumnStretch(
                column,
                1,
            )

        root_layout.addLayout(
            info_layout
        )

        root_layout.addLayout(
            button_layout
        )

    @staticmethod
    def make_spin(
        unit,
        value,
        minimum,
    ):
        spin = SafeDoubleSpinBox()

        spin.setRange(
            minimum,
            999999.0,
        )

        spin.setDecimals(2)
        spin.setSingleStep(1.0)
        spin.setValue(value)
        spin.setSuffix(f" {unit}")
        spin.setFixedHeight(38)

        return spin

    def update_position(self, value):
        unit = AXIS_UNITS[
            self.axis_name
        ][0]

        self.position_label.setText(
            f"{value:.2f} {unit}"
        )

    def set_step(self, step):
        self.value_spin.setSingleStep(step)
        self.speed_spin.setSingleStep(step)

    def update_status(
        self,
        text,
        status_type="normal",
    ):
        self.status_label.setText(text)

        object_name = {
            "normal": "StatusNormal",
            "running": "StatusRunning",
            "alarm": "StatusAlarm",
        }.get(
            status_type,
            "StatusNormal",
        )

        self.status_label.setObjectName(
            object_name
        )

        self.status_label.style().unpolish(
            self.status_label
        )

        self.status_label.style().polish(
            self.status_label
        )


class LogDialog(QDialog):
    def __init__(
        self,
        text,
        parent=None,
    ):
        super().__init__(parent)

        self.setWindowTitle("运行日志")
        self.resize(760, 500)

        editor = QTextEdit()

        editor.setReadOnly(True)
        editor.setPlainText(text)

        close_button = QPushButton(
            "关闭"
        )

        close_button.setObjectName(
            "PrimaryButton"
        )

        close_button.setFixedWidth(110)

        close_button.clicked.connect(
            self.accept
        )

        layout = QVBoxLayout(self)

        layout.setContentsMargins(
            20,
            20,
            20,
            20,
        )

        layout.addWidget(editor)

        layout.addWidget(
            close_button,
            alignment=Qt.AlignRight,
        )


class AppMessageDialog(QDialog):
    def __init__(
        self,
        title,
        message,
        parent=None,
        kind="info",
        buttons=("知道了",),
        primary_index=0,
    ):
        super().__init__(parent)

        self.clicked_index = None

        self.setObjectName("AppDialog")
        self.setWindowTitle(title)
        self.setModal(True)
        self.setMinimumWidth(430)
        self.setWindowFlags(
            Qt.Dialog
            | Qt.FramelessWindowHint
            | Qt.WindowStaysOnTopHint
        )
        self.setAttribute(Qt.WA_TranslucentBackground, True)

        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(16)

        dialog_card = QFrame()
        dialog_card.setObjectName("AppDialogCard")
        card_layout = QVBoxLayout(dialog_card)
        card_layout.setContentsMargins(24, 22, 24, 20)
        card_layout.setSpacing(16)

        title_layout = QHBoxLayout()
        title_layout.setSpacing(12)

        icon_label = QLabel("!")
        icon_label.setObjectName(
            "DialogIconWarning"
            if kind == "warning"
            else "DialogIconInfo"
        )
        icon_label.setAlignment(Qt.AlignCenter)
        icon_label.setFixedSize(32, 32)

        title_label = QLabel(title)
        title_label.setObjectName("DialogTitle")

        title_layout.addWidget(icon_label)
        title_layout.addWidget(title_label, stretch=1)

        message_label = QLabel(message)
        message_label.setObjectName("DialogMessage")
        message_label.setWordWrap(True)
        message_label.setMinimumWidth(360)

        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.setSpacing(10)

        for index, text in enumerate(buttons):
            button = QPushButton(text)
            button.setFixedHeight(36)
            button.setMinimumWidth(96)
            button.setObjectName(
                "PrimaryButton"
                if index == primary_index
                else "SecondaryButton"
            )
            button.clicked.connect(
                lambda checked=False, value=index: self.finish(value)
            )
            button_layout.addWidget(button)

        card_layout.addLayout(title_layout)
        card_layout.addWidget(message_label)
        card_layout.addLayout(button_layout)
        root_layout.addWidget(dialog_card)

    def finish(self, index):
        self.clicked_index = index
        self.accept()


class AppMessageOverlay(QFrame):
    def __init__(
        self,
        title,
        message,
        parent=None,
        kind="info",
        buttons=("知道了",),
        primary_index=0,
    ):
        super().__init__(parent)

        self.clicked_index = None
        self.event_loop = None

        self.setObjectName("MessageOverlay")
        self.setAttribute(Qt.WA_StyledBackground, True)

        if parent is not None:
            self.setGeometry(parent.rect())

        overlay_layout = QVBoxLayout(self)
        overlay_layout.setContentsMargins(0, 0, 0, 0)
        overlay_layout.addStretch()

        center_layout = QHBoxLayout()
        center_layout.addStretch()

        card = QFrame()
        card.setObjectName("AppDialogCard")
        card.setFixedWidth(430)
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(24, 22, 24, 20)
        card_layout.setSpacing(16)

        title_layout = QHBoxLayout()
        title_layout.setSpacing(12)

        icon_label = QLabel("!")
        icon_label.setObjectName(
            "DialogIconWarning"
            if kind == "warning"
            else "DialogIconInfo"
        )
        icon_label.setAlignment(Qt.AlignCenter)
        icon_label.setFixedSize(32, 32)

        title_label = QLabel(title)
        title_label.setObjectName("DialogTitle")

        title_layout.addWidget(icon_label)
        title_layout.addWidget(title_label, stretch=1)

        message_label = QLabel(message)
        message_label.setObjectName("DialogMessage")
        message_label.setWordWrap(True)

        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.setSpacing(10)

        for index, text in enumerate(buttons):
            button = QPushButton(text)
            button.setFixedHeight(36)
            button.setMinimumWidth(96)
            button.setObjectName(
                "PrimaryButton"
                if index == primary_index
                else "SecondaryButton"
            )
            button.clicked.connect(
                lambda checked=False, value=index: self.finish(value)
            )
            button_layout.addWidget(button)

        card_layout.addLayout(title_layout)
        card_layout.addWidget(message_label)
        card_layout.addLayout(button_layout)

        center_layout.addWidget(card)
        center_layout.addStretch()
        overlay_layout.addLayout(center_layout)
        overlay_layout.addStretch()

    def exec_(self):
        self.raise_()
        self.show()
        self.event_loop = QEventLoop(self)
        self.event_loop.exec_()
        return self.clicked_index

    def finish(self, index):
        self.clicked_index = index
        self.hide()
        if self.event_loop is not None:
            self.event_loop.quit()
        self.deleteLater()


class SettingsDialog(QDialog):
    def __init__(
        self,
        settings,
        current_positions,
        parent=None,
    ):
        super().__init__(parent)

        self.setWindowTitle("设置")
        self.setModal(True)
        self.setWindowFlags(
            Qt.Dialog
            | Qt.FramelessWindowHint
        )
        self.setMinimumSize(560, 460)
        screen = QApplication.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            self.resize(
                min(760, max(560, available.width() - 220)),
                min(660, max(460, available.height() - 180)),
            )
        else:
            self.resize(720, 620)
        self.setObjectName("SettingsDialog")

        self.current_positions = dict(current_positions)
        self._drag_position = None

        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(18, 16, 18, 16)
        root_layout.setSpacing(12)

        header_frame = QFrame()
        header_frame.setObjectName("DialogDragHeader")
        header_layout = QHBoxLayout(header_frame)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(8)

        title = QLabel("系统设置")
        title.setObjectName("DialogTitle")
        close_button = QPushButton("×")
        close_button.setObjectName("DialogCloseButton")
        close_button.setFixedSize(32, 32)
        close_button.clicked.connect(self.reject)

        header_layout.addWidget(title)
        header_layout.addStretch()
        header_layout.addWidget(close_button)
        root_layout.addWidget(header_frame)

        self._drag_widgets = {
            self,
            header_frame,
            title,
        }
        for widget in self._drag_widgets:
            widget.installEventFilter(self)

        body_widget = QWidget()
        body_layout = QVBoxLayout(body_widget)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(12)

        scroll_area = QScrollArea()
        scroll_area.setObjectName("SettingsScroll")
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setWidget(body_widget)

        self.ip_edit = QLineEdit(str(settings["plc_ip"]))
        self.port_edit = QLineEdit(str(settings["plc_port"]))
        self.ip_edit.setMaximumWidth(260)
        self.port_edit.setMaximumWidth(160)
        self.default_speed_spin = self.make_number(
            settings["default_speed"],
            1.0,
            999999.0,
            " mm/s",
        )
        self.xyz_step_spin = self.make_number(
            settings["axis_step"]["XYZ"],
            0.01,
            999999.0,
            " mm",
        )
        self.r_step_spin = self.make_number(
            settings["axis_step"]["R"],
            0.01,
            999999.0,
            " °",
        )

        connection_card = QGroupBox("1 连接与默认值")
        connection_layout = QGridLayout(connection_card)
        connection_layout.setContentsMargins(14, 18, 14, 12)
        connection_layout.setHorizontalSpacing(12)
        connection_layout.setVerticalSpacing(10)
        connection_layout.addWidget(QLabel("PLC IP"), 0, 0)
        connection_layout.addWidget(self.ip_edit, 0, 1)
        connection_layout.addWidget(QLabel("端口"), 1, 0)
        connection_layout.addWidget(self.port_edit, 1, 1)
        connection_layout.addWidget(QLabel("默认速度"), 2, 0)
        connection_layout.addWidget(self.default_speed_spin, 2, 1)
        connection_layout.addWidget(QLabel("XYZ调节步长"), 3, 0)
        connection_layout.addWidget(self.xyz_step_spin, 3, 1)
        connection_layout.addWidget(QLabel("R调节步长"), 4, 0)
        connection_layout.addWidget(self.r_step_spin, 4, 1)
        connection_layout.setColumnStretch(1, 1)
        body_layout.addWidget(connection_card)

        self.limit_spins = {}
        limits_card = QGroupBox("2 轴软限位")
        limits_layout = QGridLayout(limits_card)
        limits_layout.setContentsMargins(14, 18, 14, 12)
        limits_layout.setHorizontalSpacing(12)
        limits_layout.setVerticalSpacing(10)
        limits_layout.addWidget(QLabel("轴"), 0, 0)
        limits_layout.addWidget(QLabel("最小值"), 0, 1)
        limits_layout.addWidget(QLabel("最大值"), 0, 2)
        for row, axis_name in enumerate(("X", "Y", "Z", "R"), start=1):
            lower, upper = settings["soft_limits"][axis_name]
            unit = " °" if axis_name == "R" else " mm"
            lower_spin = self.make_number(lower, -999999.0, 999999.0, unit)
            upper_spin = self.make_number(upper, -999999.0, 999999.0, unit)
            self.limit_spins[axis_name] = (lower_spin, upper_spin)
            limits_layout.addWidget(QLabel(axis_name), row, 0)
            limits_layout.addWidget(lower_spin, row, 1)
            limits_layout.addWidget(upper_spin, row, 2)
        body_layout.addWidget(limits_card)

        danger_card = QGroupBox("3 R轴危险区")
        danger_layout = QVBoxLayout(danger_card)
        danger_layout.setContentsMargins(14, 18, 14, 12)
        disabled_note = QLabel("危险区软件限位已关闭；X/Y/Z/R基础软限位仍然生效。")
        disabled_note.setWordWrap(True)
        danger_layout.addWidget(disabled_note)
        danger_card.setVisible(False)
        body_layout.addWidget(danger_card)

        self.origin_spins = {}
        origin_card = QGroupBox("4 工作原点")
        origin_layout = QGridLayout(origin_card)
        origin_layout.setContentsMargins(14, 18, 14, 12)
        origin_layout.setHorizontalSpacing(12)
        origin_layout.setVerticalSpacing(10)
        for row, axis_name in enumerate(("X", "Y", "Z", "R")):
            unit = " °" if axis_name == "R" else " mm"
            spin = self.make_number(
                settings["work_origin"][axis_name],
                -999999.0,
                999999.0,
                unit,
            )
            self.origin_spins[axis_name] = spin
            origin_layout.addWidget(QLabel(f"{axis_name} 原点"), row, 0)
            origin_layout.addWidget(spin, row, 1)

        current_button = QPushButton("使用当前坐标")
        current_button.setObjectName("SecondaryButton")
        current_button.clicked.connect(self.use_current_position)
        origin_layout.addWidget(current_button, 4, 0, 1, 2)
        body_layout.addWidget(origin_card)

        root_layout.addWidget(scroll_area, stretch=1)

        button_layout = QHBoxLayout()
        button_layout.addStretch()
        cancel_button = QPushButton("取消")
        save_button = QPushButton("保存设置")
        cancel_button.setObjectName("SecondaryButton")
        save_button.setObjectName("PrimaryButton")
        cancel_button.clicked.connect(self.reject)
        save_button.clicked.connect(self.accept)
        button_layout.addWidget(cancel_button)
        button_layout.addWidget(save_button)
        root_layout.addLayout(button_layout)

    @staticmethod
    def make_number(value, minimum, maximum, suffix=""):
        spin = SettingsDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(2)
        spin.setSingleStep(1.0)
        spin.setValue(float(value))
        spin.setSuffix(suffix)
        spin.setFixedHeight(36)
        spin.setMaximumWidth(150)
        return spin

    def eventFilter(self, watched, event):
        if watched in getattr(self, "_drag_widgets", set()):
            if (
                event.type() == QEvent.MouseButtonPress
                and event.button() == Qt.LeftButton
            ):
                self._drag_position = (
                    event.globalPos()
                    - self.frameGeometry().topLeft()
                )
                event.accept()
                return True
            if (
                event.type() == QEvent.MouseMove
                and event.buttons() & Qt.LeftButton
                and self._drag_position is not None
            ):
                self.move(
                    event.globalPos()
                    - self._drag_position
                )
                event.accept()
                return True
            if event.type() == QEvent.MouseButtonRelease:
                self._drag_position = None

        return super().eventFilter(watched, event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag_position = (
                event.globalPos()
                - self.frameGeometry().topLeft()
            )
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if (
            event.buttons() & Qt.LeftButton
            and self._drag_position is not None
        ):
            self.move(
                event.globalPos()
                - self._drag_position
            )
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self._drag_position = None
        super().mouseReleaseEvent(event)

    def use_current_position(self):
        for axis_name, spin in self.origin_spins.items():
            spin.setValue(
                float(
                    self.current_positions.get(
                        axis_name,
                        0.0,
                    )
                )
            )

    def get_settings(self):
        return {
            "plc_ip": self.ip_edit.text().strip(),
            "plc_port": self.port_edit.text().strip(),
            "default_speed": float(self.default_speed_spin.value()),
            "axis_step": {
                "XYZ": float(self.xyz_step_spin.value()),
                "R": float(self.r_step_spin.value()),
            },
            "soft_limits": {
                axis_name: [
                    float(spins[0].value()),
                    float(spins[1].value()),
                ]
                for axis_name, spins in self.limit_spins.items()
            },
            "r_forbidden_zones_enabled": False,
            "r_forbidden_zones": [],
            "work_origin": {
                axis_name: float(spin.value())
                for axis_name, spin in self.origin_spins.items()
            },
        }


class ControlCanvas(QWidget):
    DESIGN_WIDTH = 1320
    DESIGN_HEIGHT = 900

    def __init__(self):
        super().__init__()

        self.setFixedSize(
            self.DESIGN_WIDTH,
            self.DESIGN_HEIGHT,
        )

        self.client = None
        self.connected = False
        self.upper_mode = False
        self.estop_active = False

        self.logs = []

        self.axes = {
            axis_name: AxisState(
                axis_name,
                speed=DEFAULT_SETTINGS["axis_default_speeds"].get(axis_name, 30.0),
            )
            for axis_name in (
                "X",
                "Y",
                "Z",
                "R",
            )
        }

        self.modbus_map = {
            "system": {
                "mode_address": MODE_ADDRESS,
                "estop_address": ESTOP_ADDRESS,
            },
            **AXIS_CONFIG,
        }

        self.settings_path = Path(__file__).with_name(
            "gantry_settings.json"
        )
        self.settings = self.load_settings()
        self.soft_limits = {
            axis_name: tuple(values)
            for axis_name, values in self.settings[
                "soft_limits"
            ].items()
        }
        self.r_forbidden_zones = list(
            self.settings["r_forbidden_zones"]
        )
        self.work_origin = dict(
            self.settings["work_origin"]
        )

        self.last_timer_time = time.time()

        self.build_ui()
        self.apply_settings_to_ui()
        self.connect_signals()
        self.apply_style()

        self.timer = QTimer(self)

        self.timer.timeout.connect(
            self.on_timer
        )

        self.timer.start(300)

        self.add_log(
            "软件启动。请连接PLC后操作。"
        )

    def load_settings(self):
        settings = json.loads(
            json.dumps(DEFAULT_SETTINGS)
        )

        if self.settings_path.exists():
            try:
                loaded = json.loads(
                    self.settings_path.read_text(
                        encoding="utf-8"
                    )
                )
                settings.update(
                    {
                        key: value
                        for key, value in loaded.items()
                        if key
                        in (
                            "plc_ip",
                            "plc_port",
                            "default_speed",
                            "axis_default_speeds",
                            "axis_step",
                            "soft_limits",
                            "r_forbidden_zones_enabled",
                            "r_forbidden_zones",
                            "work_origin",
                        )
                    }
                )
            except Exception:
                pass

        return self.normalize_settings(settings)

    @staticmethod
    def normalize_range(values, fallback):
        try:
            lower = float(values[0])
            upper = float(values[1])
        except Exception:
            lower, upper = fallback

        if lower > upper:
            lower, upper = upper, lower

        return [lower, upper]

    def normalize_settings(self, settings):
        defaults = json.loads(
            json.dumps(DEFAULT_SETTINGS)
        )

        result = json.loads(
            json.dumps(DEFAULT_SETTINGS)
        )
        result["plc_ip"] = str(
            settings.get(
                "plc_ip",
                defaults["plc_ip"],
            )
        )
        result["plc_port"] = str(
            settings.get(
                "plc_port",
                defaults["plc_port"],
            )
        )
        try:
            result["default_speed"] = float(
                settings.get(
                    "default_speed",
                    defaults["default_speed"],
                )
            )
        except Exception:
            result["default_speed"] = defaults[
                "default_speed"
            ]

        axis_default_speeds = settings.get("axis_default_speeds", {})
        for axis_name in ("X", "Y", "Z", "R"):
            try:
                result["axis_default_speeds"][axis_name] = max(
                    float(axis_default_speeds.get(
                        axis_name,
                        defaults["axis_default_speeds"][axis_name],
                    )),
                    1.0,
                )
            except Exception:
                result["axis_default_speeds"][axis_name] = defaults["axis_default_speeds"][axis_name]

        axis_step = settings.get(
            "axis_step",
            defaults["axis_step"],
        )
        for key in ("XYZ", "R"):
            try:
                result["axis_step"][key] = max(
                    float(
                        axis_step.get(
                            key,
                            defaults["axis_step"][key],
                        )
                    ),
                    0.01,
                )
            except Exception:
                result["axis_step"][key] = defaults[
                    "axis_step"
                ][key]

        for axis_name in ("X", "Y", "Z", "R"):
            result["soft_limits"][axis_name] = (
                self.normalize_range(
                    settings.get(
                        "soft_limits",
                        {},
                    ).get(
                        axis_name,
                        defaults["soft_limits"][axis_name],
                    ),
                    defaults["soft_limits"][axis_name],
                )
            )

        result["r_forbidden_zones_enabled"] = False
        result["r_forbidden_zones"] = []

        for axis_name in ("X", "Y", "Z", "R"):
            try:
                result["work_origin"][axis_name] = float(
                    settings.get(
                        "work_origin",
                        {},
                    ).get(
                        axis_name,
                        defaults["work_origin"][axis_name],
                    )
                )
            except Exception:
                result["work_origin"][axis_name] = defaults[
                    "work_origin"
                ][axis_name]

        return result

    def build_ui(self):
        root_layout = QVBoxLayout(self)

        root_layout.setContentsMargins(
            22,
            14,
            22,
            14,
        )

        root_layout.setSpacing(9)

        title = QLabel(
            "龙门架控制系统"
        )

        title.setObjectName(
            "MainTitle"
        )

        title.setAlignment(
            Qt.AlignCenter
        )

        title.setFixedHeight(58)

        root_layout.addWidget(title)

        root_layout.addWidget(
            self.build_status_bar()
        )

        root_layout.addWidget(
            self.build_connection_card()
        )

        root_layout.addWidget(
            self.build_navigation()
        )

        self.page_stack = QStackedWidget()

        self.page_stack.setFixedHeight(458)

        self.page_stack.addWidget(
            self.build_single_axis_page()
        )

        self.page_stack.addWidget(
            self.build_coordinate_page()
        )

        self.page_stack.addWidget(
            self.build_full_log_page()
        )

        root_layout.addWidget(
            self.page_stack
        )

        root_layout.addWidget(
            self.build_bottom_bar()
        )

    def build_status_bar(self):
        frame = QFrame()

        frame.setObjectName(
            "StatusFrame"
        )

        frame.setFixedHeight(36)

        layout = QHBoxLayout(frame)

        layout.setContentsMargins(
            14,
            4,
            14,
            4,
        )

        layout.setSpacing(22)

        self.connection_status = QLabel(
            "● 未连接"
        )

        self.connection_status.setObjectName(
            "HeaderOffline"
        )

        self.mode_status = QLabel(
            "本地模式"
        )

        self.mode_status.setObjectName(
            "HeaderText"
        )

        self.estop_status = QLabel(
            "急停正常"
        )

        self.estop_status.setObjectName(
            "HeaderText"
        )

        layout.addStretch()

        layout.addWidget(
            self.connection_status
        )

        layout.addWidget(
            self.mode_status
        )

        layout.addWidget(
            self.estop_status
        )

        layout.addStretch()

        return frame

    def build_connection_card(self):
        card = QGroupBox(
            "连接与安全"
        )

        card.setFixedHeight(120)

        layout = QVBoxLayout(card)

        layout.setContentsMargins(
            18,
            18,
            18,
            14,
        )

        layout.setSpacing(9)

        top_layout = QHBoxLayout()
        top_layout.setSpacing(10)

        self.ip_edit = QLineEdit(
            "192.168.6.6"
        )

        self.port_edit = QLineEdit(
            "502"
        )

        self.settings_button = QPushButton(
            "设置"
        )
        self.settings_button.setObjectName(
            "SecondaryButton"
        )

        self.ip_edit.setFixedHeight(36)
        self.port_edit.setFixedHeight(36)
        self.settings_button.setFixedHeight(36)

        self.ip_edit.setMinimumWidth(350)
        self.port_edit.setFixedWidth(150)
        self.settings_button.setFixedWidth(90)

        top_layout.addWidget(
            QLabel("PLC IP")
        )

        top_layout.addWidget(
            self.ip_edit,
            stretch=1,
        )

        top_layout.addWidget(
            QLabel("端口")
        )

        top_layout.addWidget(
            self.port_edit
        )

        top_layout.addWidget(
            self.settings_button
        )

        bottom_layout = QHBoxLayout()

        bottom_layout.setSpacing(9)

        self.connect_button = QPushButton(
            "连接 PLC"
        )

        self.disconnect_button = QPushButton(
            "断开"
        )

        self.upper_mode_button = QPushButton(
            "切换上位机"
        )

        self.local_mode_button = QPushButton(
            "切回本地"
        )

        self.estop_button = QPushButton(
            "软件急停"
        )

        self.release_estop_button = QPushButton(
            "急停恢复"
        )

        self.reset_all_button = QPushButton(
            "全部复位"
        )

        self.stop_all_button = QPushButton(
            "全部停止"
        )

        self.connect_button.setObjectName(
            "PrimaryButton"
        )

        self.estop_button.setObjectName(
            "EmergencyButton"
        )

        for button in (
            self.disconnect_button,
            self.upper_mode_button,
            self.local_mode_button,
            self.release_estop_button,
            self.reset_all_button,
            self.stop_all_button,
        ):
            button.setObjectName(
                "SecondaryButton"
            )

        for button in (
            self.connect_button,
            self.disconnect_button,
            self.upper_mode_button,
            self.local_mode_button,
            self.estop_button,
            self.release_estop_button,
            self.reset_all_button,
            self.stop_all_button,
        ):
            button.setFixedHeight(40)

            button.setSizePolicy(
                QSizePolicy.Expanding,
                QSizePolicy.Fixed,
            )

            bottom_layout.addWidget(
                button,
                stretch=1,
            )

        layout.addLayout(top_layout)
        layout.addLayout(bottom_layout)

        return card

    def build_navigation(self):
        frame = QFrame()

        frame.setObjectName(
            "NavigationFrame"
        )

        frame.setFixedHeight(54)

        layout = QHBoxLayout(frame)

        layout.setContentsMargins(
            4,
            4,
            4,
            4,
        )

        layout.setSpacing(4)

        self.axis_page_button = QPushButton(
            "单轴控制"
        )

        self.coordinate_page_button = QPushButton(
            "坐标控制"
        )

        self.log_page_button = QPushButton(
            "全部日志"
        )

        self.axis_page_button.setCheckable(True)
        self.coordinate_page_button.setCheckable(True)
        self.log_page_button.setCheckable(True)

        self.axis_page_button.setChecked(True)

        self.axis_page_button.setObjectName(
            "NavigationButton"
        )

        self.coordinate_page_button.setObjectName(
            "NavigationButton"
        )

        self.log_page_button.setObjectName(
            "NavigationButton"
        )

        self.navigation_group = QButtonGroup(self)

        self.navigation_group.setExclusive(True)

        self.navigation_group.addButton(
            self.axis_page_button,
            0,
        )

        self.navigation_group.addButton(
            self.coordinate_page_button,
            1,
        )

        self.navigation_group.addButton(
            self.log_page_button,
            2,
        )

        layout.addWidget(
            self.axis_page_button
        )

        layout.addWidget(
            self.coordinate_page_button
        )

        layout.addWidget(
            self.log_page_button
        )

        return frame

    def build_single_axis_page(self):
        page = QWidget()

        layout = QGridLayout(page)

        layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)

        self.axis_cards = {}

        for index, axis_name in enumerate(
            ("X", "Y", "Z", "R")
        ):
            card = AxisCard(
                axis_name,
                self.settings.get("axis_default_speeds", {}).get(axis_name, 30.0),
            )

            self.axis_cards[
                axis_name
            ] = card

            layout.addWidget(
                card,
                index // 2,
                index % 2,
            )

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

        layout.setRowStretch(0, 1)
        layout.setRowStretch(1, 1)

        return page

    def build_coordinate_page(self):
        page = QWidget()

        root_layout = QVBoxLayout(page)

        root_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        root_layout.setSpacing(12)

        current_card = QGroupBox(
            "当前坐标"
        )

        current_card.setFixedHeight(92)

        current_layout = QHBoxLayout(
            current_card
        )

        current_layout.setContentsMargins(
            34,
            14,
            34,
            12,
        )

        current_layout.setSpacing(24)

        self.current_coordinate_labels = {}

        for axis_name in (
            "X",
            "Y",
            "Z",
            "R",
        ):
            unit = AXIS_UNITS[
                axis_name
            ][0]

            axis_layout = QVBoxLayout()

            axis_layout.setContentsMargins(
                0,
                0,
                0,
                0,
            )

            axis_layout.setSpacing(2)

            name_label = QLabel(
                axis_name
            )

            name_label.setObjectName(
                "CoordinateName"
            )

            name_label.setAlignment(
                Qt.AlignCenter
            )

            value_label = QLabel(
                f"0.00 {unit}"
            )

            value_label.setObjectName(
                "CoordinateValueSmall"
            )

            value_label.setAlignment(
                Qt.AlignCenter
            )

            self.current_coordinate_labels[
                axis_name
            ] = value_label

            axis_layout.addWidget(
                name_label
            )

            axis_layout.addWidget(
                value_label
            )

            current_layout.addLayout(
                axis_layout,
                stretch=1,
            )

        target_card = QGroupBox(
            "目标坐标"
        )

        target_layout = QVBoxLayout(
            target_card
        )

        target_layout.setContentsMargins(
            22,
            20,
            22,
            18,
        )

        target_layout.setSpacing(13)

        hint_label = QLabel(
            "双击输入框后才可编辑；鼠标滚轮不会改变数值。"
        )

        hint_label.setObjectName(
            "TipText"
        )

        target_layout.addWidget(
            hint_label
        )

        input_layout = QHBoxLayout()

        input_layout.setSpacing(20)

        self.coordinate_spins = {}

        for axis_name in (
            "X",
            "Y",
            "Z",
            "R",
        ):
            unit = AXIS_UNITS[
                axis_name
            ][0]

            input_card = QFrame()

            input_card.setObjectName(
                "TargetInputCard"
            )

            card_layout = QVBoxLayout(
                input_card
            )

            card_layout.setContentsMargins(
                14,
                10,
                14,
                12,
            )

            card_layout.setSpacing(7)

            label = QLabel(
                f"{axis_name} 目标"
            )

            label.setObjectName(
                "TargetAxisLabel"
            )

            if axis_name == "R":
                self.r_guard_label = QLabel(
                    "R轴危险区内：R轴不执行"
                )
                self.r_guard_label.setObjectName(
                    "GuardWarning"
                )
                self.r_guard_label.setWordWrap(True)
                self.r_guard_label.hide()

            spin = SafeDoubleSpinBox()

            limits = self.soft_limits.get(axis_name)
            spin.setRange(
                limits[0] if limits else -999999.0,
                limits[1] if limits else 999999.0,
            )

            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.setSuffix(f" {unit}")
            spin.setFixedHeight(42)

            self.coordinate_spins[
                axis_name
            ] = spin

            card_layout.addWidget(label)
            if axis_name == "R":
                card_layout.addWidget(
                    self.r_guard_label
                )
            card_layout.addWidget(spin)

            input_layout.addWidget(
                input_card,
                stretch=1,
            )

        target_layout.addLayout(
            input_layout
        )

        speed_frame = QFrame()

        speed_frame.setObjectName(
            "SpeedInputCard"
        )

        speed_layout = QHBoxLayout(
            speed_frame
        )

        speed_layout.setContentsMargins(
            14,
            9,
            14,
            9,
        )

        speed_layout.setSpacing(12)

        speed_label = QLabel(
            "统一速度"
        )

        speed_label.setObjectName(
            "TargetAxisLabel"
        )

        self.coordinate_speed_spin = (
            SafeDoubleSpinBox()
        )

        self.coordinate_speed_spin.setRange(
            1.0,
            999999.0,
        )

        self.coordinate_speed_spin.setDecimals(2)
        self.coordinate_speed_spin.setValue(30.0)
        self.coordinate_speed_spin.setSingleStep(1.0)

        self.coordinate_speed_spin.setFixedHeight(40)
        self.coordinate_speed_spin.setFixedWidth(260)

        speed_tip = QLabel(
            "默认30；XYZ：mm/s，R轴：°/s"
        )

        speed_tip.setObjectName(
            "TipText"
        )

        speed_layout.addWidget(
            speed_label
        )

        speed_layout.addWidget(
            self.coordinate_speed_spin
        )

        speed_layout.addWidget(
            speed_tip
        )

        speed_layout.addStretch()

        target_layout.addWidget(
            speed_frame
        )

        action_layout = QHBoxLayout()

        action_layout.setSpacing(16)

        self.coordinate_move_button = QPushButton(
            "移动到目标坐标"
        )

        self.coordinate_stop_button = QPushButton(
            "停止全部轴"
        )

        self.save_origin_button = QPushButton(
            "保存当前为工作原点"
        )

        self.work_origin_button = QPushButton(
            "回工作原点"
        )

        self.coordinate_move_button.setObjectName(
            "PrimaryButton"
        )

        self.coordinate_stop_button.setObjectName(
            "SecondaryButton"
        )

        self.save_origin_button.setObjectName(
            "SecondaryButton"
        )

        self.work_origin_button.setObjectName(
            "SecondaryButton"
        )

        self.coordinate_move_button.setFixedHeight(42)
        self.coordinate_stop_button.setFixedHeight(42)
        self.save_origin_button.setFixedHeight(42)
        self.work_origin_button.setFixedHeight(42)

        action_layout.addWidget(
            self.coordinate_move_button,
            stretch=1,
        )

        action_layout.addWidget(
            self.coordinate_stop_button,
            stretch=1,
        )

        action_layout.addWidget(
            self.save_origin_button,
            stretch=1,
        )

        action_layout.addWidget(
            self.work_origin_button,
            stretch=1,
        )

        target_layout.addLayout(
            action_layout
        )

        root_layout.addWidget(
            current_card
        )

        root_layout.addWidget(
            target_card,
            stretch=1,
        )

        return page

    def build_full_log_page(self):
        page = QWidget()

        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        log_card = QGroupBox("全部日志")
        log_layout = QVBoxLayout(log_card)
        log_layout.setContentsMargins(18, 18, 18, 18)
        log_layout.setSpacing(10)

        tip_label = QLabel("显示软件本次启动后的全部运行记录。")
        tip_label.setObjectName("TipText")

        self.full_log_editor = QTextEdit()
        self.full_log_editor.setObjectName("FullLogEditor")
        self.full_log_editor.setReadOnly(True)

        log_layout.addWidget(tip_label)
        log_layout.addWidget(self.full_log_editor, stretch=1)
        layout.addWidget(log_card)

        return page

    def build_bottom_bar(self):
        frame = QFrame()

        frame.setObjectName(
            "BottomBar"
        )

        frame.setFixedHeight(112)

        layout = QVBoxLayout(frame)

        layout.setContentsMargins(
            14,
            8,
            14,
            8,
        )
        layout.setSpacing(6)

        header_layout = QHBoxLayout()

        log_title = QLabel("运行日志")
        log_title.setObjectName("LogTitle")

        self.bottom_status = QLabel(
            ""
        )

        self.bottom_status.setObjectName(
            "BottomText"
        )
        self.bottom_status.hide()

        header_layout.addWidget(log_title)

        self.log_preview = QTextEdit()
        self.log_preview.setObjectName("LogPreview")
        self.log_preview.setReadOnly(True)
        self.log_preview.setFixedHeight(52)
        self.log_preview.setVerticalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff
        )
        self.log_preview.setHorizontalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff
        )

        layout.addLayout(header_layout)
        layout.addWidget(self.log_preview)

        return frame

    def connect_signals(self):
        self.navigation_group.idClicked.connect(
            self.switch_page
        )

        self.settings_button.clicked.connect(
            self.open_settings
        )

        self.connect_button.clicked.connect(
            self.connect_plc
        )

        self.disconnect_button.clicked.connect(
            self.disconnect_plc
        )

        self.upper_mode_button.clicked.connect(
            self.set_upper_mode
        )

        self.local_mode_button.clicked.connect(
            self.set_local_mode
        )

        self.estop_button.clicked.connect(
            self.emergency_stop
        )

        self.release_estop_button.clicked.connect(
            self.release_emergency_stop
        )

        self.reset_all_button.clicked.connect(
            self.reset_all_axes
        )

        self.stop_all_button.clicked.connect(
            self.stop_all_axes
        )

        self.coordinate_move_button.clicked.connect(
            self.run_coordinate_move
        )

        self.coordinate_stop_button.clicked.connect(
            self.stop_all_axes
        )

        self.save_origin_button.clicked.connect(
            self.save_current_as_work_origin
        )

        self.work_origin_button.clicked.connect(
            self.move_to_work_origin
        )

        for card in self.axis_cards.values():
            card.jog_started.connect(
                self.start_jog
            )

            card.jog_stopped.connect(
                self.stop_jog
            )

            card.home_clicked.connect(
                self.home_axis
            )

            card.reset_clicked.connect(
                self.reset_axis
            )

            card.stop_clicked.connect(
                self.stop_axis
            )

            card.absolute_clicked.connect(
                self.run_absolute_position
            )

            card.relative_clicked.connect(
                self.run_relative_position
            )

    def switch_page(self, page_index):
        self.page_stack.setCurrentIndex(
            page_index
        )

    def current_positions_dict(self):
        return {
            axis_name: state.position
            for axis_name, state in self.axes.items()
        }

    def apply_settings_to_ui(self):
        self.ip_edit.setText(self.settings["plc_ip"])
        self.port_edit.setText(self.settings["plc_port"])

        default_speed = float(self.settings["default_speed"])
        axis_default_speeds = self.settings.get("axis_default_speeds", {})
        for axis_name, card in self.axis_cards.items():
            card.speed_spin.setValue(float(axis_default_speeds.get(axis_name, default_speed)))
            card.set_step(
                self.axis_step_for(axis_name)
            )

        self.coordinate_speed_spin.setValue(default_speed)
        self.coordinate_speed_spin.setSingleStep(
            self.axis_step_for("X")
        )

        for axis_name, spin in self.coordinate_spins.items():
            limits = self.soft_limits.get(axis_name)
            if limits is not None:
                spin.setRange(limits[0], limits[1])
            spin.setSingleStep(
                self.axis_step_for(axis_name)
            )

        self.update_r_guard_hint()

    def axis_step_for(self, axis_name):
        if axis_name == "R":
            return float(self.settings["axis_step"]["R"])
        return float(self.settings["axis_step"]["XYZ"])

    def save_settings(self):
        self.settings["soft_limits"] = {
            axis_name: list(values)
            for axis_name, values in self.soft_limits.items()
        }
        self.settings["axis_step"] = {
            "XYZ": float(self.settings["axis_step"]["XYZ"]),
            "R": float(self.settings["axis_step"]["R"]),
        }
        self.settings["axis_default_speeds"] = {
            axis_name: float(self.settings["axis_default_speeds"].get(axis_name, 30.0))
            for axis_name in ("X", "Y", "Z", "R")
        }
        self.settings["r_forbidden_zones"] = (
            self.r_forbidden_zones
        )
        self.settings["work_origin"] = self.work_origin
        self.settings_path.write_text(
            json.dumps(
                self.settings,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

    def open_settings(self):
        dialog = SettingsDialog(
            self.settings,
            self.current_positions_dict(),
            self,
        )

        if dialog.exec_() != QDialog.Accepted:
            return

        self.settings = self.normalize_settings(
            dialog.get_settings()
        )
        self.settings["r_forbidden_zones_enabled"] = False
        self.soft_limits = {
            axis_name: tuple(values)
            for axis_name, values in self.settings[
                "soft_limits"
            ].items()
        }
        self.r_forbidden_zones = list(
            self.settings["r_forbidden_zones"]
        )
        self.work_origin = dict(
            self.settings["work_origin"]
        )
        self.save_settings()
        self.apply_settings_to_ui()
        self.add_log("设置已保存。")

    def save_current_as_work_origin(self):
        self.work_origin = self.current_positions_dict()
        self.settings["work_origin"] = dict(
            self.work_origin
        )
        self.save_settings()
        self.add_log("已保存当前坐标为工作原点。")

    def move_to_work_origin(self):
        if not self.can_move():
            return

        self.run_targets(
            dict(self.work_origin),
            float(self.coordinate_speed_spin.value()),
            full_log="执行回工作原点。",
            skip_r_log="执行回工作原点；R轴因危险区未执行。",
        )

    def add_log(self, text):
        current_time = datetime.now().strftime(
            "%H:%M:%S"
        )

        line = f"[{current_time}] {text}"

        self.logs.append(line)

        self.bottom_status.setText(text)
        if hasattr(self, "log_preview"):
            self.log_preview.setPlainText(
                "\n".join(self.logs[-2:])
            )
        if hasattr(self, "full_log_editor"):
            self.full_log_editor.append(line)
            scrollbar = self.full_log_editor.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

    def show_logs(self):
        dialog = LogDialog(
            "\n".join(self.logs),
            self,
        )

        dialog.exec_()

    def show_message(
        self,
        title,
        message,
        kind="info",
    ):
        dialog = AppMessageOverlay(
            title,
            message,
            self,
            kind=kind,
            buttons=("知道了",),
            primary_index=0,
        )
        dialog.exec_()

    def ask_confirm(
        self,
        title,
        message,
        confirm_text="确认",
        cancel_text="取消",
    ):
        dialog = AppMessageOverlay(
            title,
            message,
            self,
            kind="warning",
            buttons=(cancel_text, confirm_text),
            primary_index=1,
        )
        dialog.exec_()
        return dialog.clicked_index == 1

    def axis_limit_text(self, axis_name):
        limits = self.soft_limits.get(axis_name)
        if limits is None:
            return ""
        lower, upper = limits
        unit = AXIS_UNITS[axis_name][0]
        return f"{axis_name}轴允许范围：{lower:.0f}-{upper:.0f} {unit}"

    def validate_axis_target(
        self,
        axis_name,
        target,
    ):
        limits = self.soft_limits.get(axis_name)
        if limits is None:
            return True

        lower, upper = limits
        if lower <= target <= upper:
            return True

        self.axis_cards[axis_name].update_status(
            "超出软限位",
            "alarm",
        )
        self.add_log(
            f"{axis_name}轴目标超出软限位，已禁止执行。"
        )
        self.show_message(
            "目标超出软限位",
            f"当前输入目标为 {target:.2f} {AXIS_UNITS[axis_name][0]}。\n"
            f"{self.axis_limit_text(axis_name)}。\n\n"
            "请修改目标后再执行。",
            kind="warning",
        )
        return False

    def validate_jog_limit(
        self,
        axis_name,
        direction,
    ):
        limits = self.soft_limits.get(axis_name)
        if limits is None:
            return True

        lower, upper = limits
        position = self.axes[axis_name].position

        if direction < 0 and position <= lower:
            blocked_side = "负向"
        elif direction > 0 and position >= upper:
            blocked_side = "正向"
        else:
            return True

        self.axis_cards[axis_name].update_status(
            "到达软限位",
            "alarm",
        )
        self.add_log(
            f"{axis_name}轴已到达{blocked_side}软限位，点动被禁止。"
        )
        self.show_message(
            "已到达软限位",
            f"{axis_name}轴当前位置 {position:.2f} {AXIS_UNITS[axis_name][0]}。\n"
            f"{self.axis_limit_text(axis_name)}。\n\n"
            f"已禁止继续{blocked_side}点动。",
            kind="warning",
        )
        return False

    @staticmethod
    def value_in_range(value, range_values):
        if range_values is None:
            return True
        lower, upper = range_values
        return lower <= value <= upper

    def r_forbidden_zone_at(self, point):
        # The old installation-specific R danger zones are intentionally
        # disabled.  Axis soft limits remain active through self.soft_limits.
        return None

    def current_r_forbidden_zone(self):
        return self.r_forbidden_zone_at(
            {
                "X": self.axes["X"].position,
                "Y": self.axes["Y"].position,
                "Z": self.axes["Z"].position,
            }
        )

    def target_r_forbidden_zone(self, targets):
        return self.r_forbidden_zone_at(
            {
                "X": targets.get(
                    "X",
                    self.axes["X"].position,
                ),
                "Y": targets.get(
                    "Y",
                    self.axes["Y"].position,
                ),
                "Z": targets.get(
                    "Z",
                    self.axes["Z"].position,
                ),
            }
        )

    def is_r_rotation_forbidden(self):
        return self.current_r_forbidden_zone() is not None

    def update_r_guard_hint(self):
        forbidden_zone = self.current_r_forbidden_zone()
        forbidden = forbidden_zone is not None

        if hasattr(self, "r_guard_label"):
            if forbidden_zone is not None:
                self.r_guard_label.setText(
                    f"{forbidden_zone['name']}：R轴不执行"
                )
            self.r_guard_label.setVisible(forbidden)

        if "R" not in self.axis_cards:
            return

        r_state = self.axes["R"]
        current_text = self.axis_cards[
            "R"
        ].status_label.text()

        if (
            forbidden
            and not r_state.positioning
            and r_state.jog_direction == 0
            and not r_state.alarm
        ):
            self.axis_cards["R"].update_status(
                "低位禁转",
                "alarm",
            )
        elif (
            not forbidden
            and current_text
            in ("低位禁转", "禁止旋转")
            and not r_state.positioning
            and r_state.jog_direction == 0
        ):
            self.axis_cards["R"].update_status(
                "待机",
                "normal",
            )

    def show_r_rotation_warning(
        self,
        log_text=None,
        zone=None,
    ):
        if zone is None:
            zone = self.current_r_forbidden_zone()
        zone_name = zone["name"] if zone else "危险区"

        if hasattr(self, "r_guard_label"):
            self.r_guard_label.setText(
                f"{zone_name}：R轴不执行"
            )
            self.r_guard_label.show()

        self.axis_cards["R"].update_status(
            "低位禁转",
            "alarm",
        )

        self.bottom_status.setText(
            f"{zone_name}内，R轴不执行。"
        )

        if log_text:
            self.add_log(log_text)

    def validate_r_rotation_allowed(
        self,
        target_z=None,
    ):
        if target_z is None:
            zone = self.current_r_forbidden_zone()
        else:
            zone = self.r_forbidden_zone_at(
                {
                    "X": self.axes["X"].position,
                    "Y": self.axes["Y"].position,
                    "Z": target_z,
                }
            )

        if zone is not None:
            self.show_r_rotation_warning(
                f"{zone['name']}内，R轴动作未执行。"
            )
            return False

        self.update_r_guard_hint()
        return True

    def is_mock_mode(self):
        return False

    def connect_plc(self):
        if ModbusTcpClient is None:
            self.show_message(
                "缺少依赖",
                "请先安装 pymodbus：\n"
                "pip install pymodbus",
                kind="warning",
            )

            return

        try:
            port = int(
                self.port_edit.text().strip()
            )
        except ValueError:
            self.show_message(
                "端口错误",
                "端口必须是数字。",
                kind="warning",
            )

            return

        try:
            self.client = ModbusTcpClient(
                self.ip_edit.text().strip(),
                port=port,
            )

            result = self.client.connect()

        except Exception as exception:
            result = False

            self.add_log(
                f"连接异常：{exception}"
            )

        self.connected = bool(result)

        if self.connected:
            self.connection_status.setText(
                "● PLC在线"
            )

            self.connection_status.setObjectName(
                "HeaderOnline"
            )

            self.add_log(
                "PLC连接成功。"
            )
            self.refresh_system_status()

        else:
            self.connection_status.setText(
                "● 连接失败"
            )

            self.connection_status.setObjectName(
                "HeaderOffline"
            )

            self.add_log(
                "PLC连接失败。"
            )

        self.refresh_style(
            self.connection_status
        )

    def disconnect_plc(self):
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass

        self.client = None
        self.connected = False

        self.connection_status.setText(
            "● 未连接"
        )

        self.connection_status.setObjectName(
            "HeaderOffline"
        )

        self.refresh_style(
            self.connection_status
        )

        self.add_log(
            "PLC已断开。"
        )

    def set_upper_mode(self):
        if not self.connected:
            self.show_message(
                "未连接",
                "请先连接PLC。",
                kind="warning",
            )
            return

        self.upper_mode = True

        self.mode_status.setText(
            "上位机模式"
        )

        self.write_register(
            self.modbus_map[
                "system"
            ]["mode_address"],
            2,
        )

        self.add_log(
            "已切换到上位机模式。"
        )

    def set_local_mode(self):
        if not self.connected:
            self.show_message(
                "未连接",
                "请先连接PLC。",
                kind="warning",
            )
            return

        self.stop_all_axes()

        self.upper_mode = False

        self.mode_status.setText(
            "本地模式"
        )

        self.write_register(
            self.modbus_map[
                "system"
            ]["mode_address"],
            0,
        )

        self.add_log(
            "已切换到本地模式。"
        )

    def emergency_stop(self):
        self.estop_active = True

        self.stop_all_axes()

        self.estop_status.setText(
            "急停已触发"
        )

        self.estop_status.setObjectName(
            "HeaderAlarm"
        )

        self.refresh_style(
            self.estop_status
        )

        self.write_register(
            self.modbus_map[
                "system"
            ]["estop_address"],
            1,
        )

        self.add_log(
            "已触发上位机急停。"
        )

    def release_emergency_stop(self):
        if not self.ask_confirm(
            "急停恢复确认",
            "请确认四轴已经完全停止，实体急停可立即使用。\n\n"
            "确认后将解除上位机软件急停。",
            confirm_text="确认恢复",
            cancel_text="取消",
        ):
            self.add_log("急停恢复已取消，PLC未写入。")
            return

        self.estop_active = False

        self.estop_status.setText(
            "急停正常"
        )

        self.estop_status.setObjectName(
            "HeaderText"
        )

        self.refresh_style(
            self.estop_status
        )

        self.write_register(
            self.modbus_map[
                "system"
            ]["estop_address"],
            0,
        )

        self.add_log(
            "已恢复上位机软件急停。"
        )
        self.refresh_system_status()

    def can_move(self):
        if self.estop_active:
            self.show_message(
                "禁止运动",
                "当前处于急停状态。",
                kind="warning",
            )

            return False

        if not self.upper_mode:
            self.show_message(
                "提示",
                "请先切换到上位机模式。",
            )

            return False

        if not self.connected:
            self.show_message(
                "未连接",
                "请先连接PLC。",
                kind="warning",
            )

            return False

        return True

    def start_jog(
        self,
        axis_name,
        direction,
    ):
        if not self.can_move():
            return

        if not self.validate_jog_limit(
            axis_name,
            direction,
        ):
            return

        if (
            axis_name == "R"
            and not self.validate_r_rotation_allowed()
        ):
            return

        state = self.axes[axis_name]

        state.speed = float(
            self.axis_cards[
                axis_name
            ].speed_spin.value()
        )

        speed_address = self.modbus_map.get(
            axis_name,
            {},
        ).get("absolute_speed_address")
        jog_speed_address = {
            "X": 41188,
            "Y": 41288,
            "Z": 41388,
            "R": 41488,
        }.get(axis_name)
        if jog_speed_address is not None:
            self.write_dint(
                jog_speed_address,
                int(round(state.speed)),
            )

        state.jog_direction = direction
        state.positioning = False

        address = self.modbus_map.get(
            axis_name,
            {},
        ).get(
            "jog_address"
        )

        if address is not None:
            command = (
                10
                if direction > 0
                else 20
            )

            self.write_register(
                address,
                command,
            )

        self.axis_cards[
            axis_name
        ].update_status(
            "点动中",
            "running",
        )

        self.add_log(
            f"{axis_name}轴开始点动。"
        )

    def stop_jog(self, axis_name):
        state = self.axes[axis_name]

        state.jog_direction = 0

        address = self.modbus_map.get(
            axis_name,
            {},
        ).get(
            "jog_address"
        )

        if address is not None:
            self.write_register(
                address,
                0,
            )

        if not state.positioning:
            self.axis_cards[
                axis_name
            ].update_status(
                "待机",
                "normal",
            )

        self.add_log(
            f"{axis_name}轴点动停止。"
        )

    def home_axis(self, axis_name):
        if not self.can_move():
            return

        state = self.axes[axis_name]

        address = self.modbus_map.get(
            axis_name,
            {},
        ).get(
            "home_address"
        )

        if address is not None:
            self.pulse_register(
                address,
                10,
            )

        state.target = 0.0
        state.start_position = state.position
        state.motion_start_time = time.time()
        state.jog_direction = 0
        state.positioning = True
        state.motion_kind = "home"

        self.axis_cards[
            axis_name
        ].update_status(
            "回原中｜按软件急停",
            "running",
        )

        self.add_log(
            f"{axis_name}轴执行回原；如需暂停请按“软件急停”。"
        )

    def reset_axis(self, axis_name):
        address = self.modbus_map.get(
            axis_name,
            {},
        ).get("reset_address")
        if address is not None:
            self.pulse_register(address, 10)

        self.axes[
            axis_name
        ].alarm = False

        self.axis_cards[
            axis_name
        ].update_status(
            "复位完成",
            "normal",
        )

        self.add_log(
            f"{axis_name}轴复位完成。"
        )

    def stop_axis(self, axis_name):
        state = self.axes[axis_name]

        state.jog_direction = 0
        state.positioning = False
        state.motion_kind = ""

        address = self.modbus_map.get(
            axis_name,
            {},
        ).get(
            "jog_address"
        )

        if address is not None:
            self.write_register(
                address,
                0,
            )

        for key in MOTION_CLEAR_KEYS:
            clear_address = self.modbus_map.get(axis_name, {}).get(key)
            if clear_address is not None:
                self.write_register(clear_address, 0)

        self.axis_cards[
            axis_name
        ].update_status(
            "已停止",
            "normal",
        )

    def reset_all_axes(self):
        for axis_name in self.axes:
            self.reset_axis(axis_name)

        self.add_log(
            "全部轴复位完成。"
        )

    def stop_all_axes(self):
        for axis_name in self.axes:
            self.stop_axis(axis_name)

        self.add_log(
            "全部轴已停止。"
        )

    def run_absolute_position(
        self,
        axis_name,
        target,
        speed,
    ):
        if not self.can_move():
            return

        if (
            axis_name == "R"
            and not self.validate_r_rotation_allowed()
        ):
            return

        if not self.validate_axis_target(
            axis_name,
            target,
        ):
            return

        self.command_absolute(
            axis_name,
            target,
            speed,
            motion_kind="absolute",
        )

        self.add_log(
            f"{axis_name}轴绝对定位："
            f"目标={target:.2f}。"
        )

    def run_relative_position(
        self,
        axis_name,
        distance,
        speed,
    ):
        if not self.can_move():
            return

        target = (
            self.axes[
                axis_name
            ].position
            + distance
        )

        if (
            axis_name == "R"
            and not self.validate_r_rotation_allowed()
        ):
            return

        if not self.validate_axis_target(
            axis_name,
            target,
        ):
            return

        self.command_absolute(
            axis_name,
            target,
            speed,
            motion_kind="relative",
        )

        self.add_log(
            f"{axis_name}轴相对定位："
            f"位移={distance:.2f}。"
        )

    def run_targets(
        self,
        targets,
        speed,
        full_log,
        skip_r_log,
    ):
        for axis_name, target in targets.items():
            if not self.validate_axis_target(
                axis_name,
                target,
            ):
                return

        active_axes = ["X", "Y", "Z", "R"]
        r_will_move = abs(
            targets["R"] - self.axes["R"].position
        ) > 0.5

        current_zone = self.current_r_forbidden_zone()
        target_zone = self.target_r_forbidden_zone(targets)

        if r_will_move and (
            current_zone is not None
            or target_zone is not None
        ):
            active_axes = ["X", "Y", "Z"]
            self.show_r_rotation_warning(
                zone=current_zone or target_zone,
            )

        for axis_name in active_axes:
            target = targets[axis_name]
            config = self.modbus_map.get(axis_name, {})
            self.write_dint(
                config["absolute_position_address"],
                int(round(target)),
            )
            self.write_dint(
                config["absolute_speed_address"],
                int(round(speed)),
            )
            self.axes[axis_name].target = target
            self.axes[axis_name].speed = speed
            self.axes[axis_name].start_position = self.axes[
                axis_name
            ].position
            self.axes[axis_name].motion_start_time = time.time()
            self.axes[axis_name].positioning = True
            self.axes[axis_name].motion_kind = "absolute"

        for axis_name in active_axes:
            trigger_address = self.modbus_map[axis_name][
                "absolute_trigger_address"
            ]
            self.write_register(trigger_address, 10)

        time.sleep(0.08)

        for axis_name in active_axes:
            trigger_address = self.modbus_map[axis_name][
                "absolute_trigger_address"
            ]
            self.write_register(trigger_address, 0)
            self.axis_cards[axis_name].update_status(
                "定位中",
                "running",
            )

        if "R" in active_axes:
            self.add_log(full_log)
        else:
            self.add_log(skip_r_log)

    def run_coordinate_move(self):
        if not self.can_move():
            return

        speed = float(
            self.coordinate_speed_spin.value()
        )

        targets = {}

        for axis_name in (
            "X",
            "Y",
            "Z",
            "R",
        ):
            target = float(
                self.coordinate_spins[
                    axis_name
                ].value()
            )

            targets[axis_name] = target

        self.run_targets(
            targets,
            speed,
            full_log="执行四轴近同时目标坐标运动。",
            skip_r_log="执行XYZ目标坐标运动；R轴因危险区未执行。",
        )

    def command_absolute(
        self,
        axis_name,
        target,
        speed,
        motion_kind="absolute",
    ):
        state = self.axes[axis_name]

        state.target = target
        state.speed = max(
            speed,
            0.1,
        )

        state.jog_direction = 0
        state.positioning = True
        state.start_position = state.position
        state.motion_start_time = time.time()
        state.motion_kind = motion_kind

        config = self.modbus_map.get(
            axis_name,
            {},
        )

        position_address = config.get(
            "absolute_position_address"
        )

        speed_address = config.get(
            "absolute_speed_address"
        )

        trigger_address = config.get(
            "absolute_trigger_address"
        )

        if position_address is not None:
            self.write_dint(
                position_address,
                int(round(target)),
            )

        if speed_address is not None:
            self.write_dint(
                speed_address,
                int(round(speed)),
            )

        if trigger_address is not None:
            self.pulse_register(
                trigger_address,
                10,
            )

        self.axis_cards[
            axis_name
        ].update_status(
            "定位中",
            "running",
        )

    def write_register(
        self,
        address,
        value,
    ):
        if self.is_mock_mode():
            return True

        if (
            not self.connected
            or self.client is None
        ):
            return False

        try:
            try:
                result = self.client.write_register(
                    address,
                    value,
                    device_id=DEVICE_ID,
                )
            except TypeError as exception:
                if "device_id" not in str(exception):
                    raise
                result = self.client.write_register(
                    address,
                    value,
                    slave=DEVICE_ID,
                )

            return not (
                hasattr(result, "isError")
                and result.isError()
            )

        except Exception:
            return False

    def write_dint(
        self,
        address,
        value,
    ):
        value = int(value)

        if value < 0:
            value = (
                (1 << 32)
                + value
            )

        low_word = value & 0xFFFF

        high_word = (
            value >> 16
        ) & 0xFFFF

        if self.is_mock_mode():
            return True

        if (
            not self.connected
            or self.client is None
        ):
            return False

        try:
            try:
                result = self.client.write_registers(
                    address,
                    [
                        low_word,
                        high_word,
                    ],
                    device_id=DEVICE_ID,
                )
            except TypeError as exception:
                if "device_id" not in str(exception):
                    raise
                result = self.client.write_registers(
                    address,
                    [
                        low_word,
                        high_word,
                    ],
                    slave=DEVICE_ID,
                )

            return not (
                hasattr(result, "isError")
                and result.isError()
            )

        except Exception:
            return False

    def read_register(self, address, count=1):
        if not self.connected or self.client is None:
            return None

        try:
            try:
                result = self.client.read_holding_registers(
                    address,
                    count=count,
                    device_id=DEVICE_ID,
                )
            except TypeError as exception:
                if "device_id" not in str(exception):
                    raise
                result = self.client.read_holding_registers(
                    address,
                    count=count,
                    slave=DEVICE_ID,
                )
        except Exception:
            return None

        if hasattr(result, "isError") and result.isError():
            return None

        return list(getattr(result, "registers", []))

    def read_dint(self, address):
        registers = self.read_register(address, 2)
        if not registers or len(registers) < 2:
            return None

        raw = (int(registers[1]) << 16) | int(registers[0])
        if raw & 0x80000000:
            raw -= 0x100000000
        return raw

    def pulse_register(
        self,
        address,
        value=10,
    ):
        first_result = self.write_register(
            address,
            value,
        )

        time.sleep(0.08)

        second_result = self.write_register(
            address,
            0,
        )

        return (
            first_result
            and second_result
        )

    def on_timer(self):
        current_time = time.time()

        delta_time = (
            current_time
            - self.last_timer_time
        )

        self.last_timer_time = current_time

        if self.connected:
            self.refresh_system_status()
        else:
            self.update_mock_motion(
                delta_time
            )

        self.refresh_positions()

    def refresh_system_status(self):
        system = self.read_register(MODE_ADDRESS, 3)
        if system and len(system) >= 3:
            mode = system[0]
            estop = system[2]
            self.upper_mode = mode == 2
            self.estop_active = estop != 0
            self.mode_status.setText(
                "上位机模式" if mode == 2 else "本地模式"
            )
            self.estop_status.setText(
                "急停正常" if estop == 0 else "急停已触发"
            )
            self.estop_status.setObjectName(
                "HeaderText" if estop == 0 else "HeaderAlarm"
            )
            self.refresh_style(self.estop_status)

        for axis_name, config in self.modbus_map.items():
            if axis_name == "system":
                continue
            state = self.axes[axis_name]
            position = self.read_dint(config["position_address"])
            if position is not None:
                state.position = float(position)
            alarm_values = self.read_register(config["alarm_address"], 5)
            if alarm_values and len(alarm_values) >= 5:
                state.alarm = any(
                    value != 0 for value in (
                        alarm_values[0],
                        alarm_values[2],
                        alarm_values[4],
                    )
                )

            if state.positioning and not state.alarm:
                actual_speed = self.read_dint(config["actual_speed_address"])
                if state.motion_kind == "home":
                    done_values = self.read_register(
                        config["home_done_address"],
                        1,
                    )
                    home_done = bool(done_values and done_values[0] == 1)
                    near_home = abs(state.position) <= 1.0
                    elapsed = time.time() - state.motion_start_time
                    if home_done or (near_home and elapsed >= 1.5):
                        state.positioning = False
                        state.motion_kind = ""
                        state.target = state.position
                        self.axis_cards[axis_name].update_status(
                            "回原完成",
                            "normal",
                        )
                        self.add_log(f"{axis_name}轴回原完成。")
                else:
                    done_values = self.read_register(
                        config["absolute_done_address"],
                        1,
                    )
                    near_target = abs(state.position - state.target) <= 1.0
                    done = bool(done_values and done_values[0] == 1)
                    speed_stopped = actual_speed in (None, 0)
                    if near_target and (done or speed_stopped):
                        kind = state.motion_kind
                        state.positioning = False
                        state.motion_kind = ""
                        text = "相对完成" if kind == "relative" else "定位完成"
                        self.axis_cards[axis_name].update_status(
                            text,
                            "normal",
                        )
                        self.add_log(
                            f"{axis_name}轴{text}，当前位置{state.position:.2f}。"
                        )

    def update_mock_motion(
        self,
        delta_time,
    ):
        for axis_name, state in (
            self.axes.items()
        ):
            if state.jog_direction != 0:
                state.position += (
                    state.jog_direction
                    * state.speed
                    * delta_time
                )

            if state.positioning:
                difference = (
                    state.target
                    - state.position
                )

                step = (
                    max(
                        state.speed,
                        0.1,
                    )
                    * delta_time
                )

                if abs(difference) <= step:
                    state.position = (
                        state.target
                    )

                    state.positioning = False

                    self.axis_cards[
                        axis_name
                    ].update_status(
                        "已到位",
                        "normal",
                    )

                else:
                    state.position += (
                        step
                        if difference > 0
                        else -step
                    )

    def refresh_positions(self):
        self.update_r_guard_hint()

        for axis_name, state in (
            self.axes.items()
        ):
            self.axis_cards[
                axis_name
            ].update_position(
                state.position
            )

            unit = AXIS_UNITS[
                axis_name
            ][0]

            self.current_coordinate_labels[
                axis_name
            ].setText(
                f"{state.position:.2f} {unit}"
            )

            if state.alarm:
                self.axis_cards[
                    axis_name
                ].update_status(
                    "报警",
                    "alarm",
                )

            elif state.jog_direction != 0:
                self.axis_cards[
                    axis_name
                ].update_status(
                    "点动中",
                    "running",
                )

            elif state.positioning:
                if state.motion_kind == "home":
                    status_text = "回原中｜按软件急停"
                else:
                    status_text = "运行中"

                self.axis_cards[
                    axis_name
                ].update_status(
                    status_text,
                    "running",
                )

            elif axis_name == "R":
                self.update_r_guard_hint()

    @staticmethod
    def refresh_style(widget):
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def shutdown(self):
        try:
            self.stop_all_axes()

            if self.client is not None:
                self.client.close()

        except Exception:
            pass

    def apply_style(self):
        self.setStyleSheet("""
            QWidget {
                background: #F5F5F7;
                color: #1D1D1F;
                font-family:
                    "Microsoft YaHei",
                    "PingFang SC",
                    "Segoe UI";
                font-size: 14px;
            }

            QLabel {
                background: transparent;
            }

            QLabel#MainTitle {
                font-size: 36px;
                font-weight: 700;
            }

            QLabel#AxisPosition {
                font-size: 18px;
                font-weight: 700;
            }

            QLabel#CoordinateName {
                color: #86868B;
                font-size: 14px;
                font-weight: 600;
            }

            QLabel#CoordinateValueSmall {
                color: #1D1D1F;
                font-size: 18px;
                font-weight: 700;
            }

            QLabel#TargetAxisLabel {
                color: #1D1D1F;
                font-size: 14px;
                font-weight: 600;
            }

            QLabel#GuardWarning {
                color: #D70015;
                background: #FFECEE;
                border-radius: 7px;
                padding: 4px 7px;
                font-size: 12px;
                font-weight: 700;
            }

            QLabel#TipText {
                color: #6E6E73;
                font-size: 13px;
            }

            QLabel#StatusNormal {
                color: #248A3D;
                background: #EAF7ED;
                border-radius: 10px;
                padding: 4px 8px;
                font-weight: 700;
            }

            QLabel#StatusRunning {
                color: #B25000;
                background: #FFF4E5;
                border-radius: 10px;
                padding: 4px 8px;
                font-weight: 700;
            }

            QLabel#StatusAlarm {
                color: #D70015;
                background: #FFECEE;
                border-radius: 10px;
                padding: 4px 8px;
                font-weight: 700;
            }

            QGroupBox {
                background: #FFFFFF;
                border: 1px solid #E1E1E6;
                border-radius: 14px;
                margin-top: 12px;
                padding-top: 5px;
                font-size: 15px;
                font-weight: 700;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 15px;
                padding: 0 7px;
                background: #F5F5F7;
            }

            QFrame#TargetInputCard,
            QFrame#SpeedInputCard {
                background: #FBFBFD;
                border: 1px solid #E1E1E6;
                border-radius: 10px;
            }

            QLineEdit,
            QDoubleSpinBox,
            QTextEdit {
                background: #FFFFFF;
                color: #1D1D1F;
                border: 1px solid #D1D1D6;
                border-radius: 8px;
                padding: 6px 36px 6px 9px;
                font-size: 14px;
                selection-background-color: #007AFF;
            }

            QDoubleSpinBox[editingUnlocked="false"] {
                background: #F8F8FA;
                color: #6E6E73;
            }

            QDoubleSpinBox[editingUnlocked="true"] {
                background: #FFFFFF;
                color: #1D1D1F;
                border: 2px solid #007AFF;
            }

            QToolButton#SpinArrowButton {
                background: transparent;
                border: none;
                border-radius: 4px;
                padding: 0;
                margin: 0;
            }

            QToolButton#SpinArrowButton:hover {
                background: #E8E8ED;
            }

            QPushButton {
                border-radius: 8px;
                padding: 6px 9px;
                font-size: 14px;
                font-weight: 600;
            }

            QPushButton#PrimaryButton {
                background: #007AFF;
                color: #FFFFFF;
                border: none;
            }

            QPushButton#PrimaryButton:hover {
                background: #0A84FF;
            }

            QPushButton#SecondaryButton,
            QPushButton#AxisButton {
                background: #F2F2F7;
                color: #1D1D1F;
                border: 1px solid #D1D1D6;
            }

            QPushButton#SecondaryButton:hover,
            QPushButton#AxisButton:hover {
                background: #E8E8ED;
            }

            QPushButton#EmergencyButton {
                background: #FF3B30;
                color: #FFFFFF;
                border: none;
                font-weight: 700;
            }

            QPushButton#BlankAxisButton {
                background: transparent;
                border: none;
            }

            QFrame#NavigationFrame {
                background: #E9E9EE;
                border: 1px solid #DADAE0;
                border-radius: 12px;
            }

            QPushButton#NavigationButton {
                background: transparent;
                color: #3A3A3C;
                border: none;
                border-radius: 10px;
                font-size: 18px;
                font-weight: 700;
            }

            QPushButton#NavigationButton:checked {
                background: #FFFFFF;
                color: #007AFF;
                border: 1px solid #C7D7FF;
            }

            QFrame#StatusFrame,
            QFrame#BottomBar {
                background: #FFFFFF;
                border: 1px solid #E1E1E6;
                border-radius: 10px;
            }

            QLabel#HeaderOnline {
                color: #248A3D;
                font-weight: 700;
            }

            QLabel#HeaderOffline {
                color: #86868B;
                font-weight: 700;
            }

            QLabel#HeaderText {
                color: #6E6E73;
            }

            QLabel#HeaderAlarm {
                color: #D70015;
                font-weight: 700;
            }

            QLabel#BottomText {
                color: #6E6E73;
                font-size: 13px;
            }

            QLabel#LogTitle {
                color: #1D1D1F;
                font-size: 15px;
                font-weight: 700;
            }

            QTextEdit#LogPreview {
                background: #FBFBFD;
                color: #3A3A3C;
                border: 1px solid #E5E5EA;
                border-radius: 8px;
                padding: 6px 8px;
                font-size: 13px;
            }

            QTextEdit#FullLogEditor {
                background: #FBFBFD;
                color: #1D1D1F;
                border: 1px solid #E5E5EA;
                border-radius: 10px;
                padding: 10px 12px;
                font-size: 14px;
            }

            QDialog#AppDialog {
                background: transparent;
            }

            QFrame#MessageOverlay {
                background: rgba(245, 245, 247, 135);
            }

            QFrame#AppDialogCard {
                background: #FFFFFF;
                border: 1px solid #E5E5EA;
                border-radius: 14px;
            }

            QDialog#SettingsDialog {
                background: #F5F5F7;
            }

            QScrollArea#SettingsScroll {
                background: transparent;
                border: none;
            }

            QScrollArea#SettingsScroll > QWidget > QWidget {
                background: transparent;
            }

            QLabel#DialogTitle {
                color: #1D1D1F;
                font-size: 18px;
                font-weight: 700;
            }

            QLabel#DialogMessage {
                color: #3A3A3C;
                font-size: 14px;
                line-height: 1.4;
            }

            QLabel#DialogIconInfo {
                color: #FFFFFF;
                background: #007AFF;
                border-radius: 16px;
                font-size: 18px;
                font-weight: 700;
            }

            QLabel#DialogIconWarning {
                color: #FFFFFF;
                background: #FF9500;
                border-radius: 16px;
                font-size: 18px;
                font-weight: 700;
            }

            QPushButton#DialogCloseButton {
                background: #F2F2F7;
                color: #3A3A3C;
                border: 1px solid #D1D1D6;
                border-radius: 16px;
                font-size: 18px;
                font-weight: 700;
                padding: 0;
            }

            QPushButton#DialogCloseButton:hover {
                background: #E5E5EA;
                color: #D70015;
            }

            QPushButton#TextButton {
                background: transparent;
                color: #007AFF;
                border: none;
                padding: 3px 7px;
            }
        """)


class ScaledGraphicsView(QGraphicsView):
    def __init__(
        self,
        canvas,
        parent=None,
    ):
        super().__init__(parent)

        self.setFrameShape(
            QFrame.NoFrame
        )

        self.setHorizontalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff
        )

        self.setVerticalScrollBarPolicy(
            Qt.ScrollBarAlwaysOff
        )

        self.setAlignment(
            Qt.AlignCenter
        )

        self.setBackgroundBrush(
            QColor("#F5F5F7")
        )

        scene = QGraphicsScene(self)

        self.setScene(scene)

        self.proxy = scene.addWidget(
            canvas
        )

        scene.setSceneRect(
            self.proxy.sceneBoundingRect()
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)

        self.fitInView(
            self.proxy.sceneBoundingRect(),
            Qt.KeepAspectRatio,
        )

    def showEvent(self, event):
        super().showEvent(event)

        QTimer.singleShot(
            0,
            lambda: self.fitInView(
                self.proxy.sceneBoundingRect(),
                Qt.KeepAspectRatio,
            ),
        )


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle(
            "龙门架控制系统"
        )

        self.setMinimumSize(
            800,
            540,
        )

        self.resize(
            1100,
            730,
        )

        self.canvas = ControlCanvas()

        self.view = ScaledGraphicsView(
            self.canvas
        )

        self.setCentralWidget(
            self.view
        )

        self.center_window()

    def center_window(self):
        screen = QApplication.primaryScreen()

        if screen is None:
            return

        frame = self.frameGeometry()

        frame.moveCenter(
            screen.availableGeometry().center()
        )

        self.move(
            frame.topLeft()
        )

    def closeEvent(self, event):
        self.canvas.shutdown()
        event.accept()


if __name__ == "__main__":
    QApplication.setAttribute(
        Qt.AA_EnableHighDpiScaling,
        True,
    )

    QApplication.setAttribute(
        Qt.AA_UseHighDpiPixmaps,
        True,
    )

    app = QApplication(sys.argv)

    app.setStyle("Fusion")

    window = MainWindow()

    window.showNormal()

    sys.exit(
        app.exec_()
    )
