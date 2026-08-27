# -*- coding: utf-8 -*-
from __future__ import annotations
import json
from PySide6.QtCore import QObject, Property, Signal


class TwinBridge(QObject):
    stateJsonChanged = Signal()

    def __init__(self, parent=None):
        super().__init__(parent); self._state_json = "{}"

    def getStateJson(self): return self._state_json
    stateJson = Property(str, getStateJson, notify=stateJsonChanged)

    def update_state(self, state):
        text = json.dumps(state or {}, ensure_ascii=False, default=str)
        if text != self._state_json:
            self._state_json = text; self.stateJsonChanged.emit()
