# -*- coding: utf-8 -*-
"""HMI/SCADA monitor theme for the loading digital-twin workstation.

Default: light industrial plant-floor style (Rockwell/ISA-101 inspired):
  - desaturated light-gray background
  - steel-blue accent for controls
  - reserved bright red/amber/green for device & process status
  - dense typography (≈11px UI / 10px tables)

Optional:
  ``JUSHEN_UI_THEME=material`` → qt-material light_pink gallery look
  ``JUSHEN_UI_THEME=dark`` → dark industrial hybrid
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import QApplication

STYLES = Path(__file__).resolve().parent / "styles"
HMI_OVERLAY = STYLES / "hmi_monitor.qss"
LIGHT_MATERIAL_OVERLAY = STYLES / "light_material_overlay.qss"
DARK_OVERLAY = STYLES / "industrial_overlay.qss"
DARK_MATERIAL_THEME = STYLES / "dark_industrial.xml"

HMI_BG = "#e8eaed"
HMI_PANEL = "#ffffff"
HMI_PRIMARY = "#1565c0"
HMI_TEXT = "#1f2329"
HMI_TWIN = "#d5dae2"

DARK_BG = "#0b1220"
DARK_PANEL = "#121a2b"
DARK_PRIMARY = "#74d8ff"
DARK_TEXT = "#e8eef7"

DARK_COLORS: dict[str, Any] = {
    "[dark]": {
        "primary": DARK_PRIMARY,
        "background": DARK_BG,
        "foreground": DARK_TEXT,
        "border": "#2a3f5d",
        "input.background": "#152841",
        "tableSectionHeader.background": "#152841",
        "statusBar.background": DARK_PANEL,
        "toolbar.background": DARK_PANEL,
    }
}

_ACTIVE_MODE = "light"


def is_light_theme() -> bool:
    return _ACTIVE_MODE == "light"


def twin_clear_color() -> QColor:
    return QColor(HMI_TWIN if is_light_theme() else DARK_BG)


def load_overlay_qss(name: str) -> str:
    mapping = {
        "hmi": HMI_OVERLAY,
        "light": LIGHT_MATERIAL_OVERLAY,
        "dark": DARK_OVERLAY,
    }
    path = mapping.get(name, HMI_OVERLAY)
    if path.is_file():
        return path.read_text(encoding="utf-8")
    return ""


def _apply_app_font(app: QApplication) -> None:
    # Dense HMI base: 9pt ≈ 12 logical px on 96 DPI; QSS further sets 10–11px.
    font = QFont("Microsoft YaHei UI", 9)
    font.setStyleHint(QFont.StyleHint.SansSerif)
    font.setHintingPreference(QFont.HintingPreference.PreferFullHinting)
    app.setFont(font)


def _set_palette_light(app: QApplication, accent: str = HMI_PRIMARY) -> None:
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(HMI_BG))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(HMI_TEXT))
    palette.setColor(QPalette.ColorRole.Base, QColor(HMI_PANEL))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#f4f6f8"))
    palette.setColor(QPalette.ColorRole.Text, QColor(HMI_TEXT))
    palette.setColor(QPalette.ColorRole.Button, QColor(HMI_PANEL))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(HMI_TEXT))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(accent))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(HMI_PANEL))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(HMI_TEXT))
    app.setPalette(palette)


def _apply_hmi(app: QApplication, overlay: str) -> str:
    app.setStyle("Fusion")
    _set_palette_light(app, HMI_PRIMARY)
    app.setStyleSheet(overlay or "")
    return "hmi-monitor"


def _apply_qt_material_light(app: QApplication, overlay: str) -> str:
    from qt_material import apply_stylesheet

    theme = str(os.environ.get("JUSHEN_MATERIAL_THEME") or "light_pink.xml").strip()
    if not theme.endswith(".xml"):
        theme = f"{theme}.xml"
    extra = {
        "font_family": "Microsoft YaHei UI",
        "font_size": "11px",
        "density_scale": "-2",
        "button_shape": "default",
    }
    apply_stylesheet(app, theme=theme, invert_secondary=True, extra=extra)
    if overlay:
        app.setStyleSheet((app.styleSheet() or "") + "\n" + overlay)
    return f"qt-material:{theme}"


def _apply_qt_material_dark(app: QApplication, overlay: str) -> str:
    from qt_material import apply_stylesheet

    theme = str(DARK_MATERIAL_THEME) if DARK_MATERIAL_THEME.is_file() else "dark_cyan.xml"
    extra = {
        "font_family": "Microsoft YaHei UI",
        "font_size": "11px",
        "density_scale": "-2",
        "button_shape": "default",
    }
    apply_stylesheet(app, theme=theme, invert_secondary=False, extra=extra)
    if overlay:
        app.setStyleSheet((app.styleSheet() or "") + "\n" + overlay)
    return "qt-material:dark"


def _apply_qdarktheme(app: QApplication, overlay: str) -> str:
    import qdarktheme

    qdarktheme.setup_theme(
        theme="dark",
        corner_shape="sharp",
        custom_colors=DARK_COLORS,
        additional_qss=overlay or None,
    )
    return "qdarktheme"


def _apply_fallback_dark(app: QApplication, overlay: str) -> str:
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(DARK_BG))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(DARK_TEXT))
    palette.setColor(QPalette.ColorRole.Base, QColor("#0a101c"))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(DARK_PANEL))
    palette.setColor(QPalette.ColorRole.Text, QColor(DARK_TEXT))
    palette.setColor(QPalette.ColorRole.Button, QColor("#173554"))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(DARK_TEXT))
    palette.setColor(QPalette.ColorRole.Highlight, QColor("#0b72d0"))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    app.setPalette(palette)
    app.setStyle("Fusion")
    app.setStyleSheet(overlay or "")
    return "fallback-dark"


def apply_app_theme(app: QApplication | None = None) -> str:
    """Apply global UI theme. Returns engine name used.

    Env ``JUSHEN_UI_THEME``:
      - ``hmi`` / ``light`` / unset (default): industrial monitor HMI
      - ``material`` / ``pink``: qt-material light gallery
      - ``dark``: dark industrial
    """
    global _ACTIVE_MODE

    app = app or QApplication.instance()
    if app is None:
        raise RuntimeError("QApplication must exist before apply_app_theme()")

    _apply_app_font(app)
    choice = str(os.environ.get("JUSHEN_UI_THEME", "hmi") or "hmi").strip().lower()

    if choice in {"dark", "qdarktheme", "industrial"}:
        _ACTIVE_MODE = "dark"
        overlay = load_overlay_qss("dark")
        engines = [_apply_qdarktheme, _apply_qt_material_dark, _apply_fallback_dark]
    elif choice in {"material", "pink", "qt-material", "qt_material"}:
        _ACTIVE_MODE = "light"
        overlay = load_overlay_qss("light")
        engines = [_apply_qt_material_light, lambda a, o: _apply_hmi(a, load_overlay_qss("hmi"))]
    else:
        # hmi / light / monitor — plant-floor monitoring default
        _ACTIVE_MODE = "light"
        overlay = load_overlay_qss("hmi")
        engines = [_apply_hmi]

    last_error: Exception | None = None
    for engine in engines:
        try:
            name = engine(app, overlay)
            app.setProperty("jushenUiTheme", name)
            app.setProperty("jushenUiMode", _ACTIVE_MODE)
            return name
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            continue
    if last_error:
        raise last_error
    return "none"
