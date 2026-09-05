# -*- coding: utf-8 -*-
"""角点人工审核 / 手动标点矫正。

识别完成后弹出：审核模型角点是否正确；不对时在图上点击重新标点，
确认后保存矫正标注图，并把矫正后的像素坐标写回流程。
"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap, QFont
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QListWidget,
    QListWidgetItem, QMessageBox, QSplitter, QWidget, QSizePolicy,
)

from utils.cv_io import read_image, write_image

PROJECT_ROOT = Path(__file__).resolve().parents[1]
REVIEW_ROOT = PROJECT_ROOT / "runtime" / "recognition_results" / "corner_review"


class CornerCanvas(QLabel):
    """Scaled image canvas; emits clicks in original image pixel coordinates."""

    pointClicked = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(640, 420)
        self.setStyleSheet("background:#06101d;border:1px solid #244b72;color:#6f8ca5")
        self.setCursor(Qt.CursorShape.CrossCursor)
        self._source: Optional[QPixmap] = None
        self._display: Optional[QPixmap] = None
        self._scale = 1.0
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._markers: List[Dict[str, Any]] = []
        self._active = ""

    def set_image_path(self, path: str | Path, markers: Sequence[Mapping[str, Any]], active: str = ""):
        image = read_image(path)
        if image is None:
            self._source = None
            self.setText(f"无法读取图片：{path}")
            return
        import cv2
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888).copy()
        self._source = QPixmap.fromImage(qimg)
        self._markers = [dict(m) for m in markers]
        self._active = str(active or "")
        self._refresh()

    def set_markers(self, markers: Sequence[Mapping[str, Any]], active: str = ""):
        self._markers = [dict(m) for m in markers]
        self._active = str(active or "")
        self._refresh()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh()

    def _refresh(self):
        if self._source is None or self._source.isNull():
            return
        available = self.size()
        scaled = self._source.scaled(
            available,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._scale = scaled.width() / max(1, self._source.width())
        canvas = QPixmap(available)
        canvas.fill(QColor("#06101d"))
        painter = QPainter(canvas)
        ox = (available.width() - scaled.width()) // 2
        oy = (available.height() - scaled.height()) // 2
        self._offset_x = float(ox)
        self._offset_y = float(oy)
        painter.drawPixmap(ox, oy, scaled)

        for marker in self._markers:
            name = str(marker.get("name") or "")
            u = float(marker.get("x", 0.0))
            v = float(marker.get("y", 0.0))
            sx = ox + u * self._scale
            sy = oy + v * self._scale
            corrected = bool(marker.get("corrected"))
            active = name == self._active
            color = QColor("#ffd76f") if active else (QColor("#61c7ff") if corrected else QColor("#7CFFB2"))
            pen = QPen(color, 2 if not active else 3)
            painter.setPen(pen)
            size = 14 if not active else 18
            painter.drawLine(int(sx - size), int(sy), int(sx + size), int(sy))
            painter.drawLine(int(sx), int(sy - size), int(sx), int(sy + size))
            painter.drawEllipse(int(sx - 8), int(sy - 8), 16, 16)
            painter.setFont(QFont("Microsoft YaHei UI", 10, QFont.Weight.Bold))
            painter.drawText(int(sx + 12), int(sy - 10), f"{name} ({u:.0f},{v:.0f})")
        painter.end()
        self._display = canvas
        self.setPixmap(canvas)
        self.setText("")

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton or self._source is None:
            return
        if self._scale <= 0:
            return
        x = (event.position().x() - self._offset_x) / self._scale
        y = (event.position().y() - self._offset_y) / self._scale
        if x < 0 or y < 0 or x >= self._source.width() or y >= self._source.height():
            return
        self.pointClicked.emit(float(x), float(y))


class CornerReviewDialog(QDialog):
    """Review / manually correct corner pixel points after model recognition."""

    def __init__(
        self,
        image_points: Mapping[str, Mapping[str, Any]],
        corner_ids: Sequence[str],
        parent=None,
        result_tag: str = "review",
    ):
        super().__init__(parent)
        self.setWindowTitle("步骤6 · 角点人工审核 / 手动矫正")
        self.resize(1180, 760)
        self.setModal(True)
        self._accepted = False
        self.result_tag = str(result_tag or "review")
        self.corner_ids = [str(x) for x in corner_ids]
        self.points: Dict[str, Dict[str, Any]] = {
            pid: deepcopy(dict(image_points.get(pid) or {})) for pid in self.corner_ids
        }
        self._original = {pid: deepcopy(self.points[pid]) for pid in self.corner_ids}
        self.groups = self._build_groups()
        self.group_index = 0
        self.active_point = ""

        root = QVBoxLayout(self)
        tip = QLabel(
            "先核对模型角点是否正确。不对时：左侧点选角点编号，再在图片上点击正确位置。"
            "确认通过后会保存矫正标注图，并继续后续 WORLD 转换。"
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#9fdcff;padding:4px 2px")
        root.addWidget(tip)

        split = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(split, 1)

        left = QWidget()
        left_l = QVBoxLayout(left)
        left_l.setContentsMargins(0, 0, 0, 0)
        self.group_label = QLabel("-")
        self.group_label.setStyleSheet("font-weight:700;color:#74d8ff")
        left_l.addWidget(self.group_label)
        self.canvas = CornerCanvas()
        self.canvas.pointClicked.connect(self._on_canvas_click)
        left_l.addWidget(self.canvas, 1)
        nav = QHBoxLayout()
        self.prev_btn = QPushButton("上一组图")
        self.next_btn = QPushButton("下一组图")
        self.prev_btn.clicked.connect(lambda: self._switch_group(-1))
        self.next_btn.clicked.connect(lambda: self._switch_group(1))
        nav.addWidget(self.prev_btn)
        nav.addWidget(self.next_btn)
        nav.addStretch(1)
        left_l.addLayout(nav)
        split.addWidget(left)

        right = QWidget()
        right_l = QVBoxLayout(right)
        right_l.setContentsMargins(8, 0, 0, 0)
        right_l.addWidget(QLabel("本组角点（点击选中后，在图上点一下即可矫正）"))
        self.list = QListWidget()
        self.list.currentItemChanged.connect(self._on_list_changed)
        right_l.addWidget(self.list, 1)
        self.status = QLabel("-")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color:#a9c7d5")
        right_l.addWidget(self.status)

        actions = QVBoxLayout()
        reset_one = QPushButton("重置当前点为模型结果")
        reset_all = QPushButton("重置全部为模型结果")
        reset_one.clicked.connect(self._reset_active)
        reset_all.clicked.connect(self._reset_all)
        actions.addWidget(reset_one)
        actions.addWidget(reset_all)
        right_l.addLayout(actions)

        buttons = QHBoxLayout()
        cancel = QPushButton("取消（中断本步）")
        accept = QPushButton("确认通过并保存")
        accept.setObjectName("primary")
        accept.setStyleSheet("background:#0b72d0;font-weight:700;padding:8px 14px")
        cancel.clicked.connect(self.reject)
        accept.clicked.connect(self._accept_and_save)
        buttons.addWidget(cancel)
        buttons.addStretch(1)
        buttons.addWidget(accept)
        right_l.addLayout(buttons)
        split.addWidget(right)
        split.setSizes([820, 360])

        if self.groups:
            self.active_point = self.groups[0]["point_ids"][0]
        self._reload_group()

    def _build_groups(self) -> List[Dict[str, Any]]:
        buckets: Dict[str, List[str]] = {}
        for pid in self.corner_ids:
            path = str((self.points.get(pid) or {}).get("image") or "")
            if not path:
                raise RuntimeError(f"{pid} 缺少原图路径，无法人工审核")
            buckets.setdefault(path, []).append(pid)
        groups = []
        for index, (path, ids) in enumerate(buckets.items(), start=1):
            groups.append({
                "title": f"第 {index}/{len(buckets)} 组端点图",
                "image_path": path,
                "point_ids": ids,
            })
        return groups

    def _current_group(self) -> Dict[str, Any]:
        return self.groups[self.group_index]

    def _switch_group(self, delta: int):
        if not self.groups:
            return
        self.group_index = (self.group_index + delta) % len(self.groups)
        self.active_point = self.groups[self.group_index]["point_ids"][0]
        self._reload_group()

    def _reload_group(self):
        group = self._current_group()
        self.group_label.setText(f"{group['title']}  ·  {Path(group['image_path']).name}")
        self.prev_btn.setEnabled(len(self.groups) > 1)
        self.next_btn.setEnabled(len(self.groups) > 1)
        self.list.clear()
        for pid in group["point_ids"]:
            point = self.points[pid]
            flag = "人工" if point.get("corrected") else "模型"
            item = QListWidgetItem(
                f"{pid}  [{flag}]  u={float(point.get('x', 0)):.1f}  v={float(point.get('y', 0)):.1f}"
            )
            item.setData(Qt.ItemDataRole.UserRole, pid)
            self.list.addItem(item)
            if pid == self.active_point:
                self.list.setCurrentItem(item)
        markers = []
        for pid in group["point_ids"]:
            point = self.points[pid]
            markers.append({
                "name": pid,
                "x": float(point.get("x", 0.0)),
                "y": float(point.get("y", 0.0)),
                "corrected": bool(point.get("corrected")),
            })
        self.canvas.set_image_path(group["image_path"], markers, self.active_point)
        corrected_count = sum(1 for pid in self.corner_ids if self.points[pid].get("corrected"))
        self.status.setText(
            f"共 {len(self.corner_ids)} 个角点，已人工矫正 {corrected_count} 个。"
            f"当前选中：{self.active_point or '-'}"
        )

    def _on_list_changed(self, current: Optional[QListWidgetItem], _previous):
        if current is None:
            return
        self.active_point = str(current.data(Qt.ItemDataRole.UserRole) or "")
        group = self._current_group()
        markers = []
        for pid in group["point_ids"]:
            point = self.points[pid]
            markers.append({
                "name": pid,
                "x": float(point.get("x", 0.0)),
                "y": float(point.get("y", 0.0)),
                "corrected": bool(point.get("corrected")),
            })
        self.canvas.set_markers(markers, self.active_point)
        self.status.setText(f"当前选中：{self.active_point}。在图上点击即可更新该点像素坐标。")

    def _on_canvas_click(self, u: float, v: float):
        if not self.active_point:
            QMessageBox.information(self, "请先选点", "请先在右侧列表选中要矫正的角点编号。")
            return
        point = self.points[self.active_point]
        point["x"] = float(u)
        point["y"] = float(v)
        point["corrected"] = True
        point["status"] = "人工矫正"
        point["source"] = "manual_correction"
        # Keep a small synthetic box around the new click for downstream display.
        box = 24.0
        point["bbox_xyxy"] = [u - box, v - box, u + box, v + box]
        point["confidence"] = float(point.get("confidence") or 1.0)
        self._reload_group()

    def _reset_active(self):
        if not self.active_point:
            return
        self.points[self.active_point] = deepcopy(self._original[self.active_point])
        self._reload_group()

    def _reset_all(self):
        self.points = {pid: deepcopy(self._original[pid]) for pid in self.corner_ids}
        self._reload_group()

    def _accept_and_save(self):
        try:
            saved = save_corrected_corner_images(self.points, self.corner_ids, self.result_tag)
        except Exception as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return
        for pid, path in saved.items():
            self.points[pid]["result_image"] = path
            self.points[pid]["reviewed"] = True
        self._accepted = True
        self.accept()

    def reviewed_payload(self) -> Dict[str, Any]:
        corrected_ids = [pid for pid in self.corner_ids if self.points[pid].get("corrected")]
        return {
            "success": True,
            "reviewed": True,
            "corner_ids": list(self.corner_ids),
            "image_points": deepcopy(self.points),
            "corrected_point_ids": corrected_ids,
            "corrected_count": len(corrected_ids),
            "result_image_paths_by_point": {
                pid: str(self.points[pid].get("result_image") or "") for pid in self.corner_ids
            },
            "message": (
                f"角点人工审核通过：共 {len(self.corner_ids)} 点，"
                f"其中人工矫正 {len(corrected_ids)} 点；矫正图已保存"
            ),
        }


def save_corrected_corner_images(
    image_points: Mapping[str, Mapping[str, Any]],
    corner_ids: Sequence[str],
    result_tag: str = "review",
) -> Dict[str, str]:
    """Draw reviewed markers and save one annotated image per point."""
    import cv2

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(result_tag or "review"))
    out_dir = REVIEW_ROOT / f"{safe}_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    # One annotated file per unique source image, then copy path onto each point.
    by_image: Dict[str, List[str]] = {}
    for pid in corner_ids:
        path = str((image_points.get(pid) or {}).get("image") or "")
        by_image.setdefault(path, []).append(str(pid))

    saved_by_point: Dict[str, str] = {}
    for image_path, pids in by_image.items():
        image = read_image(image_path, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"无法读取角点原图用于保存矫正结果：{image_path}")
        canvas = image.copy()
        for pid in pids:
            point = image_points[pid]
            u = int(round(float(point.get("x", 0.0))))
            v = int(round(float(point.get("y", 0.0))))
            corrected = bool(point.get("corrected"))
            color = (0, 215, 255) if corrected else (0, 255, 120)
            cv2.drawMarker(canvas, (u, v), color, cv2.MARKER_CROSS, 28, 2)
            cv2.circle(canvas, (u, v), 10, color, 2)
            label = f"{pid} {'MANUAL' if corrected else 'MODEL'} ({u},{v})"
            cv2.putText(
                canvas, label, (max(4, u + 12), max(22, v - 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA,
            )
        shared_name = f"group_{Path(image_path).stem}_reviewed.jpg"
        shared_path = out_dir / shared_name
        if not write_image(shared_path, canvas, [cv2.IMWRITE_JPEG_QUALITY, 94]):
            raise RuntimeError(f"矫正标注图写入失败：{shared_path}")
        for pid in pids:
            # Also keep a per-point copy for module evidence compatibility.
            point_path = out_dir / f"{pid}_reviewed.jpg"
            write_image(point_path, canvas, [cv2.IMWRITE_JPEG_QUALITY, 94])
            saved_by_point[pid] = str(point_path.resolve())
    return saved_by_point


def run_corner_review(
    image_points: Mapping[str, Mapping[str, Any]],
    corner_ids: Sequence[str],
    parent=None,
    result_tag: str = "review",
) -> Dict[str, Any]:
    dialog = CornerReviewDialog(image_points, corner_ids, parent=parent, result_tag=result_tag)
    if dialog.exec() != QDialog.DialogCode.Accepted:
        raise RuntimeError("角点人工审核已取消，步骤6未通过")
    return dialog.reviewed_payload()
