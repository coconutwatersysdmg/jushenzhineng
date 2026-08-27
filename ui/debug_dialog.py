# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QLineEdit, QPushButton, QHBoxLayout,
    QFileDialog, QDialogButtonBox, QLabel, QScrollArea, QWidget
)


class DebugInputDialog(QDialog):
    """离线/联调输入。真实设备接入后这些路径由相机/雷达适配器产生。"""
    def __init__(self, initial=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("v8 调试输入（真实设备接入后不需要）")
        self.resize(920, 760)
        self.lines={}; initial=initial or {}
        lay=QVBoxLayout(self)
        lay.addWidget(QLabel("角点需要 JPG + 与该 JPG 同步对齐的深度图。首轮雷达给4点则用P1-P4；给6点则用P1-P6。"))
        scroll=QScrollArea(); scroll.setWidgetResizable(True); body=QWidget(); form=QFormLayout(body); scroll.setWidget(body); lay.addWidget(scroll,1)

        def add_file(key,label,kind="image",value=""):
            row=QHBoxLayout(); line=QLineEdit(); line.setText(str(value or "")); btn=QPushButton("选择")
            btn.clicked.connect(lambda _=False,l=line,k=kind:self._choose(l,k)); row.addWidget(line,1); row.addWidget(btn); form.addRow(label,row); self.lines[key]=line

        add_file("pallet_rgb","插孔 RGB","image",initial.get("pallet_rgb"))
        add_file("pallet_depth","插孔 Depth","depth",initial.get("pallet_depth"))
        add_file("pre_pick_offset","步骤2 插取前货物-托盘偏差 JPG","image",(initial.get("images") or {}).get("pre_pick_offset"))
        add_file("point_cloud_path","步骤3.2 雷达 PCD","pcd",initial.get("point_cloud_path"))

        cimgs=initial.get("corner_images") or {}; cdeps=initial.get("corner_depths") or {}
        for i in range(1,7):
            pid=f"P{i}"
            add_file(f"{pid}_rgb",f"步骤5 {pid} JPG","image",cimgs.get(pid))
            add_file(f"{pid}_depth",f"步骤5 {pid} Depth","depth",cdeps.get(pid))

        images=initial.get("images") or {}
        add_file("neighbor_pose","步骤8.2 临近托盘姿态 JPG","image",images.get("neighbor_pose"))
        depths=initial.get("depths") or {}
        add_file("pre_place_monitor_rgb","步骤8.4 放置前动态监测 RGB","image",images.get("pre_place_monitor"))
        add_file("pre_place_monitor_depth","步骤8.4 放置前动态监测 Z16 Depth","depth",depths.get("pre_place_monitor"))
        add_file("dynamic_camera_path","动态监测 camera.json","json",initial.get("dynamic_camera_path"))
        add_file("post_place_bottom_rgb","步骤10.0 放置后底层托盘 RGB","image",images.get("post_place_bottom_pallet"))
        add_file("post_place_bottom_depth","步骤10.0 放置后底层托盘 Z16 Depth","depth",depths.get("post_place_bottom_pallet"))
        add_file("region_deviation","步骤10.1 托盘-车板区域偏差 JPG","image",images.get("region_deviation"))
        add_file("face_a","步骤10.1 货物面A JPG","image",images.get("face_a"))
        add_file("face_b","步骤10.1 货物面B JPG","image",images.get("face_b"))
        add_file("post_place_offset","步骤10.2 托盘-货物偏差 JPG","image",images.get("post_place_offset"))

        self.neighbor_json=QLineEdit(json.dumps(initial.get("neighbor_pose_measurement") or {},ensure_ascii=False))
        self.region_json=QLineEdit(json.dumps(initial.get("post_region_measurement") or {},ensure_ascii=False))
        form.addRow("8.2 测量结果JSON（可选 dx_mm/dy_mm/dz_mm/yaw_deg）",self.neighbor_json)
        form.addRow("10.1 测量结果JSON（可选 dx_mm/dy_mm/dz_mm/yaw_deg）",self.region_json)

        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept); buttons.rejected.connect(self.reject); lay.addWidget(buttons)

    def _choose(self,line,kind):
        if kind=="pcd": filt="Point Cloud (*.pcd)"
        elif kind=="json": filt="JSON (*.json)"
        elif kind=="depth": filt="Depth (*.png *.tif *.tiff)"
        else: filt="Images (*.jpg *.jpeg *.png *.bmp *.webp)"
        p,_=QFileDialog.getOpenFileName(self,"选择文件","",filt)
        if p: line.setText(p)

    @staticmethod
    def _json(text):
        try: return json.loads(text.strip() or "{}")
        except Exception: return {}

    def values(self):
        out={
            "pallet_rgb":self.lines["pallet_rgb"].text().strip(),
            "pallet_depth":self.lines["pallet_depth"].text().strip(),
            "point_cloud_path":self.lines["point_cloud_path"].text().strip(),
            "images":{},"depths":{},"corner_images":{},"corner_depths":{},
            "dynamic_camera_path":self.lines["dynamic_camera_path"].text().strip(),
            "neighbor_pose_measurement":self._json(self.neighbor_json.text()),
            "post_region_measurement":self._json(self.region_json.text()),
        }
        for i in range(1,7):
            pid=f"P{i}"; out["corner_images"][pid]=self.lines[f"{pid}_rgb"].text().strip(); out["corner_depths"][pid]=self.lines[f"{pid}_depth"].text().strip()
        for key in ("pre_pick_offset","neighbor_pose","region_deviation","face_a","face_b","post_place_offset"):
            out["images"][key]=self.lines[key].text().strip()
        out["images"]["pre_place_monitor"]=self.lines["pre_place_monitor_rgb"].text().strip()
        out["images"]["post_place_bottom_pallet"]=self.lines["post_place_bottom_rgb"].text().strip()
        out["depths"]["pre_place_monitor"]=self.lines["pre_place_monitor_depth"].text().strip()
        out["depths"]["post_place_bottom_pallet"]=self.lines["post_place_bottom_depth"].text().strip()
        return out
