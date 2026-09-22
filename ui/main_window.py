# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path

from PySide6.QtCore import QEventLoop, Qt, QTimer, QUrl, Signal
from PySide6.QtGui import QColor, QPixmap
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QFrame, QLabel,
    QPushButton, QTableWidget, QTableWidgetItem, QHeaderView, QTextEdit, QMessageBox,
    QTabWidget, QToolButton, QSizePolicy, QComboBox, QCheckBox,
)

from controllers.flow_controller import FlowController
from ui.twin_bridge import TwinBridge
from ui.debug_dialog import DebugInputDialog
from ui.corner_review_dialog import run_corner_review
from ui.plc_motion_dialog import PlcMotionDialog
from config.feature_switches import (
    apply_run_profile,
    get_run_profile,
    list_run_profiles,
)
import config.feature_switches as feature_switches

PROJECT_ROOT=Path(__file__).resolve().parents[1]
QML_FILE=PROJECT_ROOT/"ui"/"qml"/"TwinScene.qml"
DEFAULT_PLAN=PROJECT_ROOT/"data"/"loading_plan.json"


class MainWindow(QMainWindow):
    motionSnapshot = Signal(dict, dict)
    plcCommandReady = Signal(dict)

    FLOW_STAGES = [
        (1,"设备连接检查"),
        (2,"货物-托盘偏差"),
        (3,"3.1 插取 ∥ 3.2 雷达"),
        (4,"雷达点 → PLC → 相机"),
        (5,"角点 RGB-D"),
        (6,"角点识别 / 人工审核"),
        (7,"转换 WORLD"),
        (8,"8.1 规划 · 8.2 补偿 · 8.3 锁定"),
        (9,"放置前监测 / 计算位置 / 放置"),
        (10,"底托检测 · 区域两面 · 货物偏差"),
        (11,"反馈 PLC / 空间管理"),
        (12,"机械臂返回 / 下一轮"),
    ]
    LAB_FLOW_STAGES = [
        (1, "设备连接检查"),
        (2, "相机插孔识别 ∥ 雷达四角粗定位"),
        (3, "相机精定位（拍照+YOLO）"),
        (4, "B列循环：放前监测 / 手动放货 / 放后检测"),
    ]
    STEP_STAGE = {
        "DEVICE_CHECK":1,"LAB_SENSE":2,"LAB_CORNER_SHELL":3,
        "LAB_RETURN_ORIGIN":4,"LAB_PRE_PLACE_MONITOR":4,"LAB_PLACE_VERIFY":4,
        "PRE_PICK_OFFSET":2,"PARALLEL_LOCATE":3,"PICK_ONLY":3,"RADAR_TO_CAMERA":4,
        "CAPTURE_CORNERS":5,"CORNER_RECOGNITION":6,"CAMERA_TO_WORLD":7,
        "INITIAL_SPACE_PLAN":8,"NEIGHBOR_POSE":8,"TARGET_CONFIRM":8,"PRE_PLACE_MONITOR":9,"PLACE":9,
        "POST_PLACE_BOTTOM":10,"POST_REGION":10,"POST_CARGO_OFFSET":10,"FEEDBACK":11,"RETURN":12,"DONE":12,
    }

    def __init__(self):
        super().__init__()
        self.controller=FlowController(); self.bridge=TwinBridge(); self.debug_inputs=self.controller.default_debug_inputs()
        self._step_busy=False
        self._running_stage=0  # 正在执行的流程格编号；与 step_code（可能已前进到下一步）区分
        self._profile_changing=False
        self._latest_plc_cmd=None
        self._plc_cmd_queue=[]
        self._latest_plc_batch=[]
        self.plc_dialog=None
        self.motionSnapshot.connect(self._apply_motion_snapshot)
        self.plcCommandReady.connect(self._enqueue_plc_command, Qt.ConnectionType.QueuedConnection)
        self.controller.set_state_listener(self.motionSnapshot.emit)
        self.controller.set_plc_command_listener(self.plcCommandReady.emit)
        self.controller.set_corner_review_callback(self._review_corners)
        self.timer=QTimer(self); self.timer.setInterval(900); self.timer.timeout.connect(self._auto_tick)
        self.setWindowTitle("具身智能装载数字孪生 · 单机械臂携货 / 挂载相机角点识别")
        screen=self.screen().availableGeometry()
        self.resize(min(1920,max(1080,int(screen.width()*0.94))),min(1120,max(700,int(screen.height()*0.92))))
        self.setMinimumSize(1024,680)
        self._build(); self._ensure_plc_dialog(); self._load_plan(); self._apply_ui_mode(); self._refresh(); self._sync_auto_push_policy()

    def _card(self,title):
        f=QFrame(); f.setObjectName("card"); l=QVBoxLayout(f); l.setContentsMargins(9,8,9,8)
        t=QLabel(title); t.setObjectName("sectionTitle"); l.addWidget(t); return f,l

    def _collapsible_card(self,title,*,expanded=True):
        f=QFrame(); f.setObjectName("card"); outer=QVBoxLayout(f); outer.setContentsMargins(9,8,9,8); outer.setSpacing(6)
        header=QHBoxLayout(); header.setSpacing(6)
        toggle=QToolButton(); toggle.setCheckable(True); toggle.setChecked(expanded); toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        toggle.setArrowType(Qt.ArrowType.DownArrow if expanded else Qt.ArrowType.RightArrow)
        toggle.setStyleSheet("QToolButton{border:none;background:transparent;color:#74d8ff;padding:0 2px}")
        toggle.setToolTip("展开 / 折叠")
        title_label=QLabel(title); title_label.setObjectName("sectionTitle"); title_label.setCursor(Qt.CursorShape.PointingHandCursor)
        header.addWidget(toggle,0); header.addWidget(title_label,1); header.addStretch(1); outer.addLayout(header)
        body=QWidget(); body_l=QVBoxLayout(body); body_l.setContentsMargins(0,0,0,0); body_l.setSpacing(6); outer.addWidget(body); body.setVisible(expanded)

        def _set_expanded(opened):
            body.setVisible(opened)
            toggle.setArrowType(Qt.ArrowType.DownArrow if opened else Qt.ArrowType.RightArrow)
            if toggle.isChecked()!=opened: toggle.setChecked(opened)
            f.setSizePolicy(
                QSizePolicy.Policy.Preferred,
                QSizePolicy.Policy.Preferred if opened else QSizePolicy.Policy.Maximum,
            )
            f.updateGeometry()

        _set_expanded(expanded)
        toggle.toggled.connect(_set_expanded)
        title_label.mousePressEvent=lambda _event: toggle.setChecked(not toggle.isChecked())
        return f,body_l,toggle

    def _build(self):
        root=QWidget(); self.setCentralWidget(root)
        root.setStyleSheet("""
        QWidget{background:#07111f;color:#dcecff;font-size:12px}
        QFrame#card{background:#0c1b2e;border:1px solid #1f537e;border-radius:8px}
        QLabel#title{font-size:22px;font-weight:700;color:#fff}
        QLabel#sectionTitle{font-size:14px;font-weight:700;color:#74d8ff}
        QPushButton{background:#12395f;border:1px solid #2a78bd;border-radius:5px;padding:7px 11px}
        QPushButton:hover{background:#194a78} QPushButton#primary{background:#0b72d0;font-weight:700}
        QComboBox{background:#102a46;border:1px solid #2a78bd;border-radius:5px;padding:5px 8px;min-width:160px}
        QComboBox::drop-down{border:0;width:22px}
        QComboBox QAbstractItemView{background:#0c1b2e;border:1px solid #2a78bd;selection-background-color:#194a78}
        QTableWidget,QTextEdit{background:#06101d;border:1px solid #244b72;border-radius:4px}
        QHeaderView::section{background:#102a46;color:#dcecff;padding:5px;border:0}
        """)
        lay=QVBoxLayout(root); lay.setContentsMargins(12,10,12,10); lay.setSpacing(8)
        top=QHBoxLayout(); title=QLabel("具身智能装载数字孪生"); title.setObjectName("title"); top.addWidget(title)
        top.addStretch(1)
        mode_label=QLabel("运行模式"); mode_label.setStyleSheet("color:#9ec8dc;font-weight:600")
        top.addWidget(mode_label)
        self.profile_combo=QComboBox()
        for item in list_run_profiles():
            self.profile_combo.addItem(f"{item['title']}", item["id"])
            idx=self.profile_combo.count()-1
            self.profile_combo.setItemData(idx, item.get("subtitle") or "", Qt.ItemDataRole.ToolTipRole)
        current=str(feature_switches.RUN_PROFILE or "field")
        found=self.profile_combo.findData(current)
        if found>=0: self.profile_combo.setCurrentIndex(found)
        self.profile_combo.currentIndexChanged.connect(self._on_run_profile_changed)
        self._sync_profile_combo_tooltip()
        top.addWidget(self.profile_combo)
        self.phase=QLabel("IDLE"); self.phase.setStyleSheet("font-size:15px;font-weight:700;color:#5ad0ff"); top.addWidget(self.phase)
        self.progress=QLabel("0/0"); self.progress.setStyleSheet("font-size:18px;font-weight:700"); top.addWidget(self.progress); lay.addLayout(top)

        # 三件外设：紧凑状态条（绿点已连接 / 红点未连接），悬停看原因
        device_strip=QHBoxLayout(); device_strip.setSpacing(14)
        strip_title=QLabel("外接设备")
        strip_title.setStyleSheet("color:#8eb6cc;font-weight:600;font-size:12px")
        device_strip.addWidget(strip_title,0)
        self.device_status_badges={}
        for device_id,title in (("PLC","PLC"),("RADAR","雷达"),("CAM_PICK","相机")):
            badge=QLabel(f"○ {title} 未检测")
            badge.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
            badge.setStyleSheet("color:#8faabd;font-size:12px;font-weight:600;padding:2px 0")
            self.device_status_badges[device_id]=badge
            device_strip.addWidget(badge,0)
        device_strip.addStretch(1)
        lay.addLayout(device_strip)

        f,l,self.flow_toggle=self._collapsible_card("流程链路",expanded=True)
        self.field_flow_card=f
        self.flow_nodes=[]
        for row_index,stage_row in enumerate((self.FLOW_STAGES[:6],self.FLOW_STAGES[6:])):
            row=QHBoxLayout(); row.setSpacing(5)
            for index,(number,title_text) in enumerate(stage_row):
                node=QLabel(f"{number}. {title_text}"); node.setAlignment(Qt.AlignmentFlag.AlignCenter); node.setWordWrap(True)
                node.setMinimumHeight(42); node.setToolTip(f"步骤 {number}：{title_text}"); row.addWidget(node,1); self.flow_nodes.append(node)
                if index < len(stage_row)-1:
                    arrow=QLabel("→"); arrow.setAlignment(Qt.AlignmentFlag.AlignCenter); arrow.setStyleSheet("color:#527a96;font-size:16px"); row.addWidget(arrow,0)
            l.addLayout(row)
        lay.addWidget(f,0)

        lab_flow,lab_flow_l,self.lab_flow_toggle=self._collapsible_card("实验室流程（前两步）",expanded=True)
        self.lab_flow_card=lab_flow
        self.lab_flow_nodes=[]
        lab_row=QHBoxLayout(); lab_row.setSpacing(5)
        for index,(number,title_text) in enumerate(self.LAB_FLOW_STAGES):
            node=QLabel(f"{number}. {title_text}"); node.setAlignment(Qt.AlignmentFlag.AlignCenter); node.setWordWrap(True)
            node.setMinimumHeight(42); node.setToolTip(f"实验室步骤 {number}：{title_text}")
            lab_row.addWidget(node,1); self.lab_flow_nodes.append(node)
            if index < len(self.LAB_FLOW_STAGES)-1:
                arrow=QLabel("→"); arrow.setAlignment(Qt.AlignmentFlag.AlignCenter); arrow.setStyleSheet("color:#527a96;font-size:16px"); lab_row.addWidget(arrow,0)
        lab_flow_l.addLayout(lab_row)
        lab_flow.setVisible(False)
        lay.addWidget(lab_flow,0)

        self.body_splitter=QSplitter(Qt.Orientation.Vertical); self.body_splitter.setChildrenCollapsible(False); lay.addWidget(self.body_splitter,1)
        main=QSplitter(Qt.Orientation.Horizontal); main.setChildrenCollapsible(False); self.body_splitter.addWidget(main); self.main_splitter=main
        left=QWidget(); ll=QVBoxLayout(left); ll.setContentsMargins(0,0,0,0); ll.setSpacing(8); main.addWidget(left)
        center=QWidget(); cl=QVBoxLayout(center); cl.setContentsMargins(0,0,0,0); cl.setSpacing(8); main.addWidget(center)
        right=QWidget(); rl=QVBoxLayout(right); rl.setContentsMargins(0,0,0,0); rl.setSpacing(8); main.addWidget(right)
        main.setStretchFactor(0,2); main.setStretchFactor(1,5); main.setStretchFactor(2,3)
        main.setSizes([320,820,480])

        f,l,_=self._collapsible_card("货物 / 托盘实时数据",expanded=True)
        self.cargo_card=f
        self.cargo_table=QTableWidget(0,2); self.cargo_table.setHorizontalHeaderLabels(["字段","值"]); self.cargo_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch); l.addWidget(self.cargo_table); ll.addWidget(f,1)

        f,l,_=self._collapsible_card("车辆 / 相机最终车板 WORLD 数据",expanded=True)
        self.truck_card=f
        self.truck_table=QTableWidget(0,2); self.truck_table.setHorizontalHeaderLabels(["字段","值"]); self.truck_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch); l.addWidget(self.truck_table)
        self.corner_table=QTableWidget(0,4); self.corner_table.setHorizontalHeaderLabels(["角点","X","Y","Z"]); self.corner_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch); l.addWidget(self.corner_table); ll.addWidget(f,2)

        f,l=self._card("数字孪生场景 · world / mm · X右 Y车辆前进 Z向上")
        self.quick=QQuickWidget(); self.quick.setResizeMode(QQuickWidget.ResizeMode.SizeRootObjectToView); self.quick.setClearColor(QColor("#07111f"))
        self.quick.rootContext().setContextProperty("twinBridge",self.bridge); self.quick.statusChanged.connect(self._on_qml_status); self.quick.setSource(QUrl.fromLocalFile(str(QML_FILE))); l.addWidget(self.quick,1); cl.addWidget(f,1)
        controls=QHBoxLayout()
        self.next_btn=QPushButton("执行下一步"); self.next_btn.setObjectName("primary"); self.next_btn.clicked.connect(self._next)
        self.auto_btn=QPushButton("自动运行"); self.auto_btn.clicked.connect(self._auto)
        self.debug_btn=QPushButton("调试输入"); self.debug_btn.clicked.connect(self._debug); reset=QPushButton("重置"); reset.clicked.connect(self._reset)
        self.plc_panel_btn=QPushButton("PLC 运动…")
        self.plc_panel_btn.setToolTip("打开弹出窗口：查看输出坐标、确认下发、启动 PLC 控制台")
        self.plc_panel_btn.clicked.connect(self._show_plc_dialog)
        self.auto_push_cb=QCheckBox("自动运行时自动下发到 PLC")
        self.auto_push_cb.setToolTip("勾选且处于自动运行时，算出坐标后经已连接的 PLC 直接写轴。逐步执行请在弹窗里确认下发。")
        self.auto_push_cb.toggled.connect(lambda _=False: self._sync_auto_push_policy())
        for b in (self.next_btn,self.auto_btn,self.debug_btn,reset,self.plc_panel_btn): controls.addWidget(b)
        controls.addWidget(self.auto_push_cb)
        controls.addStretch(1); cl.addLayout(controls)

        self.right_tabs=QTabWidget(); rl.addWidget(self.right_tabs,1)
        self.field_right_panel=self.right_tabs
        device_page=QWidget(); device_layout=QVBoxLayout(device_page); device_layout.setContentsMargins(5,5,5,5); device_layout.setSpacing(7)
        space_page=QWidget(); space_layout=QVBoxLayout(space_page); space_layout.setContentsMargins(5,5,5,5)

        f,l,_=self._collapsible_card("设备 / 单机械臂与挂载相机实时 WORLD 坐标",expanded=True)
        self.device_table=QTableWidget(0,7); self.device_table.setMinimumHeight(130); self.device_table.setHorizontalHeaderLabels(["设备","类型","父设备","状态","WORLD XYZ","RPY","当前动作"]); self.device_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents); self.device_table.horizontalHeader().setStretchLastSection(True); l.addWidget(self.device_table)
        device_layout.addWidget(f,3)

        f,l,_=self._collapsible_card("空间管理 · 相机几何 · 两列 × 1.2m",expanded=True)
        self.space_table=QTableWidget(0,7); self.space_table.setHorizontalHeaderLabels(["盲码","板段","列","中心XYZ","支撑块","状态","货物"]); self.space_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch); l.addWidget(self.space_table)
        self.comp=QLabel("补偿：-"); self.comp.setWordWrap(True); l.addWidget(self.comp); space_layout.addWidget(f,1)

        f,l,_=self._collapsible_card("并行状态 / 运行消息 / 标定状态",expanded=True)
        self.parallel_label=QLabel("-"); self.parallel_label.setWordWrap(True); l.addWidget(self.parallel_label)
        self.cal_label=QLabel("-"); self.cal_label.setMaximumHeight(38); self.cal_label.setStyleSheet("color:#9ec8dc"); l.addWidget(self.cal_label)
        self.database_label=QLabel("车辆数据库：等待装载会话")
        self.database_label.setWordWrap(True); self.database_label.setStyleSheet("color:#8fd4b8"); l.addWidget(self.database_label)
        self.message=QTextEdit(); self.message.setReadOnly(True); self.message.setMinimumHeight(110); l.addWidget(self.message,1)
        device_layout.addWidget(f,2)
        self.right_tabs.addTab(device_page,"设备与消息")
        self.right_tabs.addTab(space_page,"空间与补偿")

        module_page=QWidget(); module_layout=QVBoxLayout(module_page); module_layout.setContentsMargins(5,5,5,5); module_layout.setSpacing(7)
        self.module_summary=QLabel("等待流程调用模型/模块；未执行步骤不提前显示")
        self.module_summary.setWordWrap(True); self.module_summary.setStyleSheet("color:#9fdcff;font-weight:600")
        module_layout.addWidget(self.module_summary)
        self.module_table=QTableWidget(0,8)
        self.module_table.setHorizontalHeaderLabels(["阶段","模型/模块","类型","实现/权重","状态","调用","耗时ms","记录"])
        self.module_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.module_table.horizontalHeader().setStretchLastSection(True)
        self.module_table.setMinimumHeight(150)
        self.module_table.cellClicked.connect(self._show_module_detail)
        module_layout.addWidget(self.module_table,2)

        self.current_module_title=QLabel("当前模型/模块：等待流程")
        self.current_module_title.setWordWrap(True)
        self.current_module_title.setStyleSheet("color:#ffffff;font-size:13px;font-weight:700;background:#102a46;border:1px solid #2a78bd;border-radius:5px;padding:6px")
        module_layout.addWidget(self.current_module_title)

        io_splitter=QSplitter(Qt.Orientation.Horizontal); io_splitter.setChildrenCollapsible(False)
        input_card,input_layout,_=self._collapsible_card("本次输入（数据 + 图片）",expanded=True)
        self.module_input_preview=QLabel("该步骤执行后显示输入图片")
        self.module_input_preview.setAlignment(Qt.AlignmentFlag.AlignCenter); self.module_input_preview.setMinimumHeight(125)
        self.module_input_preview.setStyleSheet("background:#06101d;border:1px solid #244b72;color:#6f8ca5")
        input_layout.addWidget(self.module_input_preview,2)
        self.module_input_data=QTextEdit(); self.module_input_data.setReadOnly(True); self.module_input_data.setMinimumHeight(135)
        self.module_input_data.setPlaceholderText("当前步骤的图像路径、深度图、点云、位姿、参数等输入将显示在这里")
        input_layout.addWidget(self.module_input_data,3); io_splitter.addWidget(input_card)

        output_card,output_layout,_=self._collapsible_card("本次输出（数据 + 图片）",expanded=True)
        self.module_output_preview=QLabel("该步骤执行后显示输出图片")
        self.module_output_preview.setAlignment(Qt.AlignmentFlag.AlignCenter); self.module_output_preview.setMinimumHeight(125)
        self.module_output_preview.setStyleSheet("background:#06101d;border:1px solid #244b72;color:#6f8ca5")
        output_layout.addWidget(self.module_output_preview,2)
        self.module_output_data=QTextEdit(); self.module_output_data.setReadOnly(True); self.module_output_data.setMinimumHeight(135)
        self.module_output_data.setPlaceholderText("当前步骤的识别结果、坐标、判断、耗时和输出文件将显示在这里")
        output_layout.addWidget(self.module_output_data,3); io_splitter.addWidget(output_card)
        io_splitter.setSizes([500,500]); module_layout.addWidget(io_splitter,5)
        self.right_tabs.addTab(module_page,"模型输入/输出")
        self._module_rows=[]; self._selected_module_id=""; self._latest_module_id=""

        # 实验室右侧：相机照片 + 插孔坐标 + 雷达四角
        self.lab_right_panel=QWidget()
        lab_rl=QVBoxLayout(self.lab_right_panel); lab_rl.setContentsMargins(0,0,0,0); lab_rl.setSpacing(8)
        cam_card,cam_l,_=self._collapsible_card("相机照片 / 插孔坐标",expanded=True)
        self.lab_camera_preview=QLabel("第 2 步执行后显示托盘/插孔识别图")
        self.lab_camera_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lab_camera_preview.setMinimumHeight(220)
        self.lab_camera_preview.setStyleSheet("background:#06101d;border:1px solid #244b72;color:#6f8ca5")
        cam_l.addWidget(self.lab_camera_preview,3)
        self.lab_hole_table=QTableWidget(0,4)
        self.lab_hole_table.setHorizontalHeaderLabels(["插孔","X(mm)","Y(mm)","Z(mm)"])
        self.lab_hole_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.lab_hole_table.setMaximumHeight(120)
        cam_l.addWidget(self.lab_hole_table,1)
        lab_rl.addWidget(cam_card,3)

        radar_card,radar_l,_=self._collapsible_card("角点 WORLD（相机优先 / 雷达兜底，mm）",expanded=True)
        self.lab_radar_table=QTableWidget(0,5)
        self.lab_radar_table.setHorizontalHeaderLabels(["角点","X","Y","Z","来源"])
        self.lab_radar_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        radar_l.addWidget(self.lab_radar_table,1)
        lab_rl.addWidget(radar_card,2)
        self.lab_right_panel.setVisible(False)
        rl.addWidget(self.lab_right_panel,1)

        bottom=QSplitter(Qt.Orientation.Horizontal); bottom.setChildrenCollapsible(False); self.body_splitter.addWidget(bottom)
        f,l,_=self._collapsible_card("运行日志 / 报警",expanded=True); self.logs=QTextEdit(); self.logs.setReadOnly(True); self.logs.setMinimumHeight(110); l.addWidget(self.logs); bottom.addWidget(f)
        # 总体结果 JSON 过长暂不展示；数据仍写入 workdir 结果文件。
        self.results=None
        self.body_splitter.setStretchFactor(0,5); self.body_splitter.setStretchFactor(1,1); self.body_splitter.setSizes([820,145])

    def _on_qml_status(self,status):
        if status==QQuickWidget.Status.Error:
            msg="\n".join(e.toString() for e in self.quick.errors()) or "未知 QML 错误"
            QTimer.singleShot(0,lambda m=msg: QMessageBox.critical(self,"数字孪生场景加载失败",m))

    @staticmethod
    def _fill_kv(table,data):
        table.setRowCount(0)
        for k,v in data:
            r=table.rowCount(); table.insertRow(r); table.setItem(r,0,QTableWidgetItem(str(k))); table.setItem(r,1,QTableWidgetItem(str(v)))

    def _load_plan(self):
        if DEFAULT_PLAN.is_file():
            try:
                data=json.loads(DEFAULT_PLAN.read_text(encoding="utf-8")); self.controller.set_plan(data.get("items",data))
            except Exception: pass
        if not self.controller.queue:
            self.controller.set_plan([{"cargo_code":"CARGO-001","cargo_name":"货物","quantity":3,"length_mm":1200,"width_mm":1000,"height_mm":900,"pallet_reference_width_mm":1200}])

    def _sync_profile_combo_tooltip(self):
        profile=get_run_profile(self.profile_combo.currentData())
        tip=f"{profile.get('title')}：{profile.get('subtitle')}"
        switches=profile.get("switches") or {}
        tip += (
            f"\nDEVICE_MODE={switches.get('DEVICE_MODE')}"
            f"\nLAB_LIDAR={switches.get('USE_LAB_LIDAR_ALGO')}"
            f"  LAB_CAMERA={switches.get('USE_LAB_CAMERA_ALGO')}"
            f"\nALLOW_DEMO={switches.get('ALLOW_DEMO_DEVICE_DATA')}"
            f"  REAL_MOTION={switches.get('ALLOW_REAL_MOTION')}"
        )
        self.profile_combo.setToolTip(tip)

    def _on_run_profile_changed(self, _index: int = 0):
        if self._profile_changing:
            return
        profile_id=str(self.profile_combo.currentData() or "field")
        if profile_id == str(feature_switches.RUN_PROFILE or ""):
            self._sync_profile_combo_tooltip()
            return
        if self.controller.running or self._step_busy or self.timer.isActive():
            reply=QMessageBox.question(
                self,
                "切换运行模式",
                "当前流程正在运行。切换模式会重置控制器并重建设备连接，是否继续？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if reply != QMessageBox.StandardButton.Yes:
                self._profile_changing=True
                found=self.profile_combo.findData(str(feature_switches.RUN_PROFILE or "field"))
                if found>=0: self.profile_combo.setCurrentIndex(found)
                self._profile_changing=False
                return
        self._apply_run_profile(profile_id)

    def _apply_run_profile(self, profile_id: str):
        self.timer.stop()
        self._step_busy=False
        old_plan=list(getattr(self.controller, "queue", []) or [])
        old_debug=deepcopy(getattr(self.controller, "debug_inputs", {}) or self.debug_inputs)
        applied=apply_run_profile(profile_id, persist=True)
        self.controller=FlowController()
        self.controller.set_state_listener(self.motionSnapshot.emit)
        self.controller.set_plc_command_listener(self.plcCommandReady.emit)
        self.controller.set_corner_review_callback(self._review_corners)
        self._plc_cmd_queue.clear()
        self._latest_plc_cmd=None
        self._latest_plc_batch=[]
        if self.plc_dialog is not None:
            self.plc_dialog.clear_command()
        self._sync_auto_push_policy()
        if old_plan:
            self.controller.set_plan(old_plan)
        else:
            self._load_plan()
        if old_debug:
            self.debug_inputs=old_debug
            self.controller.set_debug_inputs(old_debug)
        else:
            self.debug_inputs=self.controller.default_debug_inputs()
        self._sync_profile_combo_tooltip()
        self.controller.twin.add_message(
            "RUN_PROFILE",
            "INFO",
            f"已切换运行模式：{applied.get('title')}（{applied.get('id')}）",
            applied.get("switches") or {},
        )
        self._apply_ui_mode()
        self._refresh()

    def _is_lab_ui(self) -> bool:
        return str(feature_switches.RUN_PROFILE or "").strip().lower() == "lab"

    def _apply_ui_mode(self) -> None:
        lab = self._is_lab_ui()
        if hasattr(self, "field_flow_card"):
            self.field_flow_card.setVisible(not lab)
        if hasattr(self, "lab_flow_card"):
            self.lab_flow_card.setVisible(lab)
        if hasattr(self, "truck_card"):
            self.truck_card.setVisible(not lab)
        if hasattr(self, "field_right_panel"):
            self.field_right_panel.setVisible(not lab)
        if hasattr(self, "lab_right_panel"):
            self.lab_right_panel.setVisible(lab)
        if hasattr(self, "debug_btn"):
            self.debug_btn.setVisible(not lab)
        title = "实验室装载测试 · B列循环（B1 → B2 → B3）" if lab else "具身智能装载数字孪生 · 单机械臂携货 / 挂载相机角点识别"
        self.setWindowTitle(title)
        if lab and hasattr(self, "main_splitter"):
            self.main_splitter.setSizes([280, 780, 420])
        elif hasattr(self, "main_splitter"):
            self.main_splitter.setSizes([320, 820, 480])

    @staticmethod
    def _first_image(value):
        if isinstance(value,dict):
            preferred=("result_image_path","result_image","rgb_path","image_path","face_a","face_b")
            for key in preferred:
                if key in value:
                    found=MainWindow._first_image(value[key])
                    if found: return found
            for item in value.values():
                found=MainWindow._first_image(item)
                if found: return found
        elif isinstance(value,(list,tuple)):
            for item in value:
                found=MainWindow._first_image(item)
                if found: return found
        elif isinstance(value,str):
            path=Path(value)
            if path.is_file() and path.suffix.lower() in {".jpg",".jpeg",".png",".bmp",".webp"}:
                return str(path.resolve())
        return ""

    @staticmethod
    def _image_paths(value):
        found=[]
        def visit(item):
            if isinstance(item,dict):
                for child in item.values(): visit(child)
            elif isinstance(item,(list,tuple)):
                for child in item: visit(child)
            elif isinstance(item,str):
                path=Path(item)
                if path.is_file() and path.suffix.lower() in {".jpg",".jpeg",".png",".bmp",".webp"}:
                    resolved=str(path.resolve())
                    if resolved not in found: found.append(resolved)
        visit(value)
        return found

    @staticmethod
    def _set_io_preview(label,image_data,empty_text):
        paths=MainWindow._image_paths(image_data)
        pixmap=QPixmap(paths[0]) if paths else QPixmap()
        if not pixmap.isNull():
            label.setText("")
            label.setPixmap(pixmap.scaled(max(120,label.width()-8),max(100,label.height()-8),Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation))
            label.setToolTip((f"共 {len(paths)} 张图片\n" if len(paths)>1 else "")+"\n".join(paths))
        else:
            label.setPixmap(QPixmap()); label.setText(empty_text); label.setToolTip("")

    @staticmethod
    def _read_live_evidence(item):
        path=Path(str(item.get("last_evidence_path") or ""))
        if path.is_file():
            try:
                evidence=json.loads(path.read_text(encoding="utf-8"))
                return evidence.get("inputs") or {},evidence.get("output") or {},evidence
            except Exception as exc:
                return item.get("last_inputs") or {},item.get("last_output") or {},{"load_error":str(exc)}
        return item.get("last_inputs") or {},item.get("last_output") or {},{}

    def _show_module_detail(self,row,column=0):
        if not (0 <= row < len(self._module_rows)): return
        item=self._module_rows[row]
        self._selected_module_id=str(item.get("module_id") or "")
        inputs,output,evidence=self._read_live_evidence(item)
        call_time=item.get("last_call") or evidence.get("time") or "-"
        invoked="已调用真实权重" if item.get("model_invoked") else "算法/模块调用（无模型权重）"
        self.current_module_title.setText(
            f"当前模型/模块：{item.get('flow_stage','-')} · {item.get('name') or item.get('module_id')}\n"
            f"{item.get('live_status','-')} · {invoked} · {item.get('last_elapsed_ms','-')} ms · {call_time}"
        )
        input_view={
            "module_id":item.get("module_id"),"implementation":item.get("implementation"),
            "model_path":item.get("model_path") or "","call_time":call_time,"inputs":inputs,
        }
        output_view={
            "status":item.get("live_status"),"model_invoked":bool(item.get("model_invoked")),
            "elapsed_ms":item.get("last_elapsed_ms"),"note":evidence.get("note") or "","output":output,
            "evidence_path":item.get("last_evidence_path") or "",
        }
        self.module_input_data.setPlainText(json.dumps(input_view,ensure_ascii=False,indent=2,default=str))
        self.module_output_data.setPlainText(json.dumps(output_view,ensure_ascii=False,indent=2,default=str))
        self._set_io_preview(self.module_input_preview,inputs,"本次输入不包含可预览图片\n详细数据见下方")
        self._set_io_preview(self.module_output_preview,output,"本次输出不包含可预览图片\n详细数据见下方")

    def _refresh_modules(self,modules):
        all_rows=list(modules or [])
        # Only live calls from the current controller run are visible. Example
        # assets can feed demo mode but must not appear as completed I/O early.
        self._module_rows=[item for item in all_rows if int(item.get("call_count",0) or 0)>0]
        self.module_table.setRowCount(0)
        models=sum(1 for item in self._module_rows if item.get("model_invoked"))
        if self._module_rows:
            self.module_summary.setText(f"已随流程显示 {len(self._module_rows)} 个已执行模型/模块｜真实权重已调用 {models}｜未到步骤不显示")
        else:
            self.module_summary.setText("等待流程调用模型/模块；未执行步骤不提前显示")
        for item in self._module_rows:
            row=self.module_table.rowCount(); self.module_table.insertRow(row)
            implementation=item.get("implementation","")
            if item.get("model_path"): implementation=f"{implementation}\n{item.get('model_path')}"
            proof=item.get("last_evidence_path") or "-"
            values=[item.get("flow_stage"),item.get("name"),item.get("category"),implementation,item.get("live_status"),item.get("call_count"),item.get("last_elapsed_ms") if item.get("last_elapsed_ms") is not None else "-",proof]
            for column,value in enumerate(values):
                cell=QTableWidgetItem(str(value)); self.module_table.setItem(row,column,cell)
            status=str(item.get("live_status"))
            color="#9df0d3" if status=="SUCCESS" else ("#ffd27a" if status in {"WAITING","FALLBACK"} else "#ff9b9b")
            self.module_table.item(row,4).setForeground(QColor(color))
        if not self._module_rows:
            self._selected_module_id=""; self._latest_module_id=""
            self.current_module_title.setText("当前模型/模块：等待流程")
            self.module_input_data.clear(); self.module_output_data.clear()
            self._set_io_preview(self.module_input_preview,{},"该步骤执行后显示输入图片")
            self._set_io_preview(self.module_output_preview,{},"该步骤执行后显示输出图片")
            return
        latest=max(self._module_rows,key=lambda item:str(item.get("last_call") or ""))
        latest_id=str(latest.get("module_id") or "")
        target_id=latest_id if latest_id!=self._latest_module_id else (self._selected_module_id or latest_id)
        self._latest_module_id=latest_id
        row_index=next((i for i,item in enumerate(self._module_rows) if str(item.get("module_id") or "")==target_id),len(self._module_rows)-1)
        self.module_table.selectRow(row_index); self._show_module_detail(row_index)

    def _review_corners(self, image_points, corner_ids, result_tag="review"):
        return run_corner_review(image_points, corner_ids, parent=self, result_tag=result_tag)

    def _run_device_check_on_start(self):
        """开始时自动跑完第1步设备检查；跑完即 DONE，不等「执行下一步」。"""
        if self.controller.finished:
            return
        if self.controller.current_step[0] != "DEVICE_CHECK":
            return
        self._step_busy=True
        self._running_stage=int(self.STEP_STAGE.get("DEVICE_CHECK",1))
        self._update_step_controls()
        self._refresh()
        QApplication.processEvents()
        try:
            self.controller.execute_next()
        finally:
            self._step_busy=False
            self._running_stage=0
            self._present_next_plc_command()
            self._refresh_flow_chain(self.controller.snapshot())

    def _start(self):
        # 内部入口：开会话并立刻跑 DEVICE_CHECK。界面上由「执行下一步 / 自动运行」触发。
        if self._step_busy: return
        try:
            self.controller.set_debug_inputs(self.debug_inputs)
            self.controller.start()
            self._refresh()
            self._run_device_check_on_start()
            self._refresh()
        except Exception as e:
            self._step_busy=False
            QMessageBox.critical(self,"启动失败",str(e)); self._refresh()
    def _begin_step_run(self):
        """锁定本次点击要跑的流程格；execute_next 前进 step_index 后仍用它画 RUN。"""
        snap = self.controller.snapshot()
        self._running_stage = int(self.STEP_STAGE.get(snap.get("step_code"), 0) or 0)
        self._step_busy = True
        self._update_step_controls()
        self._refresh_flow_chain(snap)

    def _end_step_run(self):
        self._step_busy = False
        self._running_stage = 0
        self._present_next_plc_command()
        self._refresh_flow_chain(self.controller.snapshot())

    # 硬失败后若强制跳过：多数后续步会缺数据再失败；仅联调/演示有意义。
    _CONTINUE_ADVICE = {
        "PARALLEL_LOCATE": "插取或雷达失败后，后续角点/放置通常会继续失败；现场应停在本步重试，联调才可强行跳过。",
        "PICK_ONLY": "插取失败则货物位姿不可靠，后续放置不安全；建议停止并重试插取。",
        "RADAR_TO_CAMERA": "缺少粗角点则相机搜索目标无效，后续识别/放置大概率失败。",
        "CAPTURE_CORNERS": "未拍到角点图则无法精定位，后续步骤不可靠。",
        "CORNER_RECOGNITION": "角点识别失败则无精定位，放置目标不可信。",
        "CAMERA_TO_WORLD": "无 WORLD 角点则空间规划/放置会缺输入。",
        "INITIAL_SPACE_PLAN": "空间规划失败则没有放置目标，后续放置无法正常进行。",
        "NEIGHBOR_POSE": "临近姿态缺失可能影响避障/微调，放置风险升高。",
        "TARGET_CONFIRM": "目标未确认则放置点不可信，不建议继续到放置。",
        "PRE_PLACE_MONITOR": "放置前监控失败，可继续但缺少异常预警。",
        "PLACE": "放置失败则货未到位，后续偏差/反馈无意义；建议停止排查。",
        "POST_PLACE_BOTTOM": "放置后监控失败，本轮可视为未完整闭环。",
        "POST_REGION": "区域偏差失败，本轮反馈不完整。",
        "POST_CARGO_OFFSET": "货偏失败，本轮反馈不完整。",
        "FEEDBACK": "反馈失败不影响机械回程，但本轮数据不完整。",
        "RETURN": "回程失败时龙门架可能不在安全位，下一轮很危险；建议停止。",
    }

    def _continue_advice_for(self, code: str) -> str:
        return self._CONTINUE_ADVICE.get(
            str(code or ""),
            "跳过本步后后续结果可能不可靠；现场优先停止并重试本步。",
        )

    def _ask_continue_after_failure(self, exc: Exception) -> bool:
        """阻塞性失败弹窗：继续=跳过本步前进；停止=停在本步可重试。"""
        code = ""
        name = ""
        try:
            code, name = self.controller.current_step
        except Exception:
            pass
        advice = self._continue_advice_for(code)
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Critical)
        box.setWindowTitle("步骤失败 — 是否继续？")
        box.setText(f"步骤失败：{name or code or '未知步骤'}")
        box.setInformativeText(
            f"{exc}\n\n"
            f"{advice}\n\n"
            "「继续」= 跳过本步进入下一步（不重跑失败逻辑）。\n"
            "「停止」= 停在本步，可稍后重试；自动运行会暂停。"
        )
        cont = box.addButton("继续", QMessageBox.ButtonRole.AcceptRole)
        stop = box.addButton("停止", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(stop)
        box.exec()
        return box.clickedButton() is cont

    def _handle_step_failure(self, exc: Exception, *, from_auto: bool = False) -> None:
        self._refresh()
        if self._ask_continue_after_failure(exc):
            try:
                self.controller.continue_after_step_failure(str(exc))
            except Exception as cont_exc:
                QMessageBox.critical(self, "无法继续", str(cont_exc))
                if from_auto and self.timer.isActive():
                    self.timer.stop()
                    self.auto_btn.setText("自动运行")
                    self._sync_auto_push_policy()
            self._refresh()
            return
        if from_auto and self.timer.isActive():
            self.timer.stop()
            self.auto_btn.setText("自动运行")
            self._sync_auto_push_policy()
        self._refresh()

    def _next(self):
        if self._plc_blocks_flow():
            QMessageBox.information(self, "请先处理 PLC 坐标", "请先在 PLC 弹窗中确认下发，或跳过并关闭，再执行下一步。")
            self._show_plc_dialog()
            return
        if self._step_busy: return
        self._begin_step_run()
        try:
            self.controller.set_debug_inputs(self.debug_inputs)
            if not self.controller.running:
                self._step_busy=False
                self._running_stage=0
                self._update_step_controls()
                self._start()
                return
            self._refresh()
            QApplication.processEvents()
            self.controller.execute_next(); self._refresh()
        except Exception as e:
            self._handle_step_failure(e, from_auto=False)
        finally:
            self._end_step_run()

    def _auto(self):
        if self.timer.isActive():
            self.timer.stop()
            self.auto_btn.setText("自动运行")
        else:
            if self._plc_blocks_flow():
                QMessageBox.information(self, "请先处理 PLC 坐标", "请先在 PLC 弹窗中确认下发或关闭，再自动运行。")
                self._show_plc_dialog()
                return
            if not self.controller.running:
                self._start()
            self.timer.start()
            self.auto_btn.setText("暂停")
        self._sync_auto_push_policy()

    def _auto_tick(self):
        if self._plc_blocks_flow():
            self._update_step_controls()
            return
        if self._step_busy: return
        self._begin_step_run()
        try:
            if not self.controller.running:
                self._step_busy=False
                self._running_stage=0
                self._update_step_controls()
                self._start()
                return
            self._refresh()
            QApplication.processEvents()
            self.controller.execute_next(); self._refresh()
            if self.controller.finished: self.timer.stop(); self.auto_btn.setText("自动运行"); self._sync_auto_push_policy()
        except Exception as e:
            self._handle_step_failure(e, from_auto=True)
        finally:
            self._end_step_run()

    def _debug(self):
        d=DebugInputDialog(self.debug_inputs,self)
        if d.exec(): self.debug_inputs=d.values(); self.controller.set_debug_inputs(self.debug_inputs)

    def _ensure_plc_dialog(self):
        if self.plc_dialog is not None:
            return self.plc_dialog
        self.plc_dialog = PlcMotionDialog(
            self,
            on_confirm_push=self._confirm_push_plc,
        )
        self.plc_dialog.resolved.connect(self._on_plc_dialog_resolved)
        return self.plc_dialog

    def _show_plc_dialog(self):
        dlg = self._ensure_plc_dialog()
        if dlg.is_blocking() or self._latest_plc_cmd:
            dlg.raise_()
            dlg.activateWindow()
            if not dlg.isVisible():
                dlg.show()
            return
        dlg.show()
        dlg.raise_()
        dlg.activateWindow()

    def _plc_blocks_flow(self) -> bool:
        dlg = self.plc_dialog
        return bool(dlg is not None and dlg.is_blocking())

    def _reset(self):
        self.timer.stop()
        self.auto_btn.setText("自动运行")
        self.controller.reset(keep_plan=True)
        self._step_busy=False
        self._running_stage=0
        self._latest_plc_cmd=None
        self._plc_cmd_queue.clear()
        self._latest_plc_batch=[]
        if self.plc_dialog is not None:
            self.plc_dialog.clear_command()
        self._sync_auto_push_policy()
        self._refresh()

    def _sync_auto_push_policy(self):
        # 连续自动运行 + 主界面勾选 → 自动推送；逐步模式永不自动推
        enabled = bool(self.auto_push_cb.isChecked() and self.timer.isActive())
        self.controller.set_auto_push_plc(enabled)

    def _enqueue_plc_command(self, cmd: dict):
        self._plc_cmd_queue.append(dict(cmd or {}))
        # 步骤执行中会套动画 event loop；此时弹窗容易嵌进死循环。等本步结束后再弹出。
        if not self._step_busy:
            self._present_next_plc_command()

    def _on_plc_dialog_resolved(self, skip_remaining: bool = False):
        if skip_remaining:
            self._plc_cmd_queue.clear()
            self._latest_plc_cmd = None
            self._latest_plc_batch = []
            self._update_step_controls()
            return
        self._present_next_plc_command()
        self._update_step_controls()

    def _present_next_plc_command(self):
        dlg = self._ensure_plc_dialog()
        if dlg.is_blocking():
            return
        if not self._plc_cmd_queue:
            return
        step = self._plc_cmd_queue[0].get("step")
        batch = []
        while self._plc_cmd_queue and self._plc_cmd_queue[0].get("step") == step:
            batch.append(self._plc_cmd_queue.pop(0))
        self._latest_plc_batch = batch
        self._latest_plc_cmd = batch[0]
        auto = bool(self.auto_push_cb.isChecked() and self.timer.isActive())
        push = None
        if auto:
            push = self._push_plc_batch(batch)
            status = "SUCCESS" if push.get("success") else "FAILED"
            self.controller.twin.add_message(
                "PLC_PUSH", status, push.get("message", ""), {"count": len(batch), **(push or {})}
            )
            if not push.get("success") and self.timer.isActive():
                self.timer.stop()
                self.auto_btn.setText("自动运行")
                self._sync_auto_push_policy()
        dlg.apply_batch(batch, auto_pushed=auto, push_result=push)
        self._update_step_controls()

    def _push_plc_batch(self, batch: list) -> dict:
        total = len(batch or [])
        if total <= 0:
            return {"success": False, "message": "没有可下发的运动坐标"}
        from PySide6.QtGui import QGuiApplication, QCursor
        from PySide6.QtCore import Qt as _Qt

        QGuiApplication.setOverrideCursor(QCursor(_Qt.CursorShape.WaitCursor))
        try:
            for index, cmd in enumerate(batch, 1):
                self.controller.plc_publisher.last_command = cmd
                result = self.controller.push_last_plc_command()
                if not result.get("success"):
                    return {
                        "success": False,
                        "message": f"第 {index}/{total} 段下发失败：{result.get('message') or '未知错误'}",
                    }
            channel = (result or {}).get("channel") or "local_plc"
            return {"success": True, "message": f"已按顺序下发 {total} 段（{channel}）", "channel": channel}
        finally:
            QGuiApplication.restoreOverrideCursor()

    def _confirm_push_plc(self):
        if self.plc_dialog is not None and getattr(self.plc_dialog, "_latest_cmds", None):
            batch = [dict(item) for item in self.plc_dialog._latest_cmds if item]
            if batch:
                self._latest_plc_batch = batch
                self._latest_plc_cmd = batch[0]
                return self._push_plc_batch(batch)
        batch = self._latest_plc_batch or ([self._latest_plc_cmd] if self._latest_plc_cmd else [])
        if not batch:
            return {"success": False, "message": "当前没有可下发的运动坐标"}
        return self._push_plc_batch(batch)

    def _apply_motion_snapshot(self,snapshot,motion):
        # Parallel 3.1 runs in a worker thread.  Its queued motion signal can be
        # delivered after fork pickup has already attached the cargo; never let
        # that older snapshot overwrite the latest CARRIED state in QML.
        latest=self.controller.snapshot()
        robot_id=motion.get("robot_id","")
        latest_task=(((latest.get("twin") or {}).get("devices") or {}).get(robot_id) or {}).get("task","")
        self._refresh(latest)
        if str(latest_task) != str(motion.get("task","")):
            return
        self.phase.setText(f"场景动作 · {motion.get('robot_id','')} · {motion.get('task','')}")
        if self.isVisible():
            loop=QEventLoop(self); QTimer.singleShot(460,loop.quit); loop.exec()

    def _refresh_flow_chain(self,s):
        # DONE=已跑完；RUN=本点击正在执行的那一格；WAIT=等待用户点「执行下一步」或尚未轮到
        # 注意：execute_next 结束后 step_code 已指向「下一格」，不能用它+busy 画 RUN，否则会误亮下一格。
        lab = self._is_lab_ui()
        stages = self.LAB_FLOW_STAGES if lab else self.FLOW_STAGES
        nodes = self.lab_flow_nodes if lab else self.flow_nodes
        if s.get("finished"):
            current_stage = 5 if lab else 13
        elif not s.get("running") and not s.get("results"):
            current_stage=0
        else:
            current_stage=self.STEP_STAGE.get(s.get("step_code"),1)
            if lab and s.get("step_code") == "DONE":
                current_stage = 5
        skipped=set() if lab else ({4,5,6,7} if not s.get("first_round") else set())
        if lab and not s.get("first_round"):
            skipped = {1}  # 后续轮跳过设备检查
        running=int(self._running_stage or 0) if self._step_busy else 0
        for (number,title_text),node in zip(stages, nodes):
            if number in skipped:
                status="SKIP"; bg="#172535"; border="#344b5e"; color="#7990a1"
            elif running and number == running:
                status="RUN"; bg="#0b5794"; border="#61c7ff"; color="#ffffff"
            elif current_stage and number < current_stage:
                status="DONE"; bg="#123e37"; border="#28a985"; color="#9df0d3"
            else:
                status="WAIT"; bg="#10243a"; border="#294d6d"; color="#8faabd"
            node.setText(f"{number}. {title_text}\n{status}")
            node.setStyleSheet(f"background:{bg};border:1px solid {border};border-radius:5px;color:{color};font-size:11px;font-weight:600;padding:3px")
        self._update_step_controls()

    def _update_step_controls(self):
        """步骤 RUN 或 PLC 弹窗待处理时，禁止点「执行下一步」。"""
        busy = bool(self._step_busy or self._plc_blocks_flow() or self.controller.finished)
        if hasattr(self, "next_btn"):
            self.next_btn.setEnabled(not busy)
        if hasattr(self, "auto_btn") and not self.timer.isActive():
            # 自动运行进行中仍可点「暂停」；未在自动跑时若步骤忙则禁止启动自动
            self.auto_btn.setEnabled(not self._step_busy and not self.controller.finished)

    def _refresh(self,s=None):
        s=s or self.controller.snapshot(); t=s["twin"]; self.bridge.update_state(t); self._refresh_flow_chain(s)
        lab = self._is_lab_ui()
        mode_text = ("实验室首轮建图" if s["first_round"] else "实验室 B 列循环") if lab else ("首轮建图" if s["first_round"] else "循环装载（跳过雷达/角点）")
        self.phase.setText(f"第 {s['round']} 轮 · {mode_text} · {s['step_name']}"); self.progress.setText(f"{s['completed']} / {s['total']}")
        rd=s.get("round_data") or {}; cargo=t.get("cargo") or {}; p=cargo.get("pose") or {}; truck=t.get("truck") or {}; target=truck.get("current_target") or {}; tp=target.get("final_world_pose") or {}; fb=t.get("feedback_compensation_mm") or {}
        hole=((rd.get("pick_result") or {}).get("hole_result") or {})
        if lab:
            self._fill_kv(self.cargo_table,[
                ("货物编号",cargo.get("instance_id") or cargo.get("cargo_code","-")),
                ("名称",cargo.get("cargo_name","-")),
                ("状态",cargo.get("status","-")),
                ("尺寸 mm",f"{cargo.get('length_mm','-')} × {cargo.get('width_mm','-')} × {cargo.get('height_mm','-')}"),
                ("当前 WORLD XYZ",f"{p.get('x_mm','-')}, {p.get('y_mm','-')}, {p.get('z_mm','-')}"),
                ("插孔识别", (rd.get("pick_result") or {}).get("message","等待第2步")),
                ("雷达粗定位", (rd.get("radar_result") or {}).get("message","等待第2步")),
                ("相机精定位", (rd.get("lab_corner_shell") or {}).get("message","等待第3步")),
                ("放货前监测", (rd.get("lab_pre_place_monitor") or {}).get("message","首件无需放前监测")),
                ("放货后检测", (rd.get("lab_place_verify") or {}).get("message","请手动放到当前 B 区后执行检测")),
            ])
            self._refresh_lab_sense_panels(rd, hole, truck)
        else:
            self._fill_kv(self.cargo_table,[
                ("货物编号",cargo.get("instance_id") or cargo.get("cargo_code","-")),("名称",cargo.get("cargo_name","-")),("状态",cargo.get("status","-")),
                ("随动绑定",cargo.get("attached_to") or "已释放 / 未插取"),
                ("尺寸 mm",f"{cargo.get('length_mm','-')} × {cargo.get('width_mm','-')} × {cargo.get('height_mm','-')}"),("当前 WORLD XYZ",f"{p.get('x_mm','-')}, {p.get('y_mm','-')}, {p.get('z_mm','-')}"),
                ("目标盲码",target.get("blind_code","-")),("目标 WORLD XYZ",f"{tp.get('x_mm','-')}, {tp.get('y_mm','-')}, {tp.get('z_mm','-')}"),
                ("步骤2 偏移%",(rd.get("pre_pick_offset") or {}).get("overhang_percent","-")),
                ("放置前动态监测",json.dumps({k:(rd.get("pre_place_monitor") or {}).get(k) for k in ("corner_xyz_m","offset_deg")},ensure_ascii=False)),
                ("放置后底层托盘",json.dumps({k:(rd.get("post_place_bottom_pallet") or {}).get(k) for k in ("corner_xyz_m","offset_deg")},ensure_ascii=False)),
                ("10.1 区域偏差",json.dumps(((rd.get("post_region") or {}).get("region_deviation") or {}).get("next_pallet_compensation_world_mm",{}),ensure_ascii=False)),
                ("10.2 货物/托盘偏移mm",(rd.get("post_cargo_offset") or {}).get("offset_distance_mm","-")),
                ("下一托盘 WORLD 反馈",json.dumps(fb,ensure_ascii=False)),
            ])
            trp=truck.get("pose") or {}; geom=truck.get("camera_board_geometry") or {}
            self._fill_kv(self.truck_table,[
                ("车辆编号",truck.get("truck_id")),("坐标系","world / mm；X右 Y车辆前进 Z向上"),("车辆 XYZ",f"{trp.get('x_mm')}, {trp.get('y_mm')}, {trp.get('z_mm')}"),
                ("车辆 RPY",f"{trp.get('roll_deg')}, {trp.get('pitch_deg')}, {trp.get('yaw_deg')}"),("车板类型",truck.get("board_mode")),
                ("板型判断来源",geom.get("decision_source","-")),("相机最终角点数",len(truck.get("corners") or {})),("高低差 mm",geom.get("height_difference_mm","-")),
                ("可用区域",len(truck.get("available") or [])),("已占用区域",len(truck.get("occupied") or [])),("借位规划",json.dumps(geom.get("borrow_plan"),ensure_ascii=False)),
            ])
            corners=truck.get("corners") or {}; self.corner_table.setRowCount(0)
            for pid in sorted(corners,key=lambda x:int(x[1:]) if x[1:].isdigit() else 999):
                q=corners[pid]; r=self.corner_table.rowCount(); self.corner_table.insertRow(r)
                for c,val in enumerate([pid,q.get("x"),q.get("y"),q.get("z")]): self.corner_table.setItem(r,c,QTableWidgetItem(str(val)))

            self.device_table.setRowCount(0); rows=[]
            compact=self.width()<1500
            for column in (1,2,5): self.device_table.setColumnHidden(column,compact)
            for did,d in (t.get("devices") or {}).items(): rows.append((did,d.get("kind"),"-",d.get("status"),d.get("pose") or {},d.get("task","-")))
            for cid,c in (t.get("cameras") or {}).items(): rows.append((cid,"arm_camera",c.get("parent_robot_id"),c.get("status"),c.get("world_pose") or {},c.get("task","-")))
            for did,kind,parent,status,pp,task in rows:
                r=self.device_table.rowCount(); self.device_table.insertRow(r); vals=[did,kind,parent,status,f"{pp.get('x_mm','-'):.0f}, {pp.get('y_mm','-'):.0f}, {pp.get('z_mm','-'):.0f}" if pp else "-",f"{pp.get('roll_deg','-')}, {pp.get('pitch_deg','-')}, {pp.get('yaw_deg','-')}",task]
                for c,val in enumerate(vals): self.device_table.setItem(r,c,QTableWidgetItem(str(val)))

            self.space_table.setRowCount(0)
            for reg in truck.get("regions") or []:
                center=reg.get("center_world_xyz_mm") or [0,0,0]; support=f"{reg.get('support_height_compensation_mm',0):.1f}mm" if reg.get("requires_support_block") else "-"
                vals=[reg.get("blind_code") or reg.get("region_id"),reg.get("section"),reg.get("column"),f"{center[0]:.0f},{center[1]:.0f},{center[2]:.0f}",support,reg.get("status"),reg.get("cargo_id") or "-"]
                r=self.space_table.rowCount(); self.space_table.insertRow(r)
                for c,val in enumerate(vals): self.space_table.setItem(r,c,QTableWidgetItem(str(val)))
            self.comp.setText("8.2当前补偿："+json.dumps((rd.get("neighbor_pose") or {}).get("compensation_world_mm",{}),ensure_ascii=False)+"\n历史反馈："+json.dumps(fb,ensure_ascii=False)+"\n剩余空间："+json.dumps(truck.get("remaining_space") or {},ensure_ascii=False))

            par=t.get("parallel") or {}; self.parallel_label.setText(f"3.1插取：{par.get('pick',{}).get('status')} ｜ 3.2雷达：{par.get('radar',{}).get('status')}")
            cal=s.get("calibration") or {}
            intr="参考/占位" if "PLACEHOLDER" in str(cal.get("intrinsic_parameter_status","")).upper() else str(cal.get("intrinsic_parameter_status","-"))
            extr="占位" if "PLACEHOLDER" in str(cal.get("coordinate_parameter_status","")).upper() else str(cal.get("coordinate_parameter_status","-"))
            profile=get_run_profile()
            self.cal_label.setText(
                f"运行模式：{profile.get('title')}（{profile.get('id')}）｜"
                f"设备：{str(s.get('device_mode') or 'mock').upper()}｜"
                f"标定：内参={intr}｜外参={extr}｜运动相机=实时位姿×安装外参"
            )
            self.cal_label.setToolTip(json.dumps(cal,ensure_ascii=False,indent=2))
            db=s.get("database") or {}; db_location=str(db.get("location") or db.get("path") or "-")
            db_name=("MySQL" if str(db.get("backend")).lower()=="mysql" else "SQLite")
            self.database_label.setText(f"车辆数据库：{db_name}｜装载会话：{db.get('session_id') or '未开始'}")
            self.database_label.setToolTip(db_location)
            self._refresh_modules(s.get("module_evidence") or [])
            msgs=t.get("messages") or []; self.message.setPlainText("\n".join(f"[{m.get('time')}] {m.get('source')} {m.get('status')}  {m.get('message')}" for m in msgs[-30:]))

        self._refresh_ext_devices(s)
        msgs=t.get("messages") or []
        alarm=t.get("alarm"); self.logs.setPlainText(("ALARM: "+str(alarm)+"\n\n" if alarm else "")+"\n".join(f"[{m.get('time')}] {m.get('source')} {m.get('status')} {m.get('message')}" for m in msgs))

    def _refresh_lab_sense_panels(self, rd: dict, hole: dict, truck: dict) -> None:
        """实验室右侧：相机图、插孔坐标、角点 WORLD（含来源）。"""
        place = rd.get("lab_place_verify") or {}
        pre_monitor = rd.get("lab_pre_place_monitor") or {}
        pick = rd.get("pick_result") or {}
        corner = rd.get("lab_corner_shell") or {}
        image = str(place.get("image_path") or "")
        caption_default = "执行后显示相机照片（第2步插孔 / 第3步角点 / 第4步俯拍）"
        if image:
            caption_default = "当前 B 区：手动放货后检测"
        else:
            image = str(pre_monitor.get("image_path") or "")
            if image:
                caption_default = "上一 B 区：下一件放货前监测"
            else:
                image = str(corner.get("image_path") or "")
                if image:
                    caption_default = "第3步：角点精定位拍照"
                else:
                    image = str(pick.get("image_path") or "")
                    if image:
                        caption_default = "第2步：托盘插孔识别图"
                    else:
                        image = self._first_image(hole) or self._first_image(pick.get("capture")) or ""
        if image and Path(image).is_file():
            pix = QPixmap(image)
            if not pix.isNull():
                scaled = pix.scaled(
                    max(240, self.lab_camera_preview.width() - 8),
                    max(180, self.lab_camera_preview.height() - 8),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
                self.lab_camera_preview.setPixmap(scaled)
                self.lab_camera_preview.setToolTip(f"{caption_default}\n{image}")
            else:
                self.lab_camera_preview.setText(f"无法加载图片：\n{image}")
        else:
            self.lab_camera_preview.setPixmap(QPixmap())
            self.lab_camera_preview.setText(caption_default)

        self.lab_hole_table.setRowCount(0)
        for side, label in (("left", "左插孔"), ("right", "右插孔")):
            world = hole.get(f"{side}_world_xyz_mm")
            cam = hole.get(f"{side}_xyz_mm")
            xyz = world if world is not None else cam
            if xyz is None:
                continue
            frame = "WORLD" if world is not None else "相机"
            r = self.lab_hole_table.rowCount()
            self.lab_hole_table.insertRow(r)
            vals = [f"{label}({frame})", f"{float(xyz[0]):.1f}", f"{float(xyz[1]):.1f}", f"{float(xyz[2]):.1f}"]
            for c, val in enumerate(vals):
                self.lab_hole_table.setItem(r, c, QTableWidgetItem(str(val)))

        final_corners = corner.get("final_world_corners") or {}
        camera_corners = corner.get("camera_world_corners") or {}
        radar = rd.get("radar_result") or {}
        radar_corners = radar.get("world_points") or {}
        corners = final_corners or camera_corners or radar_corners or truck.get("corners") or {}
        ids = list(corners.keys()) or list(radar.get("corner_ids") or [])
        if not ids:
            ids = sorted(corners.keys(), key=lambda x: int(x[1:]) if str(x)[1:].isdigit() else 999)
        else:
            ids = sorted(ids, key=lambda x: int(str(x)[1:]) if str(x)[1:].isdigit() else 999)
        self.lab_radar_table.setRowCount(0)
        for pid in ids:
            q = corners.get(pid) or {}
            source = q.get("source")
            if not source:
                if pid in camera_corners:
                    source = "camera_yolo"
                elif pid in radar_corners:
                    source = "lidar"
                else:
                    source = "-"
            xyz = q
            if "final_world_xyz_mm" in q and isinstance(q.get("final_world_xyz_mm"), (list, tuple)):
                xyz = {"x": q["final_world_xyz_mm"][0], "y": q["final_world_xyz_mm"][1], "z": q["final_world_xyz_mm"][2]}
            r = self.lab_radar_table.rowCount()
            self.lab_radar_table.insertRow(r)
            vals = [pid, xyz.get("x"), xyz.get("y"), xyz.get("z"), source]
            for c, val in enumerate(vals):
                text = f"{float(val):.1f}" if isinstance(val, (int, float)) else str(val)
                self.lab_radar_table.setItem(r, c, QTableWidgetItem(text))

    def _refresh_ext_devices(self,s):
        """顶栏紧凑状态：以 DEVICE_CHECK 探测结果为准。"""
        twin=(s or {}).get("twin") or {}
        devices=twin.get("devices") or {}
        cameras=twin.get("cameras") or {}
        check=((s.get("round_data") or {}).get("device_check") or {}).get("devices") or {}
        titles={"PLC":"PLC","RADAR":"雷达","CAM_PICK":"相机"}
        rows=[
            ("PLC","控制器",devices.get("PLC") or {}),
            ("RADAR","雷达",devices.get("RADAR") or {}),
            ("CAM_PICK","臂上相机",devices.get("CAM_PICK") or cameras.get("CAM_PICK") or {}),
        ]
        for device_id,kind,meta in rows:
            detail=check.get(device_id) or {}
            checked=bool(detail)
            if checked:
                ok=bool(detail.get("success"))
                label="已连接" if ok else "未连接"
                note=str(detail.get("message") or meta.get("task") or kind)
            else:
                raw=str(meta.get("status") or "UNKNOWN").upper()
                if raw in {"ONLINE","CONNECTED","SUCCESS"}:
                    ok=True; label="已连接"
                elif raw in {"FAILED","OFFLINE","ERROR"}:
                    ok=False; label="未连接"
                else:
                    ok=None; label="未检测"
                note=str(meta.get("task") or kind)
            if ok is True:
                color="#3dcea0"; mark="●"
            elif ok is False:
                color="#ff7b7b"; mark="●"
            else:
                color="#8faabd"; mark="○"
            badge=(getattr(self,"device_status_badges",{}) or {}).get(device_id)
            if badge is not None:
                badge.setText(f"{mark} {titles.get(device_id,device_id)} {label}")
                badge.setToolTip(note)
                badge.setStyleSheet(f"color:{color};font-size:12px;font-weight:600;padding:2px 0")
