# -*- coding: utf-8 -*-
"""功能开关（独立文件，后续新开关也写这里）。

用法：改下面的值后重启软件。
本文件只放“开关/模式”，PLC IP、外参数值等仍在 external_devices_config / system_config。
"""
from __future__ import annotations

# ===========================================================================
# 1) 设备模式 / Mock 数据
# ===========================================================================

# "mock" = 本地假 PLC/雷达/相机（联调）
# "real" = 真机适配器（PLC / Livox / D435i）
DEVICE_MODE = "real"

# True  = 允许联调演示数据（缺图时自动补示例、中心点等）
# False = 严格要求真实输入
# 注意：当 DEVICE_MODE="real" 时，FlowController 会强制关闭演示数据
ALLOW_DEMO_DEVICE_DATA = True

# True  = 未实现的视觉测量接口允许返回 0 占位
ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO = True

# ===========================================================================
# 2) 真机运动 / 雷达实采
# ===========================================================================

# True  = 允许向 PLC 下发真实 XYZR 运动
# False = 只记账/拦截，不写轴（安全）
ALLOW_REAL_MOTION = True

# True  = Livox 实采（忽略离线示例 PCD）
# False = 允许用文件/示例 PCD
USE_LIVE_LIDAR_CAPTURE = True

# ===========================================================================
# 3) 角点人工审核
# ===========================================================================

# True  = 步骤 6 弹出角点人工审核
CORNER_REVIEW_ENABLED = True

# True  = 联调生成的 demo 角点自动通过审核
CORNER_REVIEW_AUTO_ACCEPT_DEMO = False

# ===========================================================================
# 4) 实验室 / 现场 算法切换（两个独立开关）
# ===========================================================================

# True  = 使用 algorithm_modules/lab/lidar_lab（经典点云几何 + 实验室雷达外参）
# False = 使用原现场 PointNet++ 点云分割算法
USE_LAB_LIDAR_ALGO = False

# True  = 使用 algorithm_modules/lab/cam_yolo_lab（实验室 YOLO best.pt + 一体 WORLD）
# False = 使用原现场 corner_service.pt 像素识别 + CameraCornerWorldService
USE_LAB_CAMERA_ALGO = False

# ===========================================================================
# 后续其它开关可继续追加在本文件下方
# ===========================================================================
