实验室算法包（已迁入本项目，禁止引用项目外路径）

目录
----
lidar_lab/     实验室雷达点云几何算法 + lidar_extrinsic.json
cam_yolo_lab/  实验室 YOLO 角点 + camera_extrinsic.json + weights/best.pt

开关
----
统一在：config/feature_switches.py

  USE_LAB_LIDAR_ALGO   = False  # True=实验室雷达；False=现场 PointNet++
  USE_LAB_CAMERA_ALGO  = False  # True=实验室 YOLO WORLD；False=现场 corner_service.pt

默认均为 False（现场算法）。改开关后需重启软件。

说明
----
- 采集设备（Livox / D435i / PLC）仍走项目原适配器
- 仅处理算法在 Facade / Flow 处分流
- 实验室相机路径需要采集元数据中的 plc_pose（拍摄时已自动写入）
