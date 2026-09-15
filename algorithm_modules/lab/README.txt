实验室算法包（已迁入本项目，禁止引用项目外路径）

目录
----
lidar_lab/     实验室雷达点云几何算法 + lidar_extrinsic.json
cam_yolo_lab/  实验室 YOLO 角点 + camera_extrinsic.json + weights/best.pt

开关
----
统一在：config/feature_switches.py

前端顶部「运行模式」三选一：
  完全模拟(sim) / 实验室(lab) / 完全真实(field)

也可单独改：
  USE_LAB_LIDAR_ALGO / USE_LAB_CAMERA_ALGO
但会被「运行模式」覆盖；推荐直接用界面切换。

说明
----
- 采集设备（Livox / D435i / PLC）仍走项目原适配器
- 仅处理算法在 Facade / Flow 处分流
- 实验室相机路径需要采集元数据中的 plc_pose（拍摄时已自动写入）
