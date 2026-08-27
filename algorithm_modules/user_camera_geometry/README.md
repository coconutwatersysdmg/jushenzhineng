# 用户相机几何模块

- `camera_geometry_module_hybrid_original.py`：用户本轮上传的原始8点混合版本，原样保存用于核对。
- `camera_geometry_module_camera_only_4_6.py`：v8 入口，最终4/6角点全部来自相机，不再用雷达P5/P6构造最终板几何。

实际系统实现位于：

- `services/sensor_calibration_service.py`
- `services/camera_corner_world_service.py`
- `services/camera_board_geometry_service.py`
