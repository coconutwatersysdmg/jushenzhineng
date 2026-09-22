来源：用户提供的 dynamic_monitor_lab.zip。

该模块使用 LAB 颜色分割和轮廓几何提取纸箱四角，不使用 YOLO。
系统集成时只使用它输出的图像 UV 角点；区域合格判定由
services/lab_dynamic_box_monitor_service.py 使用 D435i 对齐深度、
实时相机 WORLD 位姿和实验室规划区域 WORLD 四角完成。
