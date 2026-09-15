实验室 D435i 相机角点精定位模块
================================

功能
----
输入两次 D435i RGB-D 采集时对应的 PLC 位姿：
1. P3/P4 拍摄位置
2. P1/P2 拍摄位置

模块内部自动完成：
D435i RGB-D采集（Depth对齐Color）
→ YOLO一次识别两个角点
→ D435i运行时读取相机内参和depth scale
→ 5×5区域深度中值
→ 像素+深度转换为相机三维坐标
→ 使用camera_extrinsic.json和当前PLC X/Y/Z/R转换到世界坐标
→ 最终输出P1/P2/P3/P4世界坐标

最终输出
--------
result.json 只包含：
{
  "P1": [X, Y, Z],
  "P2": [X, Y, Z],
  "P3": [X, Y, Z],
  "P4": [X, Y, Z]
}
单位：mm

重要说明
--------
- 本模块不控制 PLC/叉车臂，只接收采集时的 PLC X/Y/Z/R。
- D435i 内参和深度比例不写死，连接相机后由 RealSense SDK 自动读取。
- camera_extrinsic.json 为原实验室软件的外参标定结果。
- weights/best.pt 为原实验室软件的 YOLO 权重。
- 同一张图中 u 较小者分配给 P3/P1，u 较大者分配给 P4/P2。
- YOLO 参数保持原软件：conf=0.30，imgsz=960。

安装
----
pip install -r requirements.txt

运行示例
--------
python run.py --p3p4-pose 202 132 380 -80 --p1p2-pose 234 891 380 -80

运行时程序会：
1. 提示确认已经到 P3/P4 拍摄位置，按 Enter 后采集并识别。
2. 提示确认已经到 P1/P2 拍摄位置，按 Enter 后采集并识别。
3. 输出 result.json。

作为 Python 模块集成
-------------------
from camera_world_module import D435iCamera, YoloD435iWorldLocalizer

localizer = YoloD435iWorldLocalizer()
# 可将现有系统采集到的 RGB-D frame 和 PLC pose 直接传给：
# localizer.detect_pair(frame, plc_pose, ("P3", "P4"))
# localizer.detect_pair(frame, plc_pose, ("P1", "P2"))
