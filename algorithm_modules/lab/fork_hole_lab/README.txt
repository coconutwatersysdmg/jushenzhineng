托盘叉孔定位模块（实验室插孔识别）
================================

来源：fork_hole_final_stable.zip

对外接口：
    from algorithm_modules.lab.fork_hole_lab import locate_fork_holes

世界坐标转换由主流程完成。

---- 上游 README ----

托盘叉孔临时定位模块
====================

用途：
D435i采集RGB-D -> OpenCV识别左右两个叉孔 -> Depth对齐RGB -> 自动读取相机内参 -> 输出两个叉孔在D435i相机坐标系下的XYZ。

本模块不进行世界坐标转换，也不需要PLC位姿。

最终输出：
{
  "left_hole_xyz_mm":  [X, Y, Z],
  "right_hole_xyz_mm": [X, Y, Z]
}

单位：mm
D435i相机坐标系：
- +X：相机向右
- +Y：相机向下
- +Z：相机向前

识别失败或没有有效深度时，对应值为 null。

使用方法：
1. 安装依赖：
   pip install -r requirements.txt

2. 连接D435i后直接运行：
   python fork_hole_locator.py

也可以嵌入主程序：

from fork_hole_locator import D435iForkHoleLocator

with D435iForkHoleLocator() as locator:
    result = locator.locate_once()
    print(result)

说明：
- D435i连接后，程序通过RealSense SDK自动读取当前彩色相机内参 fx、fy、cx、cy，不需要手动填写内参。
- 程序自动把Depth对齐到RGB后再计算XYZ。
- 叉孔深度优先取孔口周围有效深度的中位数，避免把孔内部较远的后壁误当成孔口深度。
- 当前模块只负责输出相机坐标系XYZ；以后如需世界坐标，再在主系统外部使用外参进行转换。

现场稳定性处理：
- 默认不再使用固定灰度阈值；程序根据当前RGB画面的亮度分布自动计算暗孔阈值，以适应现场明暗变化。
- 候选区域必须满足“孔内明显暗于孔口周围纸箱表面”的局部对比度约束，用于过滤地面阴影和设备暗部。
- 同时使用左右孔基本同高、尺寸接近、中心间距合理等几何约束选择最终两个叉孔。
- 如需调试，也可在 D435iForkHoleLocator(dark_threshold=数值) 中手动指定阈值；正常使用建议保持默认自动模式。
