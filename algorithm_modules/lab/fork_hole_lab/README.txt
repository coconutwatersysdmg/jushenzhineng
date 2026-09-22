托盘叉孔定位模块（实验室插孔识别）
================================

来源：fork_hole_final.zip

用途：
已对齐的 RGB-D -> OpenCV 识别左右两个叉孔 -> 输出 D435i 相机坐标系 XYZ(mm)。

本模块不进行世界坐标转换；世界坐标由主流程用相机外参完成。

对外接口：
    from algorithm_modules.lab.fork_hole_lab import locate_fork_holes

    result = locate_fork_holes(image_bgr, depth_mm, intrinsics)

输出：
{
  "left_hole_xyz_mm":  [X, Y, Z] 或 null,
  "right_hole_xyz_mm": [X, Y, Z] 或 null
}
