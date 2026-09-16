# 第 5 步：角点 RGB-D（CAPTURE_CORNERS）

**仅首轮**。左外侧单次通过：车尾拍 1 组、车头拍 1 组。

## 输入

| 来源 | 说明 |
|------|------|
| `corner_assignments` | 第 4 步规划的 TAIL / HEAD 目标位 |
| 调试图（可选） | `corner_images` / `corner_depths` |

## 处理

1. 移到车尾拍照位 → 采 RGB-D（`CORNER_TAIL`）  
2. 移到车头拍照位 → 采 RGB-D（`CORNER_HEAD`）  
3. 同一组内多个角点共用该组图像（识别在第 6 步）  
4. 写入 `corner_images`、`corner_capture_meta`、`plc_pose`  

本步 **2 段 PLC 运动**（CAPTURE_TAIL / CAPTURE_HEAD），弹窗按序下发。

## 输出

- 物理拍摄次数：通常 2  
- `round_data.corner_group_captures`  
- 每角点元数据：相机 WORLD 位姿、深度路径、拍摄时 PLC 位姿  

不是「四个角各走一圈」；四角点来自两站照片 + 后续识别。
