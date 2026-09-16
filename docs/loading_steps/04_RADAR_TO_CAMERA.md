# 第 4 步：雷达粗点 → 相机拍摄目标（RADAR_TO_CAMERA）

**仅首轮**。第 2 轮起跳过。

## 输入

| 来源 | 说明 |
|------|------|
| `round_data.radar_result` | 雷达 WORLD 粗角点 |
| 龙门左轨 X | `GANTRY_LEFT_X_MM` |

## 处理

1. 按角点 Y 分成车尾 / 车头两组  
2. 每组算一个外侧拍照位姿（左轨、对应 Y、抬高 Z、朝车侧）  
3. 下发编排命令 `SET_CORNER_CAPTURE_TARGETS`（记账；真机轴运动在第 5 步）  
4. 写入 `round_data.corner_assignments`  

本步**一般不直接驱动轴运动**，只规划目标。

## 输出

- `corner_assignments`：每个角点对应 `capture_group`（TAIL/HEAD）与 `target_robot_pose_world`  
- PLC message / ACK：目标已登记  

注意：物理拍照位是 **车尾 1 站 + 车头 1 站**，不是四个角各去一次。
