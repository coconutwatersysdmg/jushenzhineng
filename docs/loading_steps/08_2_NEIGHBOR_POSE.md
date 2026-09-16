# 第 8.2 步：临近托盘姿态（NEIGHBOR_POSE）

机械臂移到即将放货位置附近，拍一张已放托盘的彩色图，估附近托盘姿态偏差，用来微调本轮放置。

首轮与后续轮次都执行。

## 输入

| 来源 | 说明 |
|------|------|
| 空间 peek 目标 | `space.peek_next_target` |
| 已占用区域 | 无邻居时补偿为 0，可不移动 |
| 相机 RGB | tag=`neighbor_pose` |

## 处理

1. 若尚无已放托盘 → 补偿全 0，跳过运动  
2. 否则龙门架换到目标侧观测位（可多段：升高 / 纵移 / 换侧 / 下降）  
3. 拍照，分析临近托盘姿态 → `compensation_world_mm`  
4. 写入 `round_data.neighbor_pose`、`tentative_target`  

## 输出

- 临近补偿量（dx/dy/dz/dyaw）  
- PLC 运动弹窗：有邻居时为多段路径  
- PLC message：`NEIGHBOR_PALLET_POSE`  

当前视觉权重可为外部/离线测量接口（见证据 note）。
