# 第 11 步：反馈 PLC / 更新空间（FEEDBACK）

把本轮测到的「相对车板偏多少」「货在托盘上偏多少」回传给控制侧，并更新车上哪些格子已占用、下一托还能放哪。

## 输入

| 来源 | 说明 |
|------|------|
| 10.1 区域偏差 | WORLD 补偿 |
| 10.2 货托偏移 | 相机局部横向量（单独回传） |
| 当前目标 | `occupy_current` |

本步**不移动机械臂**。

## 处理

1. 由区域偏差生成下一托盘 WORLD 反馈，写入 `SpaceManager` 持久反馈  
2. 占用当前区域，刷新 available / occupied  
3. PLC message `PLACEMENT_FEEDBACK`：两类补偿一并上报  

## 输出

- 孪生反馈补偿、空间表更新  
- `next_pallet_world_compensation_mm`  
- `cargo_on_pallet_camera_horizontal_compensation_mm`  

完成后才进入第 12 步返回。
