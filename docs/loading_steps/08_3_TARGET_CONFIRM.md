# 第 8.3 步：锁定本轮目标（TARGET_CONFIRM）

## 输入

| 来源 | 说明 |
|------|------|
| 持久可用空间 | `SpaceManager` |
| 历史反馈 | 上轮 10.1 WORLD 补偿等 |
| 8.2 临近补偿 | `neighbor_pose` |
| 当前货 | 尺寸 / 队列件 |

本步**不移动机械臂**，不重新建图。

## 处理

1. `space.confirm_target(...)` 合成最终目标区域  
2. 写入 `round_data.placement_target`  
3. 更新孪生当前目标、占用预览  

## 输出

- `placement_target`：盲码、列、`final_world_pose` 等  
- 货物表「目标盲码 / 目标 WORLD XYZ」  
- 证据：`SPACE_TARGET`  

第 9 步放货依赖本步锁定结果。
