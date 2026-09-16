# 第 3 步：并行定位（PARALLEL_LOCATE）/ 仅插取（PICK_ONLY）

一边让机械臂带着相机去找托盘插孔并插取抬货，一边（仅首轮）用雷达扫车找车板粗位置；两边同时做，互不等待。

首轮为 **3.1 插取 ∥ 3.2 雷达找车**；第 2 轮起为 **PICK_ONLY（只跑 3.1）**。

## 输入

| 来源 | 说明 |
|------|------|
| 当前货 / 待命位姿 | 孪生 `cargo` + `PICK_ARM.cargo_mount_pose` |
| 相机 RGB-D | tag=`pallet_hole`，找插孔 |
| 雷达（仅首轮 3.2） | 找车 / 粗角点 |

## 处理

### 3.1 插取

1. 机械臂移到车尾待装货上方 → 插孔识别位  
2. 拍 RGB-D，算法识别左右插孔；相机坐标转 WORLD  
3. `fork_pallet` 插取；成功则货物绑定 `PICK_ARM` 并抬升带货  
4. 本步会产生 **多段 PLC 运动坐标**（到位 / 找孔 / 抬升），弹窗一次列出按序下发  

### 3.2 雷达（仅首轮）

1. 雷达采点 / 算法找车板粗角点  
2. 结果写入 `round_data.radar_result`  
3. 与 3.1 并行；四种 SUCCESS/FAILED 组合独立记日志  

## 输出

- `round_data.pick_result`、`cargo_attachment`  
- 首轮另有 `round_data.radar_result`  
- PLC 运动弹窗：本步多段路径  
- 并行面板：`pick` / `radar` 状态  

失败：插取失败会阻断；雷达失败可按策略 warning 后继续（见现场配置）。
