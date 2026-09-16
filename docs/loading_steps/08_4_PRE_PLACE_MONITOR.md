# 第 8.4 步：放置前动态监测（PRE_PLACE_MONITOR）

## 输入

| 来源 | 说明 |
|------|------|
| `placement_target` | 第 8.3 步 |
| 相机 RGB-D | tag=`pre_place_monitor` |
| `camera.json` | 动态监测相机内参等 |

## 处理

1. 龙门架到放置侧观测位（多段运动）  
2. 采 RGB + Z16，跑动态监测模块  
3. 结果写入 `round_data.pre_place_monitor`  

## 输出

- 角点 / 偏角等监测字段（见模块输出）  
- 货物表「放置前动态监测」摘要  
- PLC 运动弹窗 + message `DYNAMIC_PRE_PLACE`  
- 模块证据：`DYNAMIC_PRE_PLACE`
