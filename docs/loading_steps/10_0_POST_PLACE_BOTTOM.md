# 第 10.0 步：放置后底托检测（POST_PLACE_BOTTOM）

货放下后，机械臂再在放置侧拍一组彩色+深度图，检测底层托盘落位和姿态，作为放货结果留证。

## 输入

| 来源 | 说明 |
|------|------|
| `placement_target` | 刚放货位置 |
| 相机 RGB-D | tag=`post_place_bottom_pallet` |

## 处理

1. 再到放置侧观测位（多段运动）  
2. 采 RGB-D，跑动态监测（底托相位）  
3. 写入 `round_data.post_place_bottom_pallet`  

## 输出

- 底层托盘角点 / 偏角等  
- 货物表「放置后底层托盘」  
- PLC message：`DYNAMIC_POST_PLACE`  
- 模块证据：`DYNAMIC_POST_PLACE`
