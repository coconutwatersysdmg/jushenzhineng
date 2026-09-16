# 第 10.1 步：区域偏差 + 两面观测（POST_REGION）

在放置侧拍托盘相对车板区域的彩色图算偏置，再沿车侧移动并向内伸入，对货物两个相邻面各拍一张图做观测。

## 输入

| 来源 | 说明 |
|------|------|
| `placement_target` | 已放位置 |
| 相机 RGB | 区域偏差图；两面 `face_a` / `face_b` |

## 处理

1. 到放置侧拍托盘-车板区域偏差图并分析  
2. 两面观测：外侧拍面 A → 向内伸入拍面 B（额外运动段）  
3. 写入 `round_data.post_region`  

## 输出

- `region_deviation`：含下一托盘 WORLD 补偿建议  
- `two_face_observation`：两面采集结果（当前以采集为主）  
- PLC 运动：换侧 + 两面观测多段  
- message：`PALLET_BOARD_REGION_DEVIATION`  

10.1 的 WORLD 偏差在第 11 步用于修正下一托盘。
