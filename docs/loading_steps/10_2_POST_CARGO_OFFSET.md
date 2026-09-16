# 第 10.2 步：托盘-货物偏差（POST_CARGO_OFFSET）

## 输入

| 来源 | 说明 |
|------|------|
| 放置后俯视 / 侧视 RGB | tag=`post_place_offset` |
| 当前货 | 算法上下文 |

## 处理

1. 到放置侧拍照  
2. 算法算货相对托盘偏移；得到相机局部横向补偿量  
3. **方向标定前不强行写入 WORLD X/Y**  
4. 写入 `round_data.post_cargo_offset`  

## 输出

- `offset_distance_mm`、`compensation_camera_horizontal_mm`  
- `applied_directly_to_world_next_target=false`  
- PLC message：`PALLET_CARGO_POST_PLACE`  
- 货物表「10.2 货物/托盘偏移mm」
