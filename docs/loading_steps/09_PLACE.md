# 第 9 步：放货（PLACE）

机械臂携货升到安全高度、换到目标作业侧，从车外侧伸出并下降，把托盘货物放到已锁定的车板目标位置后松货。

## 输入

| 来源 | 说明 |
|------|------|
| `placement_target` | 最终 WORLD 位姿 |
| 当前绑定货物 | `PICK_ARM` 随动载荷 |

## 处理

1. `SET_PLACE_POINT` 登记最终放置点  
2. 携货换到目标作业侧（安全高度多段）  
3. 从外侧伸出 → 下降到目标 → `place` 放货  
4. 货物从臂上拆绑，状态 `PLACED`  

本步 PLC 运动通常为 **多段路径**（升高 / 纵移 / 换侧 / 伸出 / 下降），弹窗一次确认按序下发。

## 输出

- `round_data.place_result`、`placed_cargo`  
- 孪生：货物落在目标位，臂任务 `CARGO_RELEASED`  
- 选侧：`selected_side` LEFT/RIGHT  

失败：缺目标、PLC 未确认放置点、下降失败等会阻断。
