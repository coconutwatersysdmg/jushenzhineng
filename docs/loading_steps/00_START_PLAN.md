# 装载计划就绪（开始时，非流程格）

点「开始」时执行，**不是**「执行下一步」的一格。

## 输入

`data/loading_plan.json` 的 `items`（字段见历史说明）。

## 处理

1. 按 `quantity` 展开队列  
2. 默认待命位姿，`status=STAGED`  
3. 写入孪生库存，当前件标 `CURRENT`  

## 输出

任务队列 + 孪生 `cargo_inventory`（`STAGED`）。
