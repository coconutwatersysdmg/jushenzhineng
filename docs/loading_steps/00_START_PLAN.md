# 装载计划就绪（开始时，非流程格）

点「执行下一步」且会话尚未开始时，会先开会话并展开计划；**不是**流程链路上的一格。

## 输入

`data/loading_plan.json` 的 `items`。

## 处理

1. 按 `quantity` 展开队列  
2. 默认待命位姿，`status=STAGED`  
3. 写入孪生库存，当前件标 `CURRENT`  

## 输出

任务队列 + 孪生 `cargo_inventory`（`STAGED`）。
