# 装载计划就绪

把装载清单展开成待装队列，把每件货放到车尾待命位，供后续轮次逐件取货。


## 输入

`data/loading_plan.json` 的 `items`。

## 处理

1. 按 `quantity` 展开队列  
2. 默认待命位姿，`status=STAGED`  
3. 写入孪生库存，当前件标 `CURRENT`  

## 输出

任务队列 + 孪生 `cargo_inventory`（`STAGED`）。
