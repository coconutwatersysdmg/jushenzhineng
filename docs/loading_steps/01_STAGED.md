# 第 1 步：装载计划 → STAGED

## 输入

文件：`data/loading_plan.json`（或 `loading_plan_mixed.json`）

```json
{
  "items": [
    {
      "cargo_code": "CARGO-001",
      "cargo_name": "示例货物",
      "quantity": 3,
      "length_mm": 1200,
      "width_mm": 1000,
      "height_mm": 900,
      "pallet_reference_width_mm": 1200
    }
  ]
}
```

| 字段 | 说明 |
|------|------|
| `items` | 计划条目列表 |
| `cargo_code` | 货物编码 |
| `cargo_name` | 货物名称 |
| `quantity` | 数量；同规格可写 1 条，按数量展开 |
| `length_mm` / `width_mm` / `height_mm` | 货物外形尺寸（mm） |
| `pallet_reference_width_mm` | 托盘参考宽度（mm），给后续识别用 |

## 处理

1. 读 JSON，取 `items`
2. 按 `quantity` 展开成队列（每件一个 `instance_id`）
3. 给默认待命位姿（车尾左侧），`status = STAGED`
4. 写入数字孪生库存

不采图、不检测实物是否到位。

## 输出

- 任务队列 `queue`（展开后的每件货）
- 孪生 `cargo_inventory`：每件带 `pose`、`status=STAGED`
