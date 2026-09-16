# 第 2 步：货-托偏移（PRE_PICK_OFFSET）

用臂上相机拍一张待命货物的彩色图，识别蓝托盘与白货边界，判断货物相对托盘是否偏出太多、是否允许插取。

## 输入

| 来源 | 说明 |
|------|------|
| 相机 RGB | tag=`pre_pick_offset`，相机角色默认 `CAM_PICK` |
| 当前货 | 队列当前件（存证据用） |
| 门限 | `pallet_cargo_offset.max_overhang_percent`（默认 5%） |

取图规则：

- **完全模拟**：可用调试图 / 自动示例图 `examples/demo_pre_pick_offset.jpg`
- **实验室 / 真实**：忽略调试预填示例图，强制 D435i 实拍；成功则存 `workdir/camera_captures/`

本步**不移动机械臂**。

## 处理

1. 拍照拿 `image_path`
2. 算法找蓝托盘 + 白货边界，算 `overhang_percent`
3. 与门限比较 → `should_fork`（可否插取）
4. 结果写入运行日志 + PLC message `PALLET_OFFSET_PRE_PICK`
5. **无论拍照/识别成功与否，都继续下一步**（总体结果可标 `warning`）

## 输出（看哪里）

- **运行日志 / 报警**：`PRE_PICK_OFFSET` 的 INFO / FAILED  
- **总体结果**：本步 `status=success|warning`，`data` 含完整字段  
- 关键字段：`capture_success`、`analysis_success`、`image_path`、`overhang_percent`、`should_fork`、`message`
