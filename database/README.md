# 车辆中心装载数据库

当前运行代码默认直连本机 MySQL 8.3：`127.0.0.1:3306/jushenzhineng`。应用使用仅授权该业务库的 `jushen_app` 账号；MySQL 由机器上的 `MySQL83` Windows 服务管理，应用不再启动项目独立实例。SQLite `runtime/vehicle_loading.db` 保留用于测试和回退。

## 数据中心关系

```text
vehicle（车辆主档）
├─ cargo_inventory（独立货物库存：可用/预留/装载中/已装载）
│    └─ cargo_image（货物图片路径）
└─ loading_session（某辆车的一次装载任务）
       ├─ cargo_item（本车本次任务的全部货物）
       ├─ truck_board_snapshot（相机车板/角点快照）
       ├─ loading_region（车头到车尾的 A/B 两列区域）
       ├─ placement_record（货物最终放置结果）
       ├─ workflow_step_run（步骤 2～12 结果）
       ├─ module_run（模型/模块输入输出）
       │    └─ module_io_asset（输入/输出图片、深度图、点云、JSON路径）
       ├─ plc_message（PLC和流程消息）
       └─ device_motion（机械臂轨迹点）
```

所有业务数据都必须先确定 `truck_id`，再确定 `session_id`。同一辆车可拥有多次装载会话；不同会话的区域、货物、模型记录不会混在一起。

## 核心约束

- `vehicle.truck_id` 是车辆主键；当前演示车辆为 `TRUCK-01`，接入业务系统后可替换为车辆编号或 VIN。
- `cargo_inventory.inventory_id` 是每件实物库存的永久编号；`inventory_status` 使用 `AVAILABLE/RESERVED/LOADING/LOADED`，`is_loaded` 直接表示是否已装载。
- 装载完成时库存记录写入 `loaded_truck_id/loaded_session_id/loaded_region_id/loaded_at`；中途重置时，未装货物会释放回 `AVAILABLE`。
- `loading_session.session_id` 标识一次完整装载过程。
- 货物使用 `(session_id, cargo_id)` 唯一确定。
- 放置区域使用 `(session_id, region_id)` 唯一确定，保存 A/B 列、排号、WORLD 中心和四角坐标。
- 同一货物在一次会话中最多有一条最终放置记录。
- 图片、深度图、PCD 和结果 JSON 保存路径与字段来源，不直接存 BLOB。
- `cargo_image` 支持一件库存多张图，保存输入/输出、RGB/深度/结果图类型、字段路径、图片路径和对应模型调用；`primary_image_path` 保存库存主图。
- `.pt/.pth` 是否真正执行由 `module_run.model_invoked` 明确记录，联调回退不会伪装成模型推理。

## 面向车辆的查询视图

- `v_vehicle_loading_overview`：某辆车每次装载的货物总数、已放数量、放置次数和模型调用次数。
- `v_vehicle_region_status`：某辆车某次装载的 A/B 区域及占用状态。
- `v_vehicle_module_io`：从车辆追溯到每次模型/模块的输入输出与证据路径。
- `v_cargo_inventory_status`：查看库存状态、是否已装载、装到哪辆车/哪个区域以及关联图片数。

查看当前配置的数据库（默认 MySQL）：

```powershell
.\venv\Scripts\python.exe database\inspect_vehicle_database.py
```

指定车辆：

```powershell
.\venv\Scripts\python.exe database\inspect_vehicle_database.py --truck-id TRUCK-01
```

显式查看旧 SQLite：

```powershell
.\venv\Scripts\python.exe database\inspect_vehicle_database.py --driver sqlite --db runtime\vehicle_loading.db
```

## 正式数据库迁移

SQLite 版适合当前单机数字孪生。MySQL 8 建表脚本为 `database/vehicle_centered_schema_mysql.sql`，使用 InnoDB、`utf8mb4`、原生 JSON、毫秒级时间和外键约束。

从 SQLite 迁移到本机 MySQL：

```powershell
.\venv\Scripts\python.exe database\migrate_sqlite_to_mysql.py --user root
```

脚本会交互读取密码，默认创建数据库 `jushenzhineng`，不会把密码写入项目。目标库已有业务数据时默认停止；只有显式添加 `--replace` 才会清空目标业务表后重新迁移。

迁移完成后，脚本会核对全部 13 张表的记录数、外键数量和 4 个查询视图。图片和模型结果文件仍保存路径，不写入数据库 BLOB。正式部署时可将路径替换为对象存储或共享文件服务 URI。
