# 具身智能装载数字孪生 v8

本版本按最新流程重构：**雷达只负责首轮找车和提供4/6个粗搜索坐标；最终角点、平板/高低板判断、车板几何、两列×1.2m装载区域全部以相机最终 WORLD 角点为准。**

## 1. 当前唯一机械臂与活动相机

- `PICK_ARM`：初始位与货物同在车尾左外侧；悬挂在双侧龙门架横梁上，可沿车身两侧纵轨运动。
- `CAM_PICK`：唯一活动 RGB-D 相机，固定挂载在 `PICK_ARM` 上，所有拍摄都使用机械臂实时位姿。

用户标定包里的其他历史相机条目仍原样保留在标定 JSON 中，但不创建对应活动设备。角点拍摄由携货状态下的 `PICK_ARM + CAM_PICK` 完成。

## 2. WORLD 坐标系

使用用户标定包定义：

- X：向右
- Y：车辆前进/车长方向
- Z：向上
- 单位：mm

### 固定雷达

```text
P_world = T_world_lidar @ P_lidar
```

### 机械臂挂载相机

相机不能使用一个永久固定的 `T_world_camera`。机械臂移动时：

```text
T_world_camera(t) = T_world_robot(t) @ T_robot_camera
```

`T_world_robot(t)`：PLC/机械臂实时位姿；
`T_robot_camera`：相机安装/手眼外参。

用户提供 `coordinate_config.json` 内相机 `T_world_sensor` 标记为 placeholder，v8 只作为静态参考；实际动态转换使用拍摄瞬间的机械臂+安装外参。

## 3. 角点 JPG -> WORLD

实际只釆集车尾/车头两组，同端角点共享该组图像和位姿：

```text
CORNER_TAIL.jpg / CORNER_HEAD.jpg
CORNER_TAIL_depth.png / CORNER_HEAD_depth.png
camera_id
camera_world_pose @ capture time
```

角点 `.pt` 在 JPG 中得到 `(u,v)`。

从对齐深度图在 `(u,v)` 附近取有效深度中位数 `Z`，然后：

```text
Xc = (u-cx) * Z / fx
Yc = (v-cy) * Z / fy
Zc = Z
```

如果真实标定 distortion 非0，代码优先使用 `cv2.undistortPoints`。

最终：

```text
P_world = T_world_camera(t) @ [Xc,Yc,Zc,1]^T
```

实现：`services/sensor_calibration_service.py`、`services/camera_corner_world_service.py`。

## 4. 最新流程

### 首轮

1. 数字孪生从任务开始即显示整批货物；全部待装货物与机械臂同向预置在车尾左外侧，已装货物跨轮次保留。
2. 货物-托盘偏差分析；S/F message -> PLC。
3. 并行：
   - 3.1 机械臂找插孔并插取；
   - 3.2 雷达找车。
   四种 SUCCESS/FAILED 组合独立保存和发送；成功分支不重跑。
4. 雷达4/6粗点 -> PLC -> 携货机械臂挂载相机目标位。
5. 机械臂携货沿左外侧轨道从车尾向车头单次通过：车尾拍1张、车头拍1张 RGB-D；拍完停在车头侧继续放货。
6. `.pt` 对车尾/车头 JPG 识别对应的角点像素坐标；离线模块验证也可按 Pn 提供独立样例图。
7. `(u,v)+depth+动态T_world_camera` -> 最终4/6 WORLD角点。
8.1 **只用相机最终 WORLD角点**判断平板/高低板，按车头到车尾生成两列×1.2m区域盲码，并做高低板边界最小借位优化。
8.2 拍当前即将放置位置附近已放托盘姿态，计算当前补偿。
8.3 用持久可用空间 + 历史反馈 + 8.2补偿锁定本轮目标区域。
8.4 放置前采集 RGB + Z16 深度并运行动态监测。
9. 计算最终放置点 -> PLC ACK -> 按目标 WORLD X 相对车体中心选择物理左/右作业侧 -> 横梁在安全高度换侧 -> 放货。
10.0 放置后检测底层托盘，保存角点 XYZ、箱体底边偏角、标注图和 JSON。
10.1 拍托盘-车板划分区域偏差，计算下一托盘 WORLD补偿；同时沿车侧移动并向内伸入，观测货物两个相邻面。
10.2 拍托盘-货物偏差，计算相机局部横向补偿。
11. 两类偏差信息返回 PLC；10.1 WORLD偏差用于下一托盘放置位置；10.2局部横向量在方向标定前单独发送，不强行写入 WORLD X/Y；更新占用/可用空间。
12. 第10、11步全部完成后，机械臂先升到龙门架安全高度，沿当前侧返回车尾，再横移到左侧初始位，进入下一轮。

### 第2轮及以后

```text
2 -> 3.1 -> 8.2 -> 8.3 -> 8.4 -> 9 -> 10.0 -> 10.1 -> 10.2 -> 11 -> 12
```

明确跳过：

- 3.2 雷达找车
- 4 雷达->角点拍摄目标
- 5 角点拍照
- 6 角点识别
- 7 角点世界坐标转换
- 8.1 车板重新建图/区域重新初始化

首轮车板模型和可用空间状态持续保存在 `SpaceManager` / `DigitalTwinState`。

## 5. 平板 / 高低板相机判断

### 4点

```text
P1 ----- P2
|         |
|  FLAT   |
|         |
P3 ----- P4
```

直接作为平板。

### 6点

```text
P1 ----- P2
| 第一段  |
P3 ----- P4   <- 分界横边
| 第二段  |
P5 ----- P6
```

使用相机最终点计算第一端与第二端高度差。只有高度差超过 `high_low_height_threshold_mm` 才判定为高低板；因此 **6个雷达粗点不等于雷达已经判定高低板**。

## 6. 高低板借位

基础行长 `1200mm`。如果第一段完整排之后剩余 `R`，新增一整排所需最小借位：

```text
borrow = 1200 - R
```

只有在：

- `borrow <= max_borrow_mm`
- 借位后第二段仍保留最小配置空间

时才建立跨分界 `BORROWED_BOUNDARY` 行。目标是：**新增一整排（A/B两格）的前提下取最小借位长度**。

例如第一段 4380mm：3×1200 后剩 780mm，最小借位 420mm。

## 7. 8.2 / 10.1 尚未提供正式视觉算法

用户目前没有上传：

- 临近托盘姿态视觉模型（8.2）
- 托盘相对车板划分区域偏差视觉模型（10.1）

因此 v8 已保留真实相机、机械臂、PLC、输入输出接口和数据结构；联调模式无外部测量时明确返回 `source=demo_zero...`，不会伪装成真实模型结果。调试窗口可直接填写 `dx_mm/dy_mm/dz_mm/yaw_deg` JSON。

### 模型输入/输出

- 启动时只准备 `examples/module_io_manifest.json` 中的联调输入源，未到流程步骤的模型/模块不提前显示。
- 右侧“模型输入/输出”页只列出本轮已实际执行项，并自动选中最新调用。当前项分开显示本次输入数据/图片和输出数据/图片。
- 每次真实流程调用写入 `runtime/module_call_evidence/<module_id>/`，汇总写入 `runtime/module_call_summary.json`。只有实际执行 `.pt/.pth` 推理时才标记模型已调用，联调回退不会伪装成模型输出。
- 自动示例会调用 `pallet_hole_best.pt`、点云 `best_model.pth`、`corner_service.pt`。77MB 离线 PCD 会真实执行 PointNet++，但因为不属于当前场景的雷达→WORLD 标定，其输出只作为可视证据，不直接发送给机械臂。
- 默认角点 JPG/深度会真实调用角点识别和像素深度转换，但因示例图不属于当前车辆标定，联调场景的最终区域使用当前车辆粗点对齐，防止示例坐标把两列区域移到车外。
- 8.2、10.1 仍是无正式权重的外部测量接口；货物两面模块当前只完成图像采集。界面对此明确标注。

### 放置前 / 放置后动态监测

- 新增 `8.4 PRE_PLACE_MONITOR` 和 `10.0 POST_PLACE_BOTTOM`，均调用 `algorithm_modules/dynamic_monitoring_module`。
- 输入为 RGB、D435i Z16 深度及 `camera.json`；输出 `corner_xyz_m`、箱体底边 `offset_deg`、底板边缘 `board_edge_deg`、标注图和 JSON。
- 用户压缩包原实现假定 `HoughLinesP` 固定返回 `(N,1,4)`，并把底板斜边角度作为箱体底边偏角。接入版兼容 `(N,4)/(N,1,4)`，并分别返回两种角度。

## 8. 车辆中心装载数据库

- 默认直连机器上的 MySQL 8.3：`127.0.0.1:3306/jushenzhineng`，由 `MySQL83` Windows 服务管理。
- `vehicle` 是主实体，每次装载生成一个 `loading_session`；货物、车板快照、A/B两列区域、放置结果、流程步骤、模型输入输出、PLC消息和机械臂轨迹全部引用该会话。
- 图片、深度图、PCD和结果 JSON 不存入 BLOB，数据库保存输入/输出方向、字段路径和文件路径。
- 主界面“设备与消息”页显示当前数据库文件和装载会话号。
- 建表脚本、关系说明和查询方法见 `database/README.md`和 `database/vehicle_centered_schema.sql`。

## 9. QML 白屏修复

- 删除大写自定义属性 `L/W/H`；
- 删除 Qt 6.8 才新增的 `PrincipledMaterial.alphaMode` 依赖；
- 主窗口捕获 `QQuickWidget` QML 错误并显示具体行号；
- WORLD轴改为用户标定包的 X右/Y车辆前进/Z向上。

## 10. 测试

```bash
python test_v8_calibration.py
python test_v8_geometry.py
python test_v8_parallel_cases.py
python test_v8_flow_mock.py
python test_v8_cargo_inventory.py
```

当前测试覆盖：

- 主点像素+深度到WORLD坐标；
- 平板4点区域划分；
- 高低板6点、4380mm剩余780mm -> 借位420mm；
- 并行4种成功/失败组合；
- 成功分支重试保护；
- 首轮14步 + 后续9步；
- 后续轮次不再调用雷达和角点。

## 11. 启动

```bash
pip install -r requirements.txt
start_digital_twin.bat
```

推荐双击 `start_digital_twin.bat`：它固定使用项目 `venv`，首次冷启动会立即显示启动画面。再次双击不会被残留锁拦截，而会自动恢复、前置并提醒已经运行的主窗口。启动阶段、PID、耗时和正常退出记录在 `runtime/startup.log`；Python/Qt 原生崩溃信息记录在 `runtime/startup_fault.log`。

当 `config/system_config.py` 的 `allow_demo_device_data=True` 且步骤2没有选择 `pre_pick_offset` JPG 时，系统会自动生成并使用 `examples/demo_pre_pick_offset.jpg`。结果会明确标记 `demo_input=true` 和“联调示例图（非相机实拍）”；关闭联调数据后仍严格要求真实 `CAM_PICK` 图片。

联调模式会继续为缺失的 `P1~P6` 角点生成带对齐 `uint16` 深度的 RGB-D，并为 8.2、10.1、10.2 与两面观测补齐明确标记的示例帧。调试窗口或真实相机传入的文件始终优先；`allow_demo_device_data=false` 时不会生成任何示例输入。

主界面顶部按修订流程显示 1–12 步链路。首轮3.1插取后货物刚性绑定 `PICK_ARM`，该机械臂携货完成步骤4–9；第2轮起自动把步骤4–7标记为 `SKIP`。机械臂适配器按最大 2200mm 分段发布轨迹，三维端用 420ms 同步缓动逐段播放机械臂与载荷；场景左下角显示任务总数、待装数、已装数、当前流程和设备动作。

真实投产前必须替换用户标定包里标记为 `placeholder` 的内外参，并完成机械臂-相机真实手眼/安装外参标定。
