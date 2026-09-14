# point_cloud_segment_module

龙门架叉车感知与定位项目中的点云分割与车板几何提取模块。

本模块完成：**原始 PCD 点云预处理 → PointNet++ 语义分割 → 底板平面拟合 → PCA/分段切片边缘提取 → 四边 RANSAC → 3D 角点、尺寸、姿态计算 → 第一作业面剩余空间规划 → JSON 输出**。

## 1. 文件夹结构

推荐目录结构如下：

```text
Algorithm_modules/
└─ point_cloud_segment_module/
   ├─ __init__.py
   ├─ inference.py
   ├─ point_cloud_pipeline.py      # 主流程；主要参数集中在文件前部 PipelineConfig
   ├─ README.md
   ├─ LICENSE
   ├─ checkpoints/
   │  └─ best_model.pth            # PointNet++ 权重
   ├─ models/
   │  ├─ __init__.py
   │  ├─ pointnet2_part_seg_msg.py
   │  └─ pointnet2_utils.py
   ├─ data/                         # 本地放原始 PCD；数据较大可不随代码分发
   │  ├─ 1-1.pcd
   │  ├─ 1-2.pcd
   │  └─ ...
   └─ output/
      └─ json/                      # 程序运行后自动生成
```

> `data/` 中的 PCD 原始数据体积较大，本压缩包不包含实际 PCD。使用时把测试/现场 PCD 放入本地 `data/` 文件夹即可。

## 2. 当前正式处理流程

```text
PCD
 ↓
ROI 裁剪
 ↓
体素降采样
 ↓
地面/主平面 RANSAC 去除
 ↓
DBSCAN 最大聚类
 ↓
统计离群点滤波
 ↓
PointNet++ 整云一次推理
 ↓
提取 label 2 / label 3
 ↓
每块底板平面 RANSAC
 ↓
PCA 建立真实朝向 U-V-N 坐标系
 ↓
分段切片寻找四条边缘候选
 ↓
四边 RANSAC + SVD 重拟合
 ↓
四边交点恢复 3D 角点
 ↓
长度 / 宽度 / 平均高度 / 倾斜角 / 偏移角
 ↓
两块底板时：label 2 第一作业面剩余空间规划
 ↓
output/json/*.json
```

当前 PointNet++ 为 **整云一次推理**：不做空间分批、不做 overlap、不做 KNN 标签恢复。

## 3. 运行环境

当前项目已在 Windows Conda `project` 环境中使用。核心依赖：

```text
Python
numpy
torch
open3d
```

另外，模型代码依赖本模块 `models/` 下的 PointNet++ 实现。

建议直接复用项目现有 `project` Conda 环境，不要随意升级 PyTorch/Open3D 后再判断模块问题。

## 4. 命令行运行

在项目根目录执行（使用便携 runtime）：

```bat
runtime\python.exe -m algorithm_modules.point_cloud_segment_module.point_cloud_pipeline ^
"algorithm_modules\point_cloud_segment_module\data" ^
--checkpoint "algorithm_modules\point_cloud_segment_module\checkpoints\best_model.pth" ^
--device cuda ^
--num-point 49152 ^
-o "algorithm_modules\point_cloud_segment_module\output"
```

也可以处理单个 PCD：

```bat
runtime\python.exe -m algorithm_modules.point_cloud_segment_module.point_cloud_pipeline ^
"algorithm_modules\point_cloud_segment_module\data\1-1.pcd" ^
--checkpoint "algorithm_modules\point_cloud_segment_module\checkpoints\best_model.pth" ^
--device cuda ^
--num-point 49152 ^
-o "algorithm_modules\point_cloud_segment_module\output"
```

## 5. 参数怎么改

所有现场常用参数都集中在 `point_cloud_pipeline.py` 最前面的：

```python
@dataclass(frozen=True)
class PipelineConfig:
```

主要参数分组如下。

### ROI

```python
roi_min = (1000.0, -10000.0, -200.0)
roi_max = (8000.0, 13000.0, 3000.0)
roi_redundancy = 0.025
```

### 预处理

```python
voxel_size = 20.0
plane_threshold = 140.0
plane_ransac_n = 3
plane_iterations = 500

dbscan_eps = 100.0
dbscan_min_points = 50

stat_neighbors = 6
stat_std_ratio = 3.0
```

### 底板平面与边缘

```python
edge_plane_distance = 100.0
edge_plane_ransac_n = 5
edge_plane_iterations = 800
edge_min_inliers = 3000

slice_width_u = 50.0
slice_width_v = 30.0
slice_min_points = 20

line_ransac_distance = 30.0
line_ransac_iterations = 300
line_min_inliers = 25
```

### 车辆方向

```python
truck_head_dir = (-1.0, 1.0, 0.0)
```

局部坐标定义：

- **U**：车长方向，`Umin` 为车头端，`Umax` 为车尾端。
- **V**：车宽方向。
- **N**：底板拟合平面法向。

### 第一作业面规划

```python
pallet_length_mm = 1200.0
pad_trigger_ratio = 0.50
```

当前规则：

- 只有同时成功识别 `label_2` 和 `label_3` 两块底板时才启用。
- 只对 `label_2`（第一作业面）计算。
- 托盘沿车长 U 方向占用 1200 mm。
- 最后剩余空间占一个托盘长度 **≥ 50%** 时，当前第一版逻辑直接判定使用垫板。
- 补偿距离：

```text
remaining_workface_compensation_mm
= pallet_length_mm - remaining_length_mm
```

例如剩余 800 mm，则补偿距离为 400 mm。

## 6. 角点与姿态定义

若检测到一块底板：

```text
P1 = Umin,Vmin
P2 = Umin,Vmax
P3 = Umax,Vmin
P4 = Umax,Vmax
```

若检测到两块底板：

```text
label_2 : P1 ~ P4
label_3 : P5 ~ P8
```

JSON 中每个角点只输出一次，不再额外重复输出总角点表。

姿态参数：

- `tilt_angle_deg`：车头 → 车尾沿长度方向相对于水平面的纵向倾斜角，单位 **°**。
- `offset_angle_deg`：俯视情况下，车长 U 正方向相对于理论 `world -Y` 的水平偏移角，单位 **°**。

两个字段都已经是角度，可以直接读取显示，无需二次换算。

## 7. JSON 输出示例

```json
{
  "label_2": {
    "corner_ids": ["P1", "P2", "P3", "P4"],
    "corners_xyz_mm": [[...], [...], [...], [...]],
    "length_mm": 4380.54,
    "width_mm": 2985.288,
    "height_mean_mm": 1414.566,
    "tilt_angle_deg": 0.393143,
    "offset_angle_deg": 2.283609,
    "loading_plan": {
      "pallet_length_mm": 1200.0,
      "normal_pallet_count": 3,
      "remaining_length_mm": 780.54,
      "remaining_ratio": 0.65045,
      "pad_trigger_ratio": 0.5,
      "pad_required": true,
      "remaining_workface_compensation_mm": 419.46,
      "final_pallet_count": 4
    }
  },
  "label_3": {
    "corner_ids": ["P5", "P6", "P7", "P8"],
    "corners_xyz_mm": [[...], [...], [...], [...]],
    "length_mm": 13000.439,
    "width_mm": 2972.443,
    "height_mean_mm": 1209.339,
    "tilt_angle_deg": 1.637939,
    "offset_angle_deg": 0.788217
  }
}
```

最终文件位置：

```text
output/json/<原PCD文件名>.json
```

## 8. 集成到上位机时

当前版本为了独立测试，会把结果写入 JSON。

真正集成到系统后，不需要依赖 `output` 文件夹。`PointCloudPipeline.process_file()` 内部已经拿到了 Python 字典 `result`，可将写 JSON 的步骤替换为直接 `return result`，交给 UI / PLC / 任务规划模块读取。

JSON 主要用于当前离线测试、核对和留档。

## 9. 注意事项

1. `best_model.pth` 必须和当前 PointNet++ 网络结构、标签定义一致。
2. 当前整云推理显存占用与输入点数有关。若 CUDA 报 OOM，说明本次整云前向显存不足；当前精简版不会自动切回旧分批方案。
3. `label_2` / `label_3` 的语义定义由当前训练数据保持一致，不要随意调换。
4. `data/` 只存测试/现场原始 PCD，不属于代码依赖，可以不随代码仓库提交。
5. 修改现场参数时，优先只改 `PipelineConfig`，不要直接在算法函数内部改硬编码。
