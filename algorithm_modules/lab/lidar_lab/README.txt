实验室雷达点云处理算法（精简独立版）

用途
----
从实验室原上位机中单独提取的雷达车板点云处理模块。
核心角点算法保持原文件不变；不包含 UI、相机、PLC、数据库和演示数据。
雷达到世界坐标系的固定外参单独保存在 lidar_extrinsic.json 中。

安装依赖
--------
pip install -r requirements.txt

运行
----
只在终端显示结果：
python run.py input.pcd

同时保存 JSON：
python run.py input.pcd -o result.json

输出
----
只输出原软件实际使用的内容：
P1、P2、P3、P4：经过 lidar_extrinsic.json 转换后的世界坐标，单位 mm
length_mm：车板长度，单位 mm
width_mm：车板宽度，单位 mm

输出格式：
{
  "P1": [X, Y, Z],
  "P2": [X, Y, Z],
  "P3": [X, Y, Z],
  "P4": [X, Y, Z],
  "length_mm": 0.0,
  "width_mm": 0.0
}

外参
----
lidar_extrinsic.json 保存：
R_world_from_lidar：3×3 旋转矩阵
T_world_from_lidar_mm：3×1 平移向量，单位 mm

以后重新标定雷达时，只需要替换该 JSON 中的外参参数，不需要修改核心算法代码。

说明
----
输入 PCD 只读取，不删除、不移动、不覆盖。
