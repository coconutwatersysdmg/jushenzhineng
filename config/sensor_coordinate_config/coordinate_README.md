# 多传感器坐标系统说明

## 1. 当前用途
本配置包用于先把多传感器融合框架、接口和数据流写入系统。

当前外参与部分相机内参均为临时占位参数，主要用于：
- 软件开发
- 接口联调
- UI显示
- 数据流测试
- 多传感器坐标统一流程验证

**禁止用于真实叉车自动运动、精确插取或安全控制。**

真实标定完成后，只需要替换 JSON 中的外参与内参，不需要修改上层融合逻辑。

## 2. 世界坐标系
- 原点：`[0, 0, 0]`
- 单位：`mm`
- X：向右
- Y：车辆长度/前进方向
- Z：向上

所有传感器最终统一转换到 `world`。

## 3. 当前设备
1. 激光雷达 `lidar`
2. 车板角点识别相机 `corner_camera`
3. 托盘插孔识别相机 `hole_camera`
4. 叉车臂左侧辅助相机 `left_camera`
5. 叉车臂右侧辅助相机 `right_camera`
6. 叉车臂坐标系 `forklift`

共 4 个相机 + 1 个激光雷达 + 1 个叉车臂坐标系。

## 4. coordinate_config.json
保存各坐标系到世界坐标系的外参。

统一定义：
```text
P_world = T_world_sensor * P_sensor
```

齐次点：
```text
P_sensor = [x, y, z, 1]^T
```

齐次变换矩阵：
```text
T = [ R  t ]
    [ 0  1 ]
```

其中：
- `R`：3×3 旋转矩阵
- `t`：3×1 平移向量，单位 mm

计算机实现时建议统一读取：
```text
transforms.<sensor_name>.T_world_sensor
```

不要把外参写死到算法代码里。

## 5. camera_intrinsic.json
保存 4 个相机的内参。

当前统一先按：
```text
Intel RealSense D435i
1280 × 720
```

使用临时参数：
```text
fx = 904.6748
fy = 904.9058
cx = 657.9651
cy = 374.3209
depth_scale = 0.001
```

内参矩阵：
```text
K =
[ fx  0  cx ]
[  0 fy  cy ]
[  0  0   1 ]
```

当前四个相机暂时共用同一组参数，仅用于软件联调。
后续真实相机型号、分辨率或标定参数确定后，分别替换对应相机条目。

## 6. 相机数据转换流程
对于深度相机像素点 `(u, v)` 和深度 `Z`：
```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

得到：
```text
P_camera = [X, Y, Z, 1]^T
```

再转换：
```text
P_world = T_world_camera * P_camera
```

## 7. 雷达数据转换流程
```text
P_world = T_world_lidar * P_lidar
```

## 8. 叉车臂数据转换流程
```text
P_world = T_world_forklift * P_forklift
```

反向：
```text
P_forklift = inverse(T_world_forklift) * P_world
```

## 9. 系统融合建议
所有感知模块统一输出世界坐标：
```json
{
  "x_world": 0.0,
  "y_world": 0.0,
  "z_world": 0.0
}
```

建议：
```text
LiDAR ---------> World
Corner Camera -> World
Hole Camera ---> World
Left Camera ---> World
Right Camera --> World
Forklift ------> World
                   |
                   v
             Fusion / UI / Planning
```

## 10. 后续替换真实标定参数
真实标定完成后主要替换：
- `coordinate_config.json` 中各 `T_world_sensor`
- `camera_intrinsic.json` 中每个相机的 `resolution / K / distortion / depth_scale`

上层业务逻辑和融合接口保持不变。
