# 第 7 步：角点转 WORLD（CAMERA_TO_WORLD）

**仅首轮**。

## 输入

| 来源 | 说明 |
|------|------|
| 像素点 | 第 6 步 `image_points` |
| 深度 | 第 5 步 depth + 采样窗口 |
| 动态外参 | 拍摄瞬间相机 WORLD 位姿（臂位姿 × 安装外参） |

本步**不移动机械臂**。

## 处理

1. `(u,v) + depth + T_world_camera` → 各角点 WORLD XYZ  
2. 实验室路径可直接用已算好的世界坐标  
3. 写入 `round_data.camera_world_corners`，更新孪生车辆角点  

## 输出

- `world_points`：P1…Pn 的 WORLD 坐标  
- 孪生场景：车板角点可视化  
- 模块证据：`CAMERA_WORLD`  

后续 8.1 规划**只使用本步相机最终 WORLD 角点**（不用雷达粗点做板型判断）。
