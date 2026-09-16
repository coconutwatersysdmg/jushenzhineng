# 第 6 步：角点识别 / 人工审核（CORNER_RECOGNITION）

在车尾、车头拍到的彩色图上用模型标出车板角点像素位置，必要时弹出界面让人手工改准。

**仅首轮**。

## 输入

| 来源 | 说明 |
|------|------|
| 第 5 步 RGB（/深度） | 按角点或按组 |
| 算法 | 现场 `corner_service.pt`，或实验室相机世界模块 |
| 审核开关 | `corner_review.enabled` |

本步**不移动机械臂**。

## 处理

1. `.pt` / 实验室模块识别像素角点  
2. 可选弹窗人工审核矫正（`CornerReviewDialog`）  
3. 结果写入 `round_data.corner_recognition`  

## 输出

- `image_points`：各 Pn 的像素坐标  
- 审核后可带 `corrected` 标记与结果图  
- 模块证据面板：CORNER_YOLO / 实验室路径输入输出  

缺图时：模拟模式可中心点兜底；真机模式应失败提示。
