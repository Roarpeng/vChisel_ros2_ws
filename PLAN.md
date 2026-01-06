# 图像更新和点位计算优化计划

## 主要修改

### 问题1：拍照第二次视窗没有更新
**原因**：图像发布时使用了旧的相机时间戳，导致与姿态数据的时间戳不匹配
**解决**：更新图像发布时的时间戳为当前时间
**文件**：`src/norm_calc/src/norm_calc_server.cpp`

### 问题2：凸起的平面孤岛不会取凿击
**原因**：凸起检测只考虑Z轴高度差，无法检测均匀升高的平面孤岛
**解决**：改进凸起检测逻辑，结合相对高度和曲率进行判断
**文件**：`src/norm_calc/src/chisel_box.cpp`

### 问题3：点位少于8个时降低评分权重
**原因**：宽松模式只降低过滤阈值，不调整评分权重
**解决**：添加动态权重调整，当点位<8时降低权重要求
**文件**：`src/norm_calc/src/norm_calc_server.cpp`

## 关键文件

- `/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_calc_server.cpp` - 服务主逻辑
- `/home/bosch/vChisel_ros2_ws/src/norm_calc/src/chisel_box.cpp` - 点位选择算法
- `/home/bosch/vChisel_ros2_ws/src/norm_calc/config/norm_calc_params.yaml` - 参数配置

## 实施步骤

### 步骤1：修复图像时间戳问题
在 `norm_calc_server.cpp` 第249行添加：
```cpp
cv_img.header.stamp = this->get_clock()->now();
```

### 步骤2：改进凸起检测逻辑
在 `chisel_box.cpp` 中添加相对高度检测：
- 计算当前网格平均高度
- 与相邻网格平均高度比较
- 结合曲率判断是否为凸起

### 步骤3：添加动态权重调整
在 `norm_calc_server.cpp` 第504行后添加：
- 检查点位数量是否<8
- 如果<8，降低权重因子为0.5
- 如果8-12，降低权重因子为0.7
- 临时调整参数后恢复

### 步骤4：重新构建和测试
```bash
colcon build --packages-select norm_calc
```

## 预期效果

- 图像每次拍照都能正确更新
- 平面孤岛能被正确识别和凿击
- 点位数量不足时自动降低评分标准，确保至少8-12个点位