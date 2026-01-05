# 法向点位计算优化计划

## 问题概述

当前系统在水泥未加工平面的情况下，只能计算出 8 个左右法向点位，且点位不在网格中心。主要原因是：

1. **宽松模式未生效**：每次服务调用都重置状态，导致宽松模式的逻辑永远不会执行
2. **宽松模式参数错误**：RELAXED_NORM_TH 配置为 0.90（与严格模式相同），没有真正宽松
3. **严格模式阈值过严**：法向阈值 0.90（cos 16°）导致平面情况下很多点位被过滤

## 目标

1. 优先严格模式，如果严格模式计算的点位不够 12 个，自动加入宽松模式
2. 确保平面情况下能找到足够的点位（至少 12 个）
3. 保持网格中心偏好的优先级

---

## 解决方案

### 方案 1：修正宽松模式参数

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/config/norm_calc_params.yaml`

**修改内容**：
```yaml
# 严格模式 (第一次尝试：只打完美的点)
STRICT_NORM_TH: 0.90       # cos(16度)，要求非常垂直
STRICT_HOLE_DIST: 0.05     # 离空洞至少 5cm
STRICT_CURV_TH: 0.04       # 表面必须非常平整

# 宽松模式 (第二次尝试：允许次优点)
RELAXED_NORM_TH: 0.87      # cos(30度)，允许斜坡 ← 从 0.90 改为 0.87
RELAXED_HOLE_DIST: 0.035   # 离空洞至少 3.5cm
RELAXED_CURV_TH: 0.10      # 允许表面稍微粗糙
```

**说明**：
- 将 RELAXED_NORM_TH 从 0.90 改为 0.87（cos 30°）
- 允许更大的角度偏差（从 16° 增加到 30°）
- 增加可选点位数量

---

### 方案 2：修改状态机重置策略

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_calc_server.cpp`

**当前问题**：
```cpp
// 【修改点】每次请求都重置状态
for (auto &grid : grids_)
  grid->reset();
```
每次服务调用都重置所有网格状态，导致永远不会进入宽松模式。

**修改方案**：
```cpp
// 只重置 STATE_COMPLETED 的网格，保留 STATE_SKIPPED_ONCE 的状态
// 这样第一轮失败的网格会在第二轮尝试宽松模式
int pending_count = 0;
for (auto &grid : grids_) {
  if (grid->getState() == ChiselBox::STATE_COMPLETED) {
    grid->reset();
  } else if (grid->getState() == ChiselBox::STATE_PENDING) {
    pending_count++;
  }
}

// 如果所有网格都是 STATE_PENDING（第一次运行），则全部重置
if (pending_count == grids_.size()) {
  for (auto &grid : grids_)
    grid->reset();
}

global_obstacles_->clear();
```

**说明**：
- 已完成的网格（STATE_COMPLETED）重置，允许重新规划
- 跳过一次的网格（STATE_SKIPPED_ONCE）保留状态，下次尝试宽松模式
- 如果是第一次运行（所有网格都是 PENDING），则全部重置

---

### 方案 3：添加点位数量检查和自动降级

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_calc_server.cpp`

**修改位置**：在 `handleService` 函数中，遍历网格之后

**添加逻辑**：
```cpp
// 网格决策
int plan_count = 0;
int skipped_count = 0;

for (auto &grid : grids_) {
  // ... 现有代码 ...

  if (grid->findBestPoint(roi_cloud, global_obstacles_, target)) {
    // ... 现有代码 ...
    plan_count++;
  } else {
    skipped_count++;
  }
}

// 如果严格模式点位不足 12 个，强制所有跳过的网格尝试宽松模式
if (plan_count < 12 && skipped_count > 0) {
  RCLCPP_WARN(this->get_logger(),
              "Strict mode only found %d points (< 12), trying relaxed mode for skipped grids...",
              plan_count);

  for (auto &grid : grids_) {
    if (grid->getState() == ChiselBox::STATE_SKIPPED_ONCE) {
      // 强制进入宽松模式
      grid->setState(ChiselBox::STATE_SKIPPED_ONCE); // 保持状态，触发宽松模式

      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr roi_cloud(
          new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      extractGridROI(processed_cloud, grid, roi_cloud);

      pcl::PointXYZRGBNormal target;
      if (grid->findBestPoint(roi_cloud, global_obstacles_, target)) {
        // ... 现有代码 ...
        plan_count++;
      }
    }
  }
}
```

**说明**：
- 检查严格模式找到的点位数量
- 如果不足 12 个，自动触发宽松模式
- 对所有跳过的网格重新尝试宽松模式

---

## 实施步骤

### 步骤 1：修正配置文件参数

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/config/norm_calc_params.yaml`

**修改**：
- 第 33 行：`RELAXED_NORM_TH: 0.90` → `RELAXED_NORM_TH: 0.87`

---

### 步骤 2：修改状态机重置逻辑

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_calc_server.cpp`

**修改位置**：`handleService` 函数中，约第 263-267 行

**修改内容**：
- 替换现有的全部重置逻辑
- 实现部分重置策略（只重置 COMPLETED 状态）

---

### 步骤 3：添加点位数量检查

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_calc_server.cpp`

**修改位置**：`handleService` 函数中，网格遍历循环之后

**添加内容**：
- 统计规划点位数量
- 检查是否少于 12 个
- 如果不足，触发宽松模式

---

### 步骤 4：重新构建和测试

**构建命令**：
```bash
cd /home/bosch/vChisel_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select norm_calc
```

**测试步骤**：
1. 加载环境：`source install/setup.bash`
2. 启动系统：`ros2 launch snap_7 system_launch.py`
3. 触发点云计算
4. 检查日志输出，确认点位数量
5. 验证点位是否在网格中心附近

---

## 关键文件列表

1. `/home/bosch/vChisel_ros2_ws/src/norm_calc/config/norm_calc_params.yaml` - 修正宽松模式参数
2. `/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_calc_server.cpp` - 修改状态机重置和点位检查
3. `/home/bosch/vChisel_ros2_ws/src/norm_calc/src/chisel_box.cpp` - 状态机逻辑（无需修改）

---

## 预期效果

### 修改前
- 平面情况：约 8 个点位
- 点位分布：可能偏离网格中心
- 宽松模式：永远不会执行

### 修改后
- 平面情况：至少 12 个点位（严格模式不足时自动降级）
- 点位分布：保持网格中心偏好
- 宽松模式：自动生效，增加可选点位

---

## 风险和缓解措施

### 风险 1：宽松模式点位质量下降
- **缓解措施**：严格模式优先，宽松模式只在点位不足时使用
- **缓解措施**：保持其他约束（曲率、避障）不变

### 风险 2：状态机逻辑复杂化
- **缓解措施**：添加详细日志，记录每个网格的状态变化
- **缓解措施**：保留原有的重置逻辑作为后备

### 风险 3：点位数量检查可能影响性能
- **缓解措施**：只在点位不足时触发宽松模式，不影响正常情况
- **缓解措施**：宽松模式只针对跳过的网格，不是全部重新计算

---

## 成功标准

1. ✅ 平面情况下能找到至少 12 个点位
2. ✅ 严格模式优先，宽松模式自动降级
3. ✅ 点位保持网格中心偏好
4. ✅ 日志输出清晰，显示点位数量和模式切换
5. ✅ 系统性能无明显下降

---

## 调试建议

### 添加日志输出

在关键位置添加日志：

**norm_calc_server.cpp**：
```cpp
RCLCPP_INFO(this->get_logger(),
            "Grid planning: %d completed, %d skipped, %d unreachable",
            completed_count, skipped_count, unreachable_count);

if (plan_count < 12) {
  RCLCPP_WARN(this->get_logger(),
              "Only %d points found (< 12), triggering relaxed mode...",
              plan_count);
}
```

**chisel_box.cpp**：
```cpp
RCLCPP_DEBUG(this->get_logger(),
             "Grid[%d,%d]: State=%d, Points=%d, BestScore=%.3f",
             row_, col_, state_, cloud->size(), best_score);
```

---

## 后续优化建议

1. **自适应阈值**：根据点位数量动态调整阈值
2. **网格优先级**：优先处理中心网格，再处理边缘网格
3. **质量验证**：添加点位质量评估，过滤低质量点位
4. **可视化增强**：在 RViz 中显示网格状态和点位评分