# norm_viewer 段错误修复计划

## 问题概述

norm_viewer 程序运行时出现段错误，GDB 分析显示问题定位到 X11 事件队列处理。根本原因是 PCLVisualizer 全局对象在程序启动时初始化，尝试连接 X11 显示服务器，与 ROS2 多线程环境产生冲突。

## 修复目标

将 PCLVisualizer 全局变量改为智能指针，在 ROS2 节点初始化后创建，避免在程序启动时就初始化 X11 相关资源。

---

## 实施步骤

### 步骤 1：修改全局变量声明

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_viewer.cpp`

**修改内容**：
- 将第32行的全局变量：
  ```cpp
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  ```
  改为智能指针：
  ```cpp
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  ```

- 保留第33行的全局变量：
  ```cpp
  bool dataRefresh = false;
  ```

---

### 步骤 2：修改 cloudViewer 函数

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_viewer.cpp`

**修改内容**：
- 在 `cloudViewer` 函数开始处（第51行之前），添加智能指针检查：
  ```cpp
  if (!viewer) {
      return;
  }
  ```

- 将所有 `viewer.` 调用改为 `viewer->`，共需修改约20处：
  - `viewer.removeAllShapes()` → `viewer->removeAllShapes()`
  - `viewer.removeAllPointClouds()` → `viewer->removeAllPointClouds()`
  - `viewer.setBackgroundColor(...)` → `viewer->setBackgroundColor(...)`
  - `viewer.addPointCloud(...)` → `viewer->addPointCloud(...)`
  - `viewer.setPointCloudRenderingProperties(...)` → `viewer->setPointCloudRenderingProperties(...)`
  - `viewer.addPointCloudNormals(...)` → `viewer->addPointCloudNormals(...)`
  - `viewer.addPolygonMesh(...)` → `viewer->addPolygonMesh(...)`
  - `viewer.addLine(...)` → `viewer->addLine(...)`
  - `viewer.addCoordinateSystem(...)` → `viewer->addCoordinateSystem(...)`
  - `viewer.setCameraPosition(...)` → `viewer->setCameraPosition(...)`
  - `viewer.setRepresentationToSurfaceForAllActors()` → `viewer->setRepresentationToSurfaceForAllActors()`
  - `viewer.spinOnce()` → `viewer->spinOnce()`

---

### 步骤 3：修改 main 函数

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_viewer.cpp`

**修改内容**：
- 在 ROS2 节点创建后（第108行之后），创建 PCLVisualizer 智能指针：
  ```cpp
  // 在 rclcpp::init(argc, argv); 之后
  // 在 auto node = rclcpp::Node::make_shared("norm_viewer"); 之后
  viewer.reset(new pcl::visualization::PCLVisualizer("PCL Viewer"));
  ```

- 在 while 循环条件中添加智能指针检查（第135行）：
  ```cpp
  while(rclcpp::ok() && viewer && !viewer->wasStopped())
  ```

---

### 步骤 4：添加必要的头文件

**文件**：`/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_viewer.cpp`

**检查内容**：
- 确认已包含 `<memory>` 头文件（第19行已包含）
- 如未包含，需添加：
  ```cpp
  #include <memory>
  ```

---

## 修改后的代码结构概览

```cpp
// 全局变量（修改后）
std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
bool dataRefresh = false;

// cloudViewer 函数（修改后）
void cloudViewer()
{
    if (!viewer) {  // 添加智能指针检查
        return;
    }
    
    if (dataRefresh) {
        viewer->removeAllShapes();  // 使用 ->
        viewer->removeAllPointClouds();
        viewer->setBackgroundColor(0.0, 0.0, 0.3);
        // ... 其他 viewer-> 调用 ...
        dataRefresh = false;
    }
    viewer->spinOnce();  // 使用 ->
}

// main 函数（修改后）
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("norm_viewer");
    
    // ROS2 节点初始化后创建 viewer
    viewer.reset(new pcl::visualization::PCLVisualizer("PCL Viewer"));
    
    // ... 订阅设置 ...
    
    viewer->setPosition(1000, 500);  // 使用 ->
    
    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok() && viewer && !viewer->wasStopped()) {  // 添加 viewer 检查
        rclcpp::spin_some(node);
        cloudViewer();
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
```

---

## 重新构建和测试步骤

### 1. 清理旧的构建文件

```bash
cd /home/bosch/vChisel_ros2_ws
rm -rf build/norm_calc install/norm_calc
```

### 2. 重新构建 norm_calc 包

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select norm_calc
```

### 3. 加载环境

```bash
source install/setup.bash
```

### 4. 测试 norm_viewer

```bash
# 方法1：单独测试（需要先运行 norm_calc_server）
ros2 run norm_calc norm_viewer

# 方法2：使用 launch 文件测试
ros2 launch norm_calc norm_calc_launch.py
```

### 5. 验证修复

- 检查程序是否正常启动，无段错误
- 检查可视化窗口是否正常显示
- 检查点云数据是否正常更新

---

## 关键文件列表

1. `/home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_viewer.cpp` - 主要修改文件
2. `/home/bosch/vChisel_ros2_ws/src/norm_calc/CMakeLists.txt` - 检查构建配置（无需修改）

---

## 风险和缓解措施

### 风险1：智能指针未正确初始化
- **缓解措施**：在 `cloudViewer` 函数中添加智能指针检查，避免空指针访问

### 风险2：viewer 对象生命周期管理问题
- **缓解措施**：使用 `std::shared_ptr` 自动管理生命周期，程序退出时自动释放

### 风险3：多线程访问冲突
- **缓解措施**：ROS2 的 `spin_some` 和 `viewer->spinOnce` 在同一个线程中调用，避免并发访问

---

## 成功标准

1. 程序能够正常启动，无段错误
2. 可视化窗口正常打开
3. 点云数据能够正常显示和更新
4. 程序能够正常退出，无资源泄漏
