# vChisel_ros2_ws - 项目上下文文档

## 项目概述

这是一个基于 ROS2 Humble 的工业自动化视觉处理系统，主要用于点云处理、与西门子PLC通信以及手眼标定。系统集成了深度相机、点云处理算法和工业控制协议，实现从视觉感知到工业执行的完整闭环。

### 核心技术栈

- **ROS2 版本**: Humble
- **构建系统**: colcon
- **编程语言**: C++14, Python 3
- **主要依赖库**:
  - PCL (Point Cloud Library) - 点云处理
  - OpenCV4 - 图像处理
  - Snap7 - 西门子PLC通信
  - pyrealsense2 - Intel RealSense相机

### 项目架构

```
vChisel_ros2_ws/
├── src/
│   ├── norm_calc/          # 点云处理包 (C++, ament_cmake)
│   ├── snap_7/             # PLC通信包 (Python, ament_python)
│   ├── hand_eye_calib/     # 手眼标定包 (C++ + Python混合, ament_cmake)
│   └── vision_opencv/      # OpenCV功能包 (ROS2官方包本地副本)
├── build/                  # 构建输出目录 (git忽略)
├── install/                # 安装目录 (git忽略)
├── log/                    # 日志目录 (git忽略)
├── Testpy/                 # 测试工具目录
│   └── Tsnap7test.py       # PLC通信测试工具
├── start_system.sh         # 系统启动脚本（带日志管理）
├── quick_start.sh          # 快速启动脚本
└── README.md               # 项目说明文档
```

## 包结构详解

### 1. norm_calc 包

**用途**: 点云数据处理和特征提取，用于凿击点位智能选择系统

**关键文件**:
- `src/norm_calc_server.cpp` - ROS2服务节点，提供点云处理服务
- `src/norm_calc.cpp` - 核心点云处理算法实现
- `src/edge_grid.cpp` - 边缘网格算法
- `src/hole_detector.cpp` - 孔洞检测算法
- `src/chisel_box.cpp` - 凿子盒区域划分
- `src/norm_viewer.cpp` - 点云可视化工具
- `src/norm_viewer_new.cpp` - 新版点云可视化工具（未编译）
- `src/image_norm_viewer.cpp` - 图像法线联合可视化（未编译）
- `config/norm_calc_params.yaml` - 参数配置文件（含详细参数说明）
- `launch/norm_calc_launch.py` - 启动文件
- `srv/NormCalcData.srv` - 服务接口定义

**可执行文件**:
- `norm_calc_server` - 点云处理服务节点
- `norm_viewer` - 点云可视化工具

**主要功能**:
- 点云预处理（直通滤波、体素下采样、统计滤波）
- 法线计算和表面分析
- 特征点提取和评分
- 边缘网格模式处理
- 深坑检测和避让
- 凿子盒区域划分和点位筛选

**依赖**:
- rclcpp, sensor_msgs, geometry_msgs
- pcl_conversions, pcl_ros, libpcl-all-dev
- opencv4, cv_bridge

### 2. snap_7 包

**用途**: 与西门子PLC通信，触发和处理点云计算，包含相机监控功能

**关键文件**:
- `snap_7/plc_client_node.py` - PLC客户端主节点
- `snap_7/camera_monitor.py` - 相机状态监控节点
- `launch/snap_7.launch.py` - 系统启动文件
- `launch/system_launch.py` - 系统完整启动文件（包含相机和可视化）
- `launch/plc_sim.launch.py` - PLC模拟启动文件
- `launch/test_plc_sim.launch.py` - PLC模拟测试启动文件

**可执行文件**:
- `snap_7_node` - PLC通信主节点
- `plc_sim_server` - PLC模拟服务器
- `camera_monitor` - 相机监控节点

**主要功能**:
- 定期轮询PLC数据块（DB）
- 监听触发值（110）启动点云处理
- 调用norm_calc服务
- 将处理结果写回PLC（支持REAL和整数类型）
- 相机进程管理和状态监控
- 物理相机连接检测（使用pyrealsense2或lsusb）
- PLC模拟模式（用于测试）

**依赖**:
- rclpy, norm_calc
- python-snap7, pyrealsense2

**PLC配置**:
- 默认IP: 192.168.1.36
- 默认DB编号: 2120
- 默认DB起始偏移: 64
- 轮询频率: 20Hz

**相机监控功能**:
- 自动检测物理相机连接
- 监控相机数据流
- 异常时自动重启相机进程
- 支持重连机制和宽限期设置

### 3. hand_eye_calib 包

**用途**: 手眼标定系统

**关键文件**:
- `src/hand_eye_calib_srv.cpp` - 标定服务节点
- `src/hand_eye_calib.cpp` - 标定算法实现
- `scripts/hand_eye_bringup.py` - Python标定启动脚本
- `scripts/data.yaml` - 标定数据配置
- `srv/HandEyeCalibData.srv` - 服务接口定义

**可执行文件**:
- `hand_eye_calib_server` - 手眼标定服务节点
- `hand_eye_bringup` - 交互式标定工具

**主要功能**:
- 相机姿态采集（通过/aruco_single/pose话题）
- 工具坐标输入（XYZABC格式）
- 手眼标定计算
- 交互式标定流程

**依赖**:
- rclcpp, rclpy, geometry_msgs, opencv4

**注意**: 该包是混合类型（ament_cmake + rclpy依赖），构建时需要确保Python模块结构正确

### 4. vision_opencv 包

**用途**: OpenCV相关功能（ROS2官方包的本地副本，版本3.1.3）

**子包**:
- **cv_bridge** - ROS图像与OpenCV图像转换
  - 支持C++和Python绑定
  - 依赖: libopencv-dev, python3-numpy, sensor_msgs

- **image_geometry** - 相机模型和几何变换
  - 提供相机标定参数与OpenCV函数的接口
  - 支持图像校正等几何操作
  - 依赖: libopencv-dev, sensor_msgs

- **opencv_tests** - OpenCV功能测试
  - Python和C++实现的cv_bridge测试
  - 包含人脸检测等示例
  - 依赖: rclpy, sensor_msgs, cv_bridge

- **vision_opencv** - 元包
  - 组织和管理vision_opencv相关包

**注意**: 此包为ROS2官方包的本地副本，包含COLCON_IGNORE标记，默认不构建

### 5. Testpy 目录

**用途**: 测试工具集合

**关键文件**:
- `Tsnap7test.py` - PLC DB数据写入测试工具
  - 支持交互式写入和读取DB2120
  - 用于验证PLC通信功能
  - 默认测试IP: 192.168.110.228

## 构建和运行

### 环境要求

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# 点云处理依赖
sudo apt install libpcl-dev \
                 ros-humble-pcl-conversions \
                 ros-humble-pcl-msgs \
                 ros-humble-pcl-ros

# 图像处理依赖
sudo apt install ros-humble-cv-bridge

# Python依赖
pip install python-snap7 pyrealsense2

# RealSense相机驱动
sudo apt install ros-humble-realsense2-camera
```

### 构建命令

```bash
# 进入工作空间
cd /home/bosch/vChisel_ros2_ws

# 加载ROS2环境
source /opt/ros/humble/setup.bash

# 构建所有包
colcon build

# 构建特定包
colcon build --packages-select norm_calc
colcon build --packages-select snap_7
colcon build --packages-select hand_eye_calib

# 构建并运行测试
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 构建时忽略vision_opencv（默认被COLCON_IGNORE标记）
colcon build --packages-skip vision_opencv
```

### 运行命令

```bash
# 加载环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 方法1: 使用一键启动脚本（带日志管理）
./start_system.sh

# 方法2: 使用快速启动脚本
./quick_start.sh

# 方法3: 手动启动系统（norm_calc + PLC客户端）
ros2 launch snap_7 snap_7.launch.py

# 方法4: 启动完整系统（包含相机、监控和可视化）
ros2 launch snap_7 system_launch.py

# 单独启动norm_calc
ros2 launch norm_calc norm_calc_launch.py

# 单独启动PLC模拟服务器
ros2 launch snap_7 plc_sim.launch.py

# 启动PLC模拟测试（模拟器 + 客户端）
ros2 launch snap_7 test_plc_sim.launch.py

# 启动相机监控节点
ros2 run snap_7 camera_monitor

# 启动手眼标定系统
ros2 run hand_eye_calib hand_eye_bringup

# 启动点云可视化工具
ros2 run norm_calc norm_viewer
```

### 测试工具

```bash
# 运行PLC通信测试
cd Testpy
python3 Tsnap7test.py
```

### 启动文件说明

| 启动文件 | 功能 | 节点 |
|---------|------|------|
| `snap_7.launch.py` | 基础系统启动 | norm_calc_server, snap_7_node |
| `system_launch.py` | 完整系统启动 | norm_calc_server, snap_7_node, camera_monitor, realsense2_camera |
| `plc_sim.launch.py` | PLC模拟服务器 | plc_sim_server |
| `test_plc_sim.launch.py` | PLC模拟测试 | plc_sim_server, snap_7_node |
| `norm_calc_launch.py` | 点云处理服务 | norm_calc_server |

**注意**: `system_launch.py` 引用了未编译的可执行文件（image_norm_viewer_node），使用前可能需要更新 CMakeLists.txt 添加相应的源文件。

## 开发约定

### 代码风格

**C++**:
- 使用 C++14 标准
- 遵循 ROS2 编码规范
- 使用 ROS2 日志系统 (RCLCPP_INFO, RCLCPP_ERROR等)
- 类名使用 PascalCase，函数名使用 camelCase

**Python**:
- 遵循 PEP 8 编码规范
- 使用类型注解（可选）
- 使用 ROS2 日志系统
- 异常处理要完善

### 包结构约定

**ament_cmake 包 (C++)**:
```
package_name/
├── CMakeLists.txt
├── package.xml
├── include/package_name/
├── src/
├── config/
├── launch/
└── srv/ (如果需要自定义服务)
```

**ament_python 包 (Python)**:
```
package_name/
├── package.xml
├── setup.py
├── setup.cfg
├── package_name/
│   ├── __init__.py
│   └── *.py
├── launch/
└── resource/package_name
```

### 服务接口定义

使用 `.srv` 文件定义服务接口，格式遵循 ROS2 规范：
```
# Request
---
# Response
```

### 参数配置

- 使用 YAML 格式的参数文件
- 参数文件位于 `config/` 目录
- 通过 launch 文件加载参数
- norm_calc_params.yaml 包含详细的参数说明和调整指南

## 常见问题和解决方案

### 构建问题

**问题**: hand_eye_calib 构建失败，提示符号链接创建失败
```
failed to create symbolic link because existing path cannot be removed: Is a directory
```
**解决**: 删除损坏的构建目录并重新构建
```bash
rm -rf build/hand_eye_calib
colcon build --packages-select hand_eye_calib
```

**问题**: system_launch.py 启动失败，提示找不到可执行文件
**错误**: 可执行文件 image_norm_viewer_node 不存在
**解决**: 该启动文件引用了未编译的源文件。可以选择：
1. 在 CMakeLists.txt 中添加 image_norm_viewer.cpp 的编译规则
2. 注释掉 system_launch.py 中对应的节点定义

### PLC通信问题

**问题**: PLC连接失败或写入被拒绝
**检查项**:
1. 确认PLC IP地址配置正确
2. 检查PLC安全设置（Full Access, PUT/GET Communication）
3. 确保DB块未启用"优化块访问"（Optimized Block Access）
4. 验证DB编号和偏移地址正确（默认: DB2120, offset 64）
5. 使用Tsnap7test.py测试基本连接

**问题**: CPU拒绝写入操作
**错误信息**: "CPU : Item not available" 或 "function refused by CPU"
**解决**:
1. 检查PLC安全设置，确保启用"Full Access"和"PUT/GET Communication"
2. 在TIA Portal中关闭DB块的"Optimized Block Access"
3. 验证DB块大小足够容纳写入的数据

### 相机问题

**问题**: 相机启动失败或频繁断开
**检查项**:
1. 确认物理相机连接（使用 `lsusb` 检查）
2. 检查相机驱动是否正确安装
3. 查看相机进程日志
4. 调整相机参数（帧率、分辨率等）
5. 使用camera_monitor节点监控相机状态

**问题**: 相机监控误报
**解决**:
1. 调整timeout_seconds参数（默认3秒）
2. 增加重连尝试次数（reconnect_attempts）
3. 确认pyrealsense2正确安装

### 点云处理问题

**问题**: 点云处理结果不理想
**调整参数**:
- `NORM_TH`: 法向量阈值（0.80-0.95）
- `SEARCH_RADIUS`: 搜索半径（0.03m）
- `SEARCH_NUM_TH`: 邻域点数阈值（100）
- `BOX_LEN`: 凿子盒边长（0.05m）
- `HEIGHT_WEIGHT`: 高度权重（3.0）
- `CURV_WEIGHT`: 曲率权重（2.0）
- `ANGLE_WEIGHT`: 角度权重（1.0）
- `MIN_DISTANCE_THRESHOLD`: 最小距离阈值（0.03m）

**参考**: norm_calc_params.yaml 文件包含详细的参数调整指南

## 日志和调试

### 日志位置

- 系统日志: `/home/bosch/logs/visual.log`（自动管理，超过5MB清空）
- 构建日志: `log/latest_build/`
- 运行时日志: 通过 `ros2 run` 或 `ros2 launch` 的标准输出

### 调试技巧

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看话题数据
ros2 topic echo /topic_name

# 查看服务列表
ros2 service list

# 调用服务
ros2 service call /norm_calc norm_calc/srv/NormCalcData "{seq: 1}"

# 查看参数
ros2 param list
ros2 param get /node_name parameter_name

# 查看节点信息
ros2 node info /node_name

# 查看相机状态（使用camera_monitor）
ros2 topic echo /camera_status
```

## 扩展开发

### 添加新的点云处理算法

1. 在 `norm_calc/include/norm_calc/` 中添加头文件
2. 在 `norm_calc/src/` 中实现算法
3. 在 `norm_calc_server.cpp` 中集成新算法
4. 更新 `config/norm_calc_params.yaml` 添加参数
5. 在 `CMakeLists.txt` 中添加新源文件到 add_executable
6. 重新构建

### 添加新的可执行文件

1. 在 `norm_calc/src/` 中添加实现文件
2. 在 `CMakeLists.txt` 中添加：
   ```cmake
   add_executable(your_executable src/your_file.cpp)
   target_link_libraries(your_executable ${PCL_LIBRARIES} ${OpenCV_LIBS})
   ament_target_dependencies(your_executable rclcpp sensor_msgs ...)
   install(TARGETS your_executable DESTINATION lib/${PROJECT_NAME})
   ```

### 扩展PLC通信功能

1. 修改 `snap_7/snap_7/plc_client_node.py`
2. 添加新的读写方法（write_real_to_db, write_integer_to_db等）
3. 更新数据解析逻辑
4. 调整DB块布局（如需要）
5. 使用Tsnap7test.py测试新功能

### 创建新包

```bash
# 创建C++包
ros2 pkg create --build-type ament_cmake package_name

# 创建Python包
ros2 pkg create --build-type ament_python package_name

# 添加依赖
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs package_name
```

## 测试

### 单元测试

```bash
# 运行所有测试
colcon test

# 运行特定包的测试
colcon test --packages-select package_name

# 查看测试结果
colcon test-result --all
```

### 集成测试

- **PLC通信测试**: 使用 `launch/plc_sim.launch.py` 模拟PLC
- **相机测试**: 使用 `opencv_tests/` 中的测试脚本
- **点云处理测试**: 提供测试点云数据并验证输出
- **PLC连接测试**: 使用 `Testpy/Tsnap7test.py` 进行交互式测试

## 版本控制

### Git忽略规则

```
/.vscode
/build
/install
/log
```

### 提交建议

- 提交前确保代码可以成功构建
- 使用有意义的提交信息
- 遵循语义化版本控制
- 重要更改更新 README.md 和 IFLOW.md

## 相关资源

- ROS2 官方文档: https://docs.ros.org/en/humble/
- PCL 官方文档: https://pointclouds.org/
- Snap7 文档: https://python-snap7.readthedocs.io/
- RealSense 文档: https://dev.intelrealsense.com/docs/
- OpenCV 文档: https://docs.opencv.org/

## 联系信息

- 维护者: berk
- 许可证: TODO (待定)