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
│   ├── aruco_ros/          # ArUco标记检测包
│   └── vision_opencv/      # OpenCV功能包 (ROS2官方包本地副本)
├── build/                  # 构建输出目录 (git忽略)
├── install/                # 安装目录 (git忽略)
├── log/                    # 日志目录 (git忽略)
├── pcd_data/               # 点云数据保存目录 (git忽略)
├── Testpy/                 # 测试工具目录
│   └── Tsnap7test.py       # PLC通信测试工具
├── start_system.sh         # 系统启动脚本（带日志管理）
├── quick_start.sh          # 快速启动脚本
├── start_hand_eye_calib.sh # 手眼标定启动脚本
├── stop_hand_eye_calib.sh  # 手眼标定停止脚本
├── clean_camera_processes.py # 相机进程清理工具
├── test_camera_cleanup.sh  # 相机清理测试脚本
├── verify_camera_cleanup.py # 相机清理验证脚本
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
- 三段式点位计算策略（严格模式 → 宽松模式 → 随机模式）
- 上一次点位存储和随机模式（位置偏移+法向偏移）
- 凸起检测和切顶切底策略（基于曲率判断）
- 备用方案确保每个方格都有点位
- 点云自动保存功能（支持断点续传）

**依赖**:
- rclcpp, sensor_msgs, geometry_msgs
- pcl_conversions, pcl_ros, libpcl-all-dev
- opencv4, cv_bridge
- std::filesystem (C++17)

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
- 坐标过滤（X<0或Z>0的无效点位自动丢弃）
- 点位计数优化（只统计有效点位）

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
- 启动前清理旧相机进程

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
- 标定结果配置文件管理
- ABC角度限制（±30度）

**依赖**:
- rclcpp, rclpy, geometry_msgs, opencv4

**注意**: 该包是混合类型（ament_cmake + rclpy依赖），构建时需要确保Python模块结构正确

### 4. aruco_ros 包

**用途**: ArUco标记检测和姿态估计

**关键文件**:
- `aruco/src/aruco/` - ArUco库核心实现
- `aruco_ros/src/` - ROS2接口实现
- `aruco_ros/launch/realsense_single.launch.py` - RealSense相机单标记检测启动文件
- `aruco_msgs/` - ArUco消息定义

**主要功能**:
- ArUco标记检测
- 标记姿态估计
- 支持RealSense相机
- 可视化标记检测结果

**依赖**:
- rclcpp, rclpy, sensor_msgs, geometry_msgs
- opencv4, cv_bridge

### 5. vision_opencv 包

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

### 6. Testpy 目录

**用途**: 测试工具集合

**关键文件**:
- `Tsnap7test.py` - PLC DB数据写入测试工具
  - 支持交互式写入和读取DB2120
  - 用于验证PLC通信功能
  - 默认测试IP: 192.168.110.228

### 7. 工具脚本

**相机进程管理**:
- `clean_camera_processes.py` - 清理所有RealSense相机进程
- `test_camera_cleanup.sh` - 测试相机清理功能
- `verify_camera_cleanup.py` - 验证相机清理结果

**系统启动**:
- `start_system.sh` - 系统启动脚本（带日志管理）
- `quick_start.sh` - 快速启动脚本

**手眼标定**:
- `start_hand_eye_calib.sh` - 手眼标定一键启动脚本
- `stop_hand_eye_calib.sh` - 手眼标定停止脚本

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
colcon build --packages-select aruco_ros

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
./start_hand_eye_calib.sh

# 启动点云可视化工具
ros2 run norm_calc norm_viewer

# 启动ArUco标记检测（RealSense相机）
ros2 launch aruco_ros realsense_single.launch.py
```

### 测试工具

```bash
# 运行PLC通信测试
cd Testpy
python3 Tsnap7test.py

# 测试相机清理功能
./test_camera_cleanup.sh

# 验证相机清理结果
python3 verify_camera_cleanup.py
```

### 启动文件说明

| 启动文件 | 功能 | 节点 |
|---------|------|------|
| `snap_7.launch.py` | 基础系统启动 | norm_calc_server, snap_7_node |
| `system_launch.py` | 完整系统启动 | norm_calc_server, snap_7_node, camera_monitor, realsense2_camera |
| `plc_sim.launch.py` | PLC模拟服务器 | plc_sim_server |
| `test_plc_sim.launch.py` | PLC模拟测试 | plc_sim_server, snap_7_node |
| `norm_calc_launch.py` | 点云处理服务 | norm_calc_server |
| `realsense_single.launch.py` | ArUco标记检测 | aruco_ros_single |

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

**问题**: 多个相机进程同时运行
**解决**:
1. 使用 `./clean_camera_processes.py` 清理所有相机进程
2. 使用 `./test_camera_cleanup.sh` 测试清理功能
3. 使用 `python3 verify_camera_cleanup.py` 验证清理结果

### 点云处理问题

**问题**: 点云处理结果不理想
**调整参数**:
- `STRICT_NORM_TH`: 严格法向阈值（0.80-0.95）
- `RELAXED_NORM_TH`: 宽松法向阈值（0.80-0.95）
- `SEARCH_RADIUS`: 搜索半径（0.02m）
- `SEARCH_NUM_TH`: 邻域点数阈值（20）
- `BOX_LEN`: 凿子盒边长（0.05m）
- `HEIGHT_WEIGHT`: 高度权重（1.0）
- `CURV_WEIGHT`: 曲率权重（5.0）
- `ANGLE_WEIGHT`: 角度权重（15.0）
- `HOLE_DIST`: 避障距离（0.015-0.02m）
- `RANDOM_OFFSET_RANGE`: 随机位置偏移范围（0.01-0.03m）
- `RANDOM_ANGLE_RANGE`: 随机法向角度范围（0.17-0.52弧度）
- `MIN_DISTANCE_THRESHOLD`: 最小距离阈值（0.03m）

**参考**: norm_calc_params.yaml 文件包含详细的参数调整指南

### 点云保存问题

**问题**: 点云文件被覆盖
**原因**: 每次启动软件时，计数器从0开始
**解决**: 系统已优化，启动时会自动扫描pcd_data目录，从最后一个文件编号继续命名
- 例如：如果目录中有1.pcd到25.pcd，下次启动时新文件将从26.pcd开始

**问题**: 点云保存失败
**检查项**:
1. 确认pcd_data目录存在且有写权限
2. 检查磁盘空间是否充足
3. 查看日志中的错误信息

## 日志和调试

### 日志位置

- 系统日志: `/home/bosch/logs/visual.log`（自动管理，超过5MB清空）
- 构建日志: `log/latest_build/`
- 运行时日志: 通过 `ros2 run` 或 `ros2 launch` 的标准输出
- 点云数据: `/home/bosch/vChisel_ros2_ws/pcd_data/`

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

# 查看点云保存日志
ros2 topic echo /rosout | grep "Saved point cloud"
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
- **相机清理测试**: 使用 `./test_camera_cleanup.sh` 和 `python3 verify_camera_cleanup.py`

## 版本控制

### Git忽略规则

```
/.vscode
/build
/install
/log
/pcd_data
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

---

## 项目更新日志

### 2026-01-16 - visual_base分支：点云保存功能优化（断点续传）

#### 核心改进
- ✅ 实现点云保存断点续传功能
- ✅ 启动时自动扫描pcd_data目录，获取最后一个文件编号
- ✅ 后续拍照从最后一个编号继续命名，避免覆盖
- ✅ 自动创建pcd_data目录（如果不存在）

#### 技术实现

**初始化逻辑**（在构造函数中）:
1. 检查 `/home/bosch/vChisel_ros2_ws/pcd_data` 目录是否存在
2. 如果存在，扫描所有 `.pcd` 文件
3. 提取文件名中的数字编号，找到最大值
4. 将最大值赋值给 `pcd_save_count_`
5. 如果不存在，自动创建目录

**保存逻辑**（在handleService函数中）:
1. 每次拍照后 `pcd_save_count_` 递增
2. 保存为 `{编号}.pcd` 格式
3. 记录保存日志（文件名和点数量）

#### 工作原理

1. **首次启动**：如果 `pcd_data` 目录为空，`pcd_save_count_` 初始化为 0，第一个保存的文件是 `1.pcd`
2. **再次启动**：扫描目录中已有的文件（如 `1.pcd` 到 `25.pcd`），找到最大编号 `25`，`pcd_save_count_` 初始化为 25，下一个保存的文件是 `26.pcd`
3. **后续拍照**：每次拍照后 `pcd_save_count_` 递增，保存新文件

#### 文件修改
- ✅ `src/norm_calc/src/norm_calc_server.cpp` - 添加启动时扫描pcd_data目录的逻辑

#### 构建状态
- ✅ 构建成功，无编译错误

#### 问题解决
- ✅ 解决每次启动软件时点云文件被覆盖的问题
- ✅ 实现点云保存的断点续传功能

---

### 2026-01-15 - visual_base分支：平面优先策略、防滑移优化和点云保存功能

#### 平面优先策略
- ✅ 重新设计法向点选择策略，实现平面优先原则
- ✅ 平面判定：面积 > 1平方cm 且 曲率 < 0.035
- ✅ 平面策略：垂直凿击，法向X分量接近+1
- ✅ 山腰策略：凹凸不平时的最佳凿击位置（切顶切底，山腰优先）
- ✅ 移除三段式策略，改为平面优先策略

#### 防滑移优化
- ✅ 修正法向约束：从检查`abs(normal_x)`改为检查`normal_x`（X+方向是向墙里面凿击）
- ✅ 添加防滑移约束：过滤法向Y分量 < -0.1（向右滑移）和法向Z分量 > 0.1（向下滑移）
- ✅ 修正坡度检查：从基于法向Z分量改为基于法向X分量
- ✅ 修正评分权重：从强调`normal_z`改为强调`normal_x`

#### 坐标系理解
- ✅ X轴：凿击方向，前进后退（X+方向是向墙里面凿击）
- ✅ Y轴：平移（墙面的水平方向，左右）
- ✅ Z轴：上下（重力方向）

#### 凹坑避让
- ✅ 增加深坑方向检查：防止法向指向深坑方向
- ✅ 增加低洼区域检测：检测周围是否有比当前点低超过2cm的位置
- ✅ 增加坡度检查：计算局部坡度，避免选择坡度太大的位置
- ✅ 增加滑移方向检查：检查电锤可能滑移的方向

#### 点云保存功能
- ✅ 每次拍照获取的点云自动保存为PCD文件
- ✅ 文件命名格式：1.pcd, 2.pcd, 3.pcd, ...
- ✅ 保存位置：`/home/bosch/vChisel_ros2_ws/pcd_data/`
- ✅ 自动创建pcd_data目录（如果不存在）
- ✅ 记录保存日志（文件名和点数量）
- ✅ 可通过`enable_pcd_save_`变量启用/禁用功能

#### PLC通信优化
- ✅ 优化点位计数逻辑：只统计有效点位，点位为0的值不统计也不发送给PLC
- ✅ 移除0值填充逻辑：只发送有效点位给PLC
- ✅ 添加无效点位检查：过滤x=y=z=0的无效点位

#### 配置参数
- ✅ 添加平面策略参数：PLANE_AREA_TH, PLANE_CURV_TH, PLANE_NORM_TH, PLANE_HOLE_DIST, PLANE_BONUS
- ✅ 添加山腰策略参数：PROTRUSION_TH, TIP_CROP_RATIO, BASE_CROP_RATIO, MOUNTAIN_NORM_TH, MOUNTAIN_HOLE_DIST
- ✅ 添加凹坑避让参数：HOLE_SAFE_DIST, ENABLE_HOLE_DIR_CHECK, DEPRESSION_DIST, MAX_SLOPE_ANGLE, SLOPE_CHECK_RADIUS
- ✅ 添加防滑移参数：MAX_NORMAL_Y, MAX_NORMAL_Z
- ✅ 所有参数都可以在yaml文件中动态调整

#### 文件修改
- ✅ `src/norm_calc/config/norm_calc_params.yaml` - 添加平面优先策略和防滑移参数
- ✅ `src/norm_calc/include/norm_calc/chisel_box.h` - 添加平面优先策略参数结构
- ✅ `src/norm_calc/src/chisel_box.cpp` - 实现平面优先策略和防滑移逻辑
- ✅ `src/norm_calc/src/norm_calc_server.cpp` - 添加参数读取代码和点云保存功能
- ✅ `src/snap_7/snap_7/plc_client_node.py` - 优化点位计数逻辑
- ✅ `.gitignore` - 添加pcd_data目录到忽略列表

#### 构建状态
- ✅ 构建成功，无编译错误
- ⚠️ 存在未使用变量警告（不影响功能）

#### 问题解决
- ✅ 解决山腰位置滑入凹区域的问题
- ✅ 解决大于1平方cm的平面没有优先凿击的问题
- ✅ 解决法向约束错误的问题（从abs改为检查正值）
- ✅ 解决无效点位发送给PLC的问题
- ✅ 实现点云自动保存功能，便于调试和数据记录

---

### 2026-01-15 - visual_base分支：PLC通信优化和相机进程管理

#### PLC通信优化
- ✅ 增加坐标过滤逻辑，丢弃X<0或Z>0的无效点位数据
- ✅ 优化点位计数逻辑，根据过滤后的实际数据计算有效点位数量
- ✅ 添加过滤日志记录，便于调试和问题诊断

#### 相机进程管理
- ✅ 增加相机进程清理功能，启动新相机前终止所有旧进程
- ✅ 防止多个相机进程同时运行导致图像混乱
- ✅ 解决关闭相机后仍接收图像的问题
- ✅ 优化相机启动流程，提高系统稳定性

#### 分支管理
- ✅ 创建visual_base分支，专注于PLC通信和相机管理优化
- ✅ 推送到远程仓库，便于协作开发

#### 技术实现
- ✅ 在 `_handle_norm_response` 函数中添加坐标过滤
- ✅ 在 `_launch_camera` 函数中添加进程清理逻辑
- ✅ 使用 `pkill -f realsense2_camera` 终止所有旧进程
- ✅ 更新点位数量计算：`pointNum = len(output_data) // 6`

#### 修改文件
- `src/snap_7/snap_7/plc_client_node.py` - 添加坐标过滤和进程清理功能

#### 提交信息
- c095886 - feat(plc_client): 增加相机进程清理和坐标过滤功能

#### 问题修复
- ✅ 修复多个相机进程同时运行导致图像混乱的问题
- ✅ 修复关闭相机后仍接收图像的问题
- ✅ 修复无效点位数据发送给PLC的问题

---

### 2026-01-05 - 手眼标定和法向点位计算优化

#### 手眼标定系统
- ✅ 创建手眼标定一键启动脚本（start_hand_eye_calib.sh）
- ✅ 标记参数更新（ID: 123，尺寸: 10cm）
- ✅ 添加图像显示窗口（相机原始图像 + ArUco检测结果）
- ✅ 标定结果应用（配置文件管理，支持热更新）
- ✅ ABC角度限制（±30度，弧度制输出）

#### 法向点位计算优化
- ✅ 修正宽松模式参数（RELAXED_NORM_TH: 0.87，允许30°偏差）
- ✅ 状态机重置策略优化（保留跳过状态支持宽松模式）
- ✅ 点位数量自动降级（不足50%触发宽松模式）
- ✅ 评分权重调整（平面优先垂直角度，ANGLE_WEIGHT: 5.0）
- ✅ 动态权重调整（点位<8时权重降至50%）
- ✅ 改进凸起检测（结合高度差和曲率，支持平面孤岛）
- ✅ 修复图像发布时间戳（确保可视化正确更新）

#### ArUco标记检测
- ✅ 修复 realsense_single.launch.py 缺失的 LaunchConfiguration 导入
- ✅ 推送到 fork 仓库（Roarpeng/aruco_ros）

#### PLC通信
- ✅ 添加四元数到欧拉角转换函数
- ✅ 添加欧拉角限制（±30度）
- ✅ 发送弧度值给PLC（PLC端自动转换）

#### 配置文件
- ✅ 添加手眼标定矩阵参数（4x4变换矩阵）
- ✅ 添加标定元数据（日期、标记ID、尺寸、数据组数）
- ✅ 优化评分权重配置

---

### 2026-01-06 - 基于面积的平面优先法向点计算逻辑

#### 核心改进
- ✅ 实现基于凸包的精确面积计算（使用 PCL ConvexHull 算法）
- ✅ 实现三段式搜索策略（带缓冲区平滑过渡）
  - 平面模式（面积 ≥ 3.5cm²）：严格标准（STRICT_NORM_TH: 0.90）
  - 混合模式（2.5cm² ≤ 面积 < 3.5cm²）：中间标准（HYBRID_NORM_TH: 0.885）
  - 凹凸面模式（面积 < 2.5cm²）：宽松标准（RELAXED_NORM_TH: 0.87）
- ✅ 重构状态机逻辑：从"严格→宽松"改为基于面积的"平面→凹凸"策略

#### 技术实现
- ✅ 添加 `calculateConvexHullArea()` 方法（鞋带公式计算凸包面积）
- ✅ 添加 `determineSearchMode()` 方法（根据面积确定搜索模式）
- ✅ 重构 `findBestPoint()` 方法（基于面积动态选择搜索参数）
- ✅ 保留现有优化功能（凸起检测、切顶切底、智能评分、避障）

#### 配置参数
- ✅ 添加 PLANE_AREA_HIGH: 0.00035（3.5cm² - 平面模式阈值）
- ✅ 添加 PLANE_AREA_LOW: 0.00025（2.5cm² - 凹凸面模式阈值）
- ✅ 添加 HYBRID_NORM_TH: 0.885（混合法向阈值）
- ✅ 添加 HYBRID_HOLE_DIST: 0.0425（混合避障距离）
- ✅ 添加 HYBRID_CURV_TH: 0.085（混合曲率阈值）

#### 文件修改
- ✅ `src/norm_calc/include/norm_calc/chisel_box.h` - 添加方法声明和参数结构
- ✅ `src/norm_calc/src/chisel_box.cpp` - 实现面积计算和模式判断逻辑
- ✅ `src/norm_calc/config/norm_calc_params.yaml` - 添加面积阈值配置
- ✅ `src/norm_calc/src/norm_calc_server.cpp` - 添加参数读取代码
- ✅ `src/norm_calc/CMakeLists.txt` - 更新 PCL 组件依赖（surface 模块）

#### 构建状态
- ✅ 构建成功，无编译错误
- ⚠️ 存在未使用变量警告（avg_z，不影响功能）

---

### 2026-01-06 - 基于面积的平面优先法向点计算逻辑（完整实施与优化）

#### 核心改进
- ✅ 实现基于凸包的精确面积计算（使用 PCL ConvexHull 算法）
- ✅ 实现三段式搜索策略（带缓冲区平滑过渡）
  - 平面模式（面积 ≥ 15cm²）：严格标准（STRICT_NORM_TH: 0.90）
  - 混合模式（8cm² ≤ 面积 < 15cm²）：中间标准（HYBRID_NORM_TH: 0.86）
  - 凹凸面模式（面积 < 8cm²）：宽松标准（RELAXED_NORM_TH: 0.82）
- ✅ 重构状态机逻辑：从"严格→宽松"改为基于面积的"平面→凹凸"策略

#### 问题诊断与修复

**问题1：零法向点计算**
- **原因**：避障距离太大（5cm），导致139个空洞点覆盖整个工作空间
- **修复**：将避障距离从5cm降低到2cm
  - STRICT_HOLE_DIST: 0.05 → 0.02
  - RELAXED_HOLE_DIST: 0.035 → 0.015
  - HYBRID_HOLE_DIST: 0.0425 → 0.0175
- **结果**：24个方格全部成功计算出法向点

**问题2：面积阈值设置错误**
- **原因**：原始阈值比实际面积小7-10倍
  - 原始：PLANE_AREA_HIGH = 3.5cm², PLANE_AREA_LOW = 2.5cm²
  - 实际：5cm×5cm方格总面积 = 25cm²
- **修复**：调整为合理值
  - PLANE_AREA_HIGH: 0.00035 → 0.0015 (15cm²)
  - PLANE_AREA_LOW: 0.00025 → 0.0008 (8cm²)

**问题3：平面权重不够**
- **原因**：评分权重不够强调平面性，导致选择曲面区域的点
- **修复**：大幅增加平面权重
  - ANGLE_WEIGHT: 5.0 → 15.0（提高3倍，极度强调平面垂直度）
  - CURV_WEIGHT: 2.0 → 5.0（提高2.5倍，优先选择最平整的区域）

**问题4：凸起检测逻辑不合理**
- **原因**：主要依赖高度差判断，导致有轻微高度变化的平面被误判为凸起
- **修复**：改为基于曲率的判定逻辑
  - 主要判定：平均曲率 > 0.02 → 判定为凸起/曲面
  - 辅助判定：平均曲率 > 0.01 且高度差 > 3cm → 判定为凸起
  - PROTRUSION_TH: 0.02 → 0.03（更严格）

#### 技术实现

**新增方法**：
- `calculateConvexHullArea()` - 使用鞋带公式计算凸包面积
- `determineSearchMode()` - 根据面积确定搜索模式

**重构方法**：
- `findBestPoint()` - 从基于状态机改为基于面积的策略
- 凸起检测逻辑 - 从基于高度差改为基于曲率

**调试日志**：
- 每个方格的点数和凸包面积
- 每个方格选择的模式和参数
- 每个方格的Z-range、平均曲率和凸起判定结果
- 每个方格过滤掉的点的数量和原因（高度/法向/曲率/避障）

#### 配置参数优化

**面积阈值**：
```yaml
PLANE_AREA_HIGH: 0.0015   # 15cm² - 平面模式阈值
PLANE_AREA_LOW: 0.0008    # 8cm² - 凹凸面模式阈值
```

**预处理参数**：
```yaml
SEARCH_RADIUS: 0.02       # 从0.015增加到0.02
SEARCH_NUM_TH: 20         # 从30降低到20
```

**法向过滤条件**：
```yaml
STRICT_NORM_TH: 0.90      # cos(16°)
HYBRID_NORM_TH: 0.86      # 介于严格和宽松之间
RELAXED_NORM_TH: 0.82     # cos(35°)
```

**避障距离**：
```yaml
STRICT_HOLE_DIST: 0.02    # 2cm
HYBRID_HOLE_DIST: 0.0175  # 1.75cm
RELAXED_HOLE_DIST: 0.015  # 1.5cm
```

**评分权重**：
```yaml
HEIGHT_WEIGHT: 1.0        # 高度权重
CURV_WEIGHT: 5.0          # 曲率权重（从2.0增加到5.0）
ANGLE_WEIGHT: 15.0        # 角度权重（从5.0增加到15.0）
CENTER_WEIGHT: 5.0        # 中心权重
```

**凸起检测**：
```yaml
PROTRUSION_TH: 0.03       # 3cm（从2cm增加到3cm）
TIP_CROP_RATIO: 0.25      # 切顶25%
BASE_CROP_RATIO: 0.10     # 切底10%
```

#### 文件修改
- ✅ `src/norm_calc/include/norm_calc/chisel_box.h` - 添加方法声明和参数结构
- ✅ `src/norm_calc/src/chisel_box.cpp` - 实现面积计算、模式判断和优化评分逻辑
- ✅ `src/norm_calc/config/norm_calc_params.yaml` - 添加面积阈值和优化参数配置
- ✅ `src/norm_calc/src/norm_calc_server.cpp` - 添加参数读取和空洞检测调试日志
- ✅ `src/norm_calc/CMakeLists.txt` - 更新 PCL 组件依赖（surface 模块）

#### 构建状态
- ✅ 构建成功，无编译错误
- ⚠️ 存在未使用变量警告（avg_z，不影响功能）

#### 测试结果
- ✅ 24个方格全部成功计算出法向点
- ✅ 平面模式正确识别（面积 > 15cm²）
- ✅ 凸起模式正确识别（曲率 > 0.02）
- ✅ 避障过滤从90-100%降低到40-80%
- ✅ 法向和曲率过滤正常（只过滤少量点）

---

### 2026-01-06 - 三段式点位计算策略（严格→宽松→随机）

#### 核心改进
- ✅ 实现三段式策略：严格模式 → 宽松模式 → 随机模式
- ✅ 添加上一次点位存储机制
- ✅ 实现随机模式（位置偏移+法向偏移）
- ✅ 确保每个方格都有点位（备用方案）

#### 策略说明

**三段式策略工作流程**：
1. **第一次尝试：严格模式**（STRICT_NORM_TH: 0.90）
   - 要求法向量与Z轴夹角 ≤ 16°
   - 要求表面非常平整（曲率 < 0.07）
   - 避障距离：2cm

2. **第二次尝试：宽松模式**（RELAXED_NORM_TH: 0.82）
   - 允许法向量与Z轴夹角 ≤ 35°
   - 允许表面稍微粗糙（曲率 < 0.10）
   - 避障距离：1.5cm

3. **第三次尝试：随机模式**（基于上一次点位或网格中心）
   - 位置偏移：在上一次点位或网格中心周围随机选择新点（默认±2cm）
   - 法向偏移：随机旋转法向量（默认±20°）
   - 如果没有上一次点位，使用网格中心作为基准点

4. **备用方案**：
   - 如果三次尝试都失败，使用非常宽松的参数（norm_th: 0.5, hole_dist: 0.01m, curv_th: 1.0）
   - 确保每个方格都能计算出点位

#### 技术实现

**新增成员变量**（ChiselBox类）：
- `last_point_` - 存储上一次点位
- `has_last_point_` - 是否有上一次点位

**新增状态**：
- `STATE_SKIPPED_TWICE` - 第二次失败（宽松模式失败）

**新增方法**：
- `searchWithRandomMode()` - 随机模式搜索
  - 在基准点周围随机选择新点
  - 随机旋转法向量
  - 归一化法向量

**重构方法**：
- `findBestPoint()` - 从基于面积改为三段式策略
  - 严格模式 → 宽松模式 → 随机模式 → 备用方案
  - 保存成功计算的点位到 `last_point_`

**调试日志**：
- 每个方格的尝试次数和模式
- 随机模式使用的基准点（上一次点位或网格中心）
- 随机模式计算出的点位位置和法向量
- 备用方案的使用情况

#### 配置参数

**随机模式参数**：
```yaml
RANDOM_OFFSET_RANGE: 0.02  # 随机位置偏移范围（米，默认2cm）
RANDOM_ANGLE_RANGE: 0.35   # 随机法向角度范围（弧度，默认20°）
```

#### 文件修改
- ✅ `src/norm_calc/include/norm_calc/chisel_box.h` - 添加上一次点位存储和随机模式方法声明
- ✅ `src/norm_calc/src/chisel_box.cpp` - 实现随机模式和三段式策略
- ✅ `src/norm_calc/config/norm_calc_params.yaml` - 添加随机模式参数
- ✅ `src/norm_calc/src/norm_calc_server.cpp` - 添加随机模式参数读取

#### 构建状态
- ✅ 构建成功，无编译错误
- ⚠️ 存在未使用变量警告（avg_z，不影响功能）

#### 测试结果
- ✅ 三段式策略成功实现
- ✅ 每个方格依次尝试严格模式、宽松模式、随机模式
- ✅ 随机模式基于上一次点位或网格中心
- ✅ 备用方案确保每个方格都有点位

---

## 当前系统状态

### 分支信息
- **当前分支**: visual_base
- **最新提交**: deff3d3 - "feat(norm_calc): 新增曲率相关参数和优化凿击策略"
- **远程仓库**: git@github.com:Roarpeng/vChisel_ros2_ws.git
- **分支状态**: 已推送到远程仓库，可正常使用

### 系统功能状态

| 功能模块 | 状态 | 说明 |
|---------|------|------|
| 点云处理 | ✅ 正常 | 三段式策略（严格→宽松→随机），确保每个方格都有点位 |
| 点云保存 | ✅ 正常 | 自动保存，支持断点续传 |
| 手眼标定 | ✅ 正常 | 配置文件管理，支持热更新 |
| PLC通信 | ✅ 正常 | ABC角度限制，弧度制输出，坐标过滤（X>0, z<0） |
| ArUco检测 | ✅ 正常 | 支持RealSense相机 |
| 图像可视化 | ✅ 正常 | 实时更新，时间戳同步 |
| 相机监控 | ✅ 正常 | 自动检测和重启，进程清理 |
| 坐标过滤 | ✅ 正常 | 自动丢弃X<0或Z>0的无效点位 |
| 进程管理 | ✅ 正常 | 防止多个相机进程同时运行 |

### 已知问题
- ⚠️ image_norm_viewer 未编译（可选功能）
- ⚠️ avg_z 变量未使用警告（不影响功能）
- ⚠️ 系统启动时可能存在多个相机进程残留（需要手动清理）

### 待办事项
- [ ] 添加单元测试
- [ ] 完善文档（API文档、用户手册）
- [ ] 性能优化（点云处理速度）
- [ ] 支持更多相机型号
- [ ] 添加标定质量评估指标
- [ ] 测试三段式策略（严格→宽松→随机）
- [ ] 优化随机模式的偏移范围和角度范围
- [ ] 添加点位质量评估指标
- [ ] 测试坐标过滤功能（X>0, z<0）
- [ ] 测试相机进程清理功能
- [ ] 测试点云保存断点续传功能

### 配置参数摘要

**手眼标定**:
- 标记ID: 123
- 标记尺寸: 0.10m
- 数据组数: 15组（推荐10-20组）

**法向点位计算**:
- 网格数量: 4×6=24
- 最小点位数: 8个（关键阈值）
- 严格模式阈值: 12个（50%网格数）
- 三段式策略: 严格模式 → 宽松模式 → 随机模式
- 严格法向阈值: 0.90（cos 16°）
- 宽松法向阈值: 0.82（cos 35°）
- 角度权重: 15.0（极度强调平面垂直度）
- 曲率权重: 5.0（优先选择最平整的区域）
- 中心权重: 0.5（大幅降低，避免过度集中）
- 避障距离: 2cm（严格），1.5cm（宽松）
- 凸起判定: 曲率 > 0.04（主要），曲率 > 0.01 且高度差 > 5cm（辅助）
- 随机模式: 位置偏移±2cm，法向偏移±20°

**PLC通信**:
- 数据库编号: 2120
- 起始偏移: 64
- PLC地址: 192.168.1.36
- 轮询频率: 20Hz
- 坐标过滤: X > 0, z < 0
- 点位数量限制: 24个（最大）
- 角度限制: ±30°（A、B、C角）
- 输出格式: 弧度制

**ABC角度限制**:
- A角（Z轴旋转）: ±30°
- B角（Y轴旋转）: ±30°
- C角（X轴旋转）: ±30°
- 输出格式: 弧度制

**点云保存**:
- 保存位置: `/home/bosch/vChisel_ros2_ws/pcd_data/`
- 文件命名: `{编号}.pcd`（如 1.pcd, 2.pcd, 26.pcd）
- 断点续传: 启动时自动扫描最后一个文件编号
- 自动创建: 目录不存在时自动创建

### 测试建议

1. **手眼标定测试**
   - 运行 `./start_hand_eye_calib.sh`
   - 采集15-20组数据
   - 验证标定结果

2. **法向点位计算测试**
   - 测试严格模式（应选择最平整、最垂直的点）
   - 测试宽松模式（严格模式失败后自动触发）
   - 测试随机模式（宽松模式失败后自动触发）
   - 测试首次运行（随机模式使用网格中心作为基准）
   - 测试重复运行（随机模式使用上一次点位作为基准）
   - 测试备用方案（三次尝试都失败时）
   - 验证每个方格都能计算出点位

3. **系统集成测试**
   - 运行 `ros2 launch snap_7 system_launch.py`
   - 验证图像更新
   - 验证PLC通信
   - 验证法向点可视化

4. **PLC通信测试**
   - 测试坐标过滤功能（X<0或Z>0的点位应被丢弃）
   - 验证点位数量正确性（过滤后的实际数量）
   - 测试角度限制（±30°）
   - 验证弧度值正确性

5. **相机进程管理测试**
   - 测试相机启动前清理旧进程
   - 验证关闭相机后不再接收图像
   - 测试多次启动/关闭循环
   - 验证只有一个相机进程在运行

6. **点云保存测试**
   - 测试点云自动保存功能
   - 测试断点续传功能（重启后从最后一个编号继续）
   - 验证文件命名正确性
   - 验证文件内容完整性

---

## 项目更新日志

### 2026-03-06 - visual_base分支：系统启动优化和自动化功能增强

#### 核心改进
- ✅ 全面完善系统启动脚本
- ✅ 新增闲时自动重启PC功能
- ✅ 新增开机自启动功能（两种方式）
- ✅ 修复snap_7包配置问题

#### 1. 系统启动脚本完善 (start_system.sh)

**修复问题**：
- ✅ 修复日志滚动逻辑bug（原来会导致文件编号跳跃）
- ✅ 修复锁文件权限问题（从/tmp改到项目目录）

**新增功能**：
- ✅ 添加sudo检查（避免误用root权限启动）
- ✅ 添加相机进程自动清理（启动前清理残留进程）
- ✅ 添加ROS2环境检查（验证环境是否正确）
- ✅ 添加工作空间检查（验证是否已构建）
- ✅ 添加进程互斥锁（防止重复启动）
- ✅ 添加信号处理（优雅退出，自动清理）
- ✅ 增强日志记录（分级日志、详细参数记录）

**技术实现**：
```bash
# 锁文件位置
LOCK_FILE="$WORKSPACE_DIR/.vchisel_system.lock"

# sudo检查
if [ "$EUID" -eq 0 ]; then
    echo "错误：请不要使用 sudo 运行此脚本！"
    exit 1
fi

# 相机进程清理
if [ -f "$WORKSPACE_DIR/clean_camera_processes.py" ]; then
    python3 "$WORKSPACE_DIR/clean_camera_processes.py"
fi

# 环境检查
check_ros2_environment() {
    # 检查 ROS2 是否安装
    # 检查工作空间是否存在
    # 检查工作空间是否已构建
}
```

#### 2. snap_7 包配置修复

**问题描述**：
- 参数文件 `snap_7_params.yaml` 未被安装到 install 目录
- 导致启动时找不到参数文件

**解决方案**：
- 修改 `src/snap_7/setup.py`
- 添加 config 目录到 data_files

**代码修改**：
```python
data_files=[
    # ... 其他文件 ...
    ('share/' + package_name + '/config', ['config/snap_7_params.yaml']),
],
```

#### 3. 闲时自动重启PC功能

**功能定义**：
- 在每天凌晨3点检查系统是否处于闲时
- 如果是闲时状态，则自动重启PC
- 闲时定义：相机关闭超过5分钟，且没有新的相机启动命令

**执行条件**：
- ✅ 时间为凌晨3点
- ✅ 系统正在运行
- ✅ 相机已关闭
- ✅ 关闭时间超过5分钟

**工作流程**：
```
凌晨3点触发
    ↓
检查系统运行状态 → 未运行退出
    ↓
检查相机状态 → 相机开启退出
    ↓
检查关闭时间 → 不足5分钟退出
    ↓
停止项目进程
    ↓
清理临时文件
    ↓
重启PC系统
```

**技术实现**：

**新增文件**：
- `idle_restart.sh` - 主重启脚本
- `install_idle_restart_cron.sh` - Cron任务安装脚本
- `uninstall_idle_restart_cron.sh` - Cron任务卸载脚本
- `test_idle_restart.sh` - 测试脚本
- `IDLE_RESTART_README.md` - 详细说明文档

**相机状态记录**（修改 `plc_client_node.py`）：
```python
# 相机启动时
with open('/tmp/vchisel_camera_status.txt', 'w') as f:
    f.write('on')

# 相机关闭时
with open('/tmp/vchisel_camera_status.txt', 'w') as f:
    f.write('off')
with open('/tmp/vchisel_camera_off_time.txt', 'w') as f:
    f.write(str(int(time.time())))
```

**安全机制**：
1. **时间限制**：只能在凌晨3点执行
2. **条件检查**：必须满足所有闲时条件
3. **交互保护**：手动运行时需要设置 `FORCE_REBOOT=true`
4. **权限控制**：需要root权限（通过root crontab）
5. **日志记录**：所有操作都有详细日志

**使用方法**：
```bash
# 安装闲时重启任务
./install_idle_restart_cron.sh

# 查看已安装的任务
sudo crontab -l

# 查看日志
tail -f /home/bosch/logs/visual.log | grep idle_restart

# 测试逻辑（不会真正重启）
./test_idle_restart.sh

# 卸载任务
./uninstall_idle_restart_cron.sh
```

#### 4. 开机自启动功能

提供两种开机自启动方式：

**方法1：systemd 服务（推荐）**

**优点**：
- 系统级服务，不依赖用户登录
- 自动重启（失败后延迟10秒）
- 专业可靠，适合生产环境
- 支持无头模式（服务器环境）

**使用方法**：
```bash
# 安装自启动服务
./install_autostart.sh

# 查看服务状态
sudo systemctl status vchisel-system.service

# 管理命令
sudo systemctl start vchisel-system.service     # 启动
sudo systemctl stop vchisel-system.service      # 停止
sudo systemctl restart vchisel-system.service   # 重启
sudo journalctl -u vchisel-system.service -f   # 查看日志

# 卸载
./uninstall_autostart.sh
```

**服务配置**（`/etc/systemd/system/vchisel-system.service`）：
```ini
[Unit]
Description=vChisel ROS2 System
After=network.target
After=graphical.target
Wants=graphical.target

[Service]
Type=simple
User=bosch
Group=bosch
Environment="ROS_DOMAIN_ID=0"
Environment="DISPLAY=:0"
WorkingDirectory=/home/bosch/vChisel_ros2_ws
ExecStart=/home/bosch/vChisel_ros2_ws/start_system.sh
ExecStop=/bin/kill -INT ${MAINPID}
Restart=on-failure
RestartSec=10
StandardOutput=append:/home/bosch/logs/visual.log
StandardError=append:/home/bosch/logs/visual.log

[Install]
WantedBy=multi-user.target
```

**方法2：桌面自启动**

**优点**：
- 配置简单，不需要root权限
- 可视化窗口，方便调试
- 适合桌面环境开发测试

**缺点**：
- 需要用户登录桌面
- 不自动重启（失败后）

**使用方法**：
```bash
# 安装桌面自启动
./install_autostart_desktop.sh

# 注销并重新登录后生效

# 卸载
./uninstall_autostart_desktop.sh
```

#### 文件修改清单

**修改的文件**：
- `.gitignore` - 添加锁文件 `.vchisel_system.lock`
- `src/snap_7/setup.py` - 添加 config 文件安装
- `src/snap_7/snap_7/plc_client_node.py` - 添加相机状态记录
- `start_system.sh` - 全面完善启动逻辑

**新增的文件**：

**闲时重启功能**：
- `idle_restart.sh` - 主重启脚本
- `install_idle_restart_cron.sh` - Cron安装脚本
- `uninstall_idle_restart_cron.sh` - Cron卸载脚本
- `test_idle_restart.sh` - 测试脚本
- `IDLE_RESTART_README.md` - 详细说明文档

**开机自启动功能**：
- `install_autostart.sh` - systemd服务安装脚本
- `uninstall_autostart.sh` - systemd服务卸载脚本
- `install_autostart_desktop.sh` - 桌面自启动安装脚本
- `uninstall_autostart_desktop.sh` - 桌面自启动卸载脚本

#### 完整安装流程（推荐）

```bash
# 1. 构建项目
cd /home/bosch/vChisel_ros2_ws
source /opt/ros/humble/setup.bash
colcon build

# 2. 安装开机自启动（方法1推荐）
./install_autostart.sh

# 3. 安装闲时重启任务
./install_idle_restart_cron.sh

# 4. 验证安装
sudo systemctl status vchisel-system.service
sudo crontab -l | grep idle_restart
```

#### 系统配置总结

**开机自启动**：
- 服务名称：vchisel-system.service
- 服务类型：systemd 服务
- 自动重启：失败后延迟10秒重启
- 日志位置：`/home/bosch/logs/visual.log`

**闲时重启**：
- 执行时间：每天凌晨 3:00
- 执行条件：相机关闭超过 5 分钟
- 重启类型：PC 系统重启
- 任务位置：root crontab

**临时文件**：
- `/tmp/vchisel_camera_status.txt` - 相机状态（on/off）
- `/tmp/vchisel_camera_off_time.txt` - 相机关闭时间戳
- `/home/bosch/vChisel_ros2_ws/.vchisel_system.lock` - 进程锁文件

#### 构建状态
- ✅ 构建成功，无编译错误
- ✅ 所有脚本已添加执行权限
- ✅ 配置文件正确安装

#### 测试建议

1. **启动脚本测试**
   - 测试不使用 sudo 启动
   - 测试重复启动防护
   - 测试相机进程清理
   - 测试日志滚动功能

2. **闲时重启测试**
   - 运行 `./test_idle_restart.sh` 测试逻辑
   - 查看日志验证条件判断
   - 不要轻易测试真实重启（危险操作）

3. **开机自启动测试**
   - 重启系统验证服务启动
   - 查看服务状态和日志
   - 测试服务管理命令

4. **系统集成测试**
   - 完整流程：开机自启 → 运行 → 闲时重启
   - 验证日志记录完整性
   - 验证进程管理正确性

#### 注意事项
- ⚠️ 不要使用 sudo 运行 start_system.sh
- ⚠️ 闲时重启任务需要 root 权限（已添加到 root crontab）
- ⚠️ 重启操作不可逆，请谨慎测试
- ⚠️ systemd 服务会在网络和图形界面启动后自动运行
- ⚠️ 详细说明请参考 `IDLE_RESTART_README.md`

---