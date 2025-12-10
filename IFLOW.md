# vChisel_ros2_ws 项目说明

## 项目概述

这是一个基于 ROS2 (Robot Operating System 2) 的项目，主要用于通过相机获取点云数据并进行法向量计算。项目包含两个主要的 ROS2 包：

1. `norm_calc` - C++ 包，负责处理相机数据、执行点云处理和法向量计算，并提供 ROS2 服务
2. `snap_7` - Python 包，通过 snap7 库与西门子 PLC (可编程逻辑控制器) 通信

该项目的主要功能是通过相机获取图像，计算出目标点的法向量，然后将结果写入 PLC 的数据块中，用于工业自动化应用。

## 项目结构

```
/home/bosch/vChisel_ros2_ws/
├── src/
│   ├── norm_calc/          # C++ ROS2 包，执行法向量计算
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   │   └── norm_calc_params.yaml    # 参数配置文件
│   │   ├── include/        # 头文件目录
│   │   ├── launch/         # 启动文件目录
│   │   │   └── norm_calc_launch.py      # norm_calc包启动文件
│   │   ├── scripts/        # 脚本文件目录
│   │   ├── src/            # 源代码目录
│   │   │   ├── beifen      # 备份文件
│   │   │   ├── chisel_box.cpp           # 凿子盒相关算法实现
│   │   │   ├── edge_grid.cpp            # 边缘网格算法实现
│   │   │   ├── hole_detector.cpp        # 孔洞检测算法实现
│   │   │   ├── norm_calc_server.cpp     # ROS2服务端主实现
│   │   │   ├── norm_calc.cpp            # 法向量计算主算法
│   │   │   └── norm_viewer.cpp          # 可视化查看器
│   │   └── srv/            # 服务定义目录
│   │       └── NormCalcData.srv         # 法向量计算服务定义
│   └── snap_7/             # Python ROS2 包，与 PLC 通信
│       ├── package.xml
│       ├── setup.cfg
│       ├── setup.py
│       ├── launch/         # 启动文件目录
│       │   └── snap_7.launch.py         # 整体系统启动文件
│       ├── resource/       # 资源文件目录
│       ├── snap_7/         # Python模块目录
│       │   ├── __init__.py
│       │   ├── len.py      # 备用文件
│       │   ├── plc_client_node.py       # PLC客户端节点主实现
│       │   └── plcclientbeifen          # PLC客户端备份文件
│       └── test/           # 测试文件目录
├── build/                  # 编译输出目录
├── install/                # 安装输出目录
└── log/                    # 构建日志目录
```

## 项目组成

### norm_calc 包 (C++)

- **功能**: 实现点云数据处理、法向量计算，并提供 ROS2 服务
- **主要文件**:
  - `src/norm_calc_server.cpp`: ROS2 服务节点，实现 `norm_calc/srv/NormCalcData` 服务
  - `src/norm_calc.cpp`: 核心法向量计算算法实现
  - `src/chisel_box.cpp`: 凿子盒算法实现
  - `src/edge_grid.cpp`: 边缘网格算法实现
  - `src/hole_detector.cpp`: 孔洞检测算法实现
  - `CMakeLists.txt`: CMake 配置文件，定义编译规则
  - `config/norm_calc_params.yaml`: 参数配置文件
  - `launch/norm_calc_launch.py`: 启动文件，配置并启动 norm_calc 服务节点
- **依赖**: PCL (点云库), OpenCV, ROS2 组件等
- **服务**: `norm_calc` - 接收序列号，返回计算出的法向量点列表
- **可执行文件**:
  - `norm_calc_node`: 主服务节点
  - `norm_viewer_node`: 可视化节点

### snap_7 包 (Python)

- **功能**: 与西门子 PLC 通信，通过 snap7 库读写数据块
- **主要文件**:
  - `snap_7/plc_client_node.py`: 主节点，实现与 PLC 的通信逻辑
  - `launch/snap_7.launch.py`: 启动文件，同时启动 norm_calc 和 PLC 客户端节点
- **功能逻辑**: 
  - 周期性读取 PLC DB2110 的触发值
  - 当触发值为 110 时，调用 `norm_calc` 服务
  - 将计算结果写回 PLC DB
- **依赖**: snap7 库、ROS2 Python 客户端库 (rclpy)
- **参数**:
  - `plc_address`: PLC IP 地址 (默认 192.168.1.36)
  - `plc_rack`: PLC 机架号 (默认 0)
  - `plc_slot`: PLC 插槽号 (默认 1)
  - `db_number`: 数据块号 (默认 2110)
  - `db_start`: 数据块起始地址 (默认 0)
  - `poll_rate`: 读取频率 (默认 20.0 Hz)

## 编译与运行

### 编译项目

```bash
cd /home/bosch/vChisel_ros2_ws
colcon build --packages-select norm_calc snap_7
source install/setup.bash
```

### 运行项目

1. **运行完整系统**:
   ```bash
   ros2 launch snap_7 snap_7.launch.py
   ```

2. **单独运行 norm_calc 服务**:
   ```bash
   ros2 launch norm_calc norm_calc_launch.py
   ```

3. **单独运行 PLC 客户端**:
   ```bash
   ros2 run snap_7 plc_client_node
   ```

4. **测试服务**:
   ```bash
   ros2 service call /norm_calc norm_calc/srv/NormCalcData "{seq: 1}"
   ```

### 依赖安装

确保安装了必要的依赖包:

- ROS2 Humble Hawksbill (或对应版本)
- snap7 库和 Python 绑定: `pip install snap7`
- PCL (Point Cloud Library): `sudo apt install libpcl-dev pcl-tools`
- OpenCV: `sudo apt install libopencv-dev`
- realsense2 相机驱动: `sudo apt install ros-humble-realsense2-camera`
- cv_bridge: `sudo apt install ros-humble-cv-bridge`

## 参数配置

### norm_calc 参数

在 `src/norm_calc/config/norm_calc_params.yaml` 中定义，包括:

- `BOX_LEN`: 凿子盒尺寸 (默认 0.05)
- `BOX_ROW`: 凿子盒行数 (默认 4)
- `BOX_COLUMN`: 凿子盒列数 (默认 6)
- `XMIN`, `XMAX`, `YMIN`, `YMAX`, `ZMIN`, `ZMAX`: 工作空间边界
- `BORDER_WIDTH`: 边界宽度 (默认 0.03)
- `NORM_TH`: 法向量阈值 (默认 0.95)
- `AFFECT_RADIUS`: 影响半径 (默认 0.0009)
- `HEIGHT_WEIGHT`: 高度权重 (默认 3.0)
- `CURV_WEIGHT`: 曲率权重 (默认 2.0)
- `ANGLE_WEIGHT`: 角度权重 (默认 1.0)
- `SEARCH_RADIUS`: 搜索半径 (默认 0.03)
- `SEARCH_NUM_TH`: 搜索数量阈值 (默认 100)
- `TH_DEEP`: 深度阈值 (默认 0.4)
- `TH_DEEPNORM`: 深度法向量阈值 (默认 0.8)
- `TH_HALF`: 半满阈值 (默认 50)
- `TH_FULL`: 全满阈值 (默认 100)
- `TH_ANGLE`: 角度阈值 (默认 0.99)

### snap_7 参数

- `plc_address`: PLC IP 地址 (默认 192.168.1.36)
- `plc_rack`: PLC 机架号 (默认 0)
- `plc_slot`: PLC 插槽号 (默认 1)
- `db_number`: 数据块号 (默认 2110)
- `db_start`: 数据块起始地址 (默认 0)
- `poll_rate`: 读取频率 (默认 20.0 Hz)

## 触发值说明 (PLC 通信协议)

- `100`: 启动相机
- `105`: 强制关闭相机
- `110`: 触发法向量计算
- `115`: 写入第二页数据
- `116`: 重写第一页数据
- `120`: 关闭相机
- `121`: 重写第二页数据
- `130`: 通信结束
- `200`: 清空缓冲区
- `210`: 第一页消息状态写入
- `215`: 第二页消息状态写入
- `230`: 通信阶段结束状态
- `99`: 节点退出

## 开发约定

- C++ 代码使用 ROS2 C++ 客户端库 (rclcpp)
- Python 代码使用 ROS2 Python 客户端库 (rclpy)
- 传感器数据使用标准 ROS2 消息类型 (sensor_msgs, geometry_msgs)
- 使用 PCL 进行点云处理
- 代码遵循 ROS2 包结构和命名约定
- 采用多线程执行器处理并发请求
- 坐标变换使用 Eigen 库进行矩阵运算
- 法向量计算涉及点云滤波、法线估计、曲率计算等步骤
- 相机坐标系到工具坐标系的转换使用预设变换矩阵