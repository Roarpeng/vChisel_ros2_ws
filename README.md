# vChisel_ros2_ws

这是一个基于ROS2的工业自动化视觉处理系统，主要用于点云处理和与西门子PLC通信。

## 项目概述

该项目包含两个主要的ROS2包：

1. **norm_calc**: 实现点云数据处理算法，包括滤波、法线计算、网格生成等
2. **snap_7**: 使用Snap7库实现与西门子PLC的通信

系统能够处理来自深度相机的点云数据，识别特定区域内的特征点，并通过PLC控制工业设备执行相应操作。

## 系统架构

```
vChisel_ros2_ws/
├── norm_calc/                 # 点云处理包
│   ├── include/norm_calc/     # 头文件
│   ├── src/                   # 源代码
│   ├── config/                # 配置文件
│   └── launch/                # 启动文件
└── snap_7/                    # PLC通信包
    ├── snap_7/                # Python节点实现
    ├── launch/                # 启动文件
    └── test/                  # 测试文件
```

## 主要功能

### norm_calc 包

- 点云预处理：直通滤波、体素下采样、统计滤波
- 法线计算：使用PCL库计算点云法线
- 特征提取：从处理后的点云中提取关键特征点
- 参数配置：支持多种处理参数的配置

### snap_7 包

- 与西门子PLC通信：使用Snap7库读写PLC数据块
- 触发机制：监听PLC特定地址值变化触发点云处理
- 结果反馈：将处理结果写回PLC指定地址

## 安装依赖

```bash
# ROS2 humble
sudo apt install ros-humble-desktop

# PCL库
sudo apt install libpcl-dev

# Snap7库
pip install python-snap7

# 其他依赖
sudo apt install ros-humble-cv-bridge \
                 ros-humble-pcl-conversions \
                 ros-humble-pcl-msgs \
                 ros-humble-message-filters
```

## 编译项目

```bash
cd vChisel_ros2_ws
colcon build --packages-select norm_calc snap_7
source install/setup.bash
```

## 运行系统

### 方法一：使用一键启动脚本

```bash
./start_system.sh
```

### 方法二：手动启动

```bash
# 终端1：启动norm_calc和PLC客户端节点
ros2 launch snap_7 snap_7.launch.py

# 如果需要单独启动norm_calc
ros2 launch norm_calc norm_calc_launch.py
```

## 配置说明

### norm_calc 参数

配置文件位于：`norm_calc/config/norm_calc_params.yaml`

主要参数：
- `BOX_*`: 网格划分参数
- `X/Y/Z MIN/MAX`: 坐标轴过滤范围
- `BORDER_WIDTH`: 边界宽度
- `NORM_TH`: 法线阈值
- `SEARCH_RADIUS`: 搜索半径
- `SEARCH_NUM_TH`: 最小邻居数阈值

### snap_7 参数

PLC通信相关参数：
- `plc_address`: PLC IP地址
- `db_number`: 数据块编号
- `db_start`: 起始偏移地址
- `poll_rate`: 轮询频率

## 使用流程

1. 确保PLC与运行节点的计算机网络连通
2. 在PLC中创建对应的数据块(DB)
3. 配置正确的DB编号和偏移地址
4. 启动系统
5. 当PLC中触发条件满足时(特定地址值变为110)，系统自动执行点云处理
6. 处理结果会写回到PLC指定地址

## 故障排除

### PLC通信问题

1. 确认PLC IP地址配置正确
2. 检查防火墙设置
3. 确保PLC中DB块配置正确且允许外部访问
4. 验证DB编号和偏移地址正确

### 点云处理问题

1. 检查点云数据源是否正常
2. 调整过滤参数适应具体场景
3. 查看日志输出定位问题

## 开发指南

### 添加新的点云处理算法

1. 在[norm_calc](file:///home/bosch/vChisel_ros2_ws/src/norm_calc/include/norm_calc/norm_calc.h#L57-L57)包中添加新算法实现
2. 修改[norm_calc.cpp](file:///home/bosch/vChisel_ros2_ws/src/norm_calc/src/norm_calc.cpp)整合新算法
3. 更新配置参数(如需要)

### 扩展PLC通信功能

1. 修改[plc_client_node.py](file:///home/bosch/vChisel_ros2_ws/src/snap_7/snap_7/plc_client_node.py)添加新的读写逻辑
2. 调整数据解析方式以适配不同的PLC数据结构