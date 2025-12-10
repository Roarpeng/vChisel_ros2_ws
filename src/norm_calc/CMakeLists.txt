cmake_minimum_required(VERSION 3.8)
project(norm_calc)

# ------------------------------------------------------------
# 编译设置
cmake_policy(SET CMP0074 NEW)  # 启用 CMP0074 政策，使 PCL_ROOT 生效

# ------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")

# ------------------------------------------------------------
# 依赖包
# ------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ------------------------------------------------------------
# 输出OpenCV状态
# ------------------------------------------------------------
message(STATUS "OpenCV for norm_calc status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# ------------------------------------------------------------
# 接口定义（srv）
# ------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NormCalcData.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

# ------------------------------------------------------------
# 包含头文件路径
# ------------------------------------------------------------
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ------------------------------------------------------------
# 可执行文件
# ------------------------------------------------------------

# 主服务节点
add_executable(norm_calc_node
  src/norm_calc_server.cpp
  src/norm_calc.cpp
  src/chisel_box.cpp
  src/edge_grid.cpp
  src/hole_detector.cpp
)

# 可视化节点
add_executable(norm_viewer_node
  src/norm_viewer.cpp
)

# 新的图像和法向信息联合可视化节点
add_executable(image_norm_viewer_node
  src/image_norm_viewer.cpp
)

# ------------------------------------------------------------
# 依赖声明
# ------------------------------------------------------------
ament_target_dependencies(norm_calc_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  pcl_msgs
  PCL
  OpenCV
)

ament_target_dependencies(norm_viewer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  PCL
  message_filters
  pcl_conversions
  pcl_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(image_norm_viewer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  PCL
  message_filters
  pcl_conversions
  cv_bridge
  OpenCV
)

# ------------------------------------------------------------
# 链接库
# ------------------------------------------------------------
target_link_libraries(norm_calc_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(norm_viewer_node
  ${PCL_LIBRARIES}
)

target_link_libraries(image_norm_viewer_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# ------------------------------------------------------------
# 链接ROS2接口支持库
# ------------------------------------------------------------
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(norm_calc_node "${cpp_typesupport_target}")
target_link_libraries(norm_viewer_node "${cpp_typesupport_target}")

# ------------------------------------------------------------
# 安装目标
# ------------------------------------------------------------
install(TARGETS
  norm_calc_node
  norm_viewer_node
  image_norm_viewer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ------------------------------------------------------------
# 导出包
# ------------------------------------------------------------
ament_package()
