cmake_minimum_required(VERSION 3.8)
project(norm_calc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NormCalcData.srv"
  DEPENDENCIES
    sensor_msgs
    geometry_msgs
)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# norm_calc executable - renamed to avoid conflict
add_executable(norm_calc_server
  src/norm_calc_server.cpp 
  src/norm_calc.cpp 
  src/chisel_box.cpp 
  src/edge_grid.cpp 
  src/hole_detector.cpp
)

# Link libraries for norm_calc_server
target_link_libraries(norm_calc_server
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Add dependencies
ament_target_dependencies(norm_calc_server
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
)

# Add dependencies for generated messages
rosidl_target_interfaces(norm_calc_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# norm_viewer executable
add_executable(norm_viewer src/norm_viewer.cpp)

# Link libraries for norm_viewer
target_link_libraries(norm_viewer
  ${PCL_LIBRARIES}
)

# Add dependencies for norm_viewer
ament_target_dependencies(norm_viewer
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
)

# Add dependencies for generated messages
rosidl_target_interfaces(norm_viewer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# image_norm_viewer executable
add_executable(image_norm_viewer src/image_norm_viewer.cpp)

# Link libraries for image_norm_viewer
target_link_libraries(image_norm_viewer
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Add dependencies for image_norm_viewer
ament_target_dependencies(image_norm_viewer
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
)

# Add dependencies for generated messages
rosidl_target_interfaces(image_norm_viewer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  norm_calc_server
  norm_viewer
  image_norm_viewer
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION include/
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Package configuration
ament_package()