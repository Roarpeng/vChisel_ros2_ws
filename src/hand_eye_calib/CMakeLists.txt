cmake_minimum_required(VERSION 3.8)
project(hand_eye_calib)

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
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/HandEyeCalibData.srv"
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(include)

# Executable
add_executable(hand_eye_calib_server
  src/hand_eye_calib_srv.cpp
  src/hand_eye_calib.cpp
)

# Link OpenCV
target_link_libraries(hand_eye_calib_server
  ${OpenCV_LIBS}
)

# Add ROS dependencies
ament_target_dependencies(hand_eye_calib_server
  rclcpp
  geometry_msgs
)

# Add dependency on generated service headers
rosidl_target_interfaces(hand_eye_calib_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install
install(TARGETS
  hand_eye_calib_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/hand_eye_bringup.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()