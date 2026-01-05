cmake_minimum_required(VERSION 3.10)
project(recipe_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Recipe.msg"
  "msg/RecipeInfo.msg"
  "srv/ListRecipes.srv"
  "srv/LoadRecipe.srv"
  "srv/SaveRecipe.srv"
  "srv/DeleteRecipe.srv"
  DEPENDENCIES std_msgs
)

# Install Python executables
install(PROGRAMS
  recipe_manager/recipe_node.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
