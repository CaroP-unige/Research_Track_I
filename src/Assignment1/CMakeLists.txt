cmake_minimum_required(VERSION 3.8)
project(Assignment1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------
add_executable(node1 src/node1.cpp)
add_executable(node2 src/node2.cpp)  

ament_target_dependencies(node1
  rclcpp
  geometry_msgs
  turtlesim
)

ament_target_dependencies(node2
  rclcpp
  geometry_msgs
  std_msgs
  turtlesim
)

# ------------------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------------------
install(TARGETS
  node1
  node2
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing / Linting
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Package
# ------------------------------------------------------------------------------
ament_package()
