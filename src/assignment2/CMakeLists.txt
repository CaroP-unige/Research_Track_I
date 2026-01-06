cmake_minimum_required(VERSION 3.8)
project(assignment2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -----------------------------------------
# Generate interfaces (msg + srv)
# -----------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/SetThreshold.srv"
  "srv/GetVelAvg.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------------------
# Node 1
# -----------------------------------------
add_executable(node1 src/node1.cpp)
ament_target_dependencies(node1
  rclcpp
  geometry_msgs
)

# Link generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(node1 ${cpp_typesupport_target})

# -----------------------------------------
# Node 2
# -----------------------------------------
add_executable(node2 src/node2.cpp)
ament_target_dependencies(node2
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Link generated interfaces
target_link_libraries(node2 ${cpp_typesupport_target})

# -----------------------------------------
# Install
# -----------------------------------------
install(TARGETS
  node1
  node2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()