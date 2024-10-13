cmake_minimum_required(VERSION 3.8)
project(self_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(ros_ws/install/self_interface/include/self_interface)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Robot.msg"
  DEPENDENCIES geometry_msgs
)

add_executable(robot_publisher src/main.cpp)
ament_target_dependencies(robot_publisher rclcpp)

rosidl_target_interfaces(robot_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS robot_publisher 
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
