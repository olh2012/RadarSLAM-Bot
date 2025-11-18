cmake_minimum_required(VERSION 3.8)
project(fishbot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# 安装目录
install(
  DIRECTORY
    urdf
    launch
    config
    worlds
    maps
  DESTINATION share/${PROJECT_NAME}/
)

# 安装Python脚本
install(
  PROGRAMS
    scripts/save_map.py
    scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# 代码检查
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
