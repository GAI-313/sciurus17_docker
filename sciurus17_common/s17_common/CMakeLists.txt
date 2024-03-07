cmake_minimum_required(VERSION 3.8)
project(s17_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(angles REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(s17_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${s17_msgs_INCLUDE_DIRS}
)

add_executable(pose_srvs src/pose_srvs.cpp)
add_executable(moveit2_l_control src/moveit2_l_control.cpp)
add_executable(moveit2_r_control src/moveit2_r_control.cpp)
#add_executable(moveit2_control src/moveit2_control.cpp src/pose_presets.cpp)
ament_target_dependencies(pose_srvs rclcpp sensor_msgs trajectory_msgs control_msgs std_srvs)
ament_target_dependencies(moveit2_l_control rclcpp moveit_ros_planning_interface geometry_msgs s17_msgs)
ament_target_dependencies(moveit2_r_control rclcpp moveit_ros_planning_interface geometry_msgs s17_msgs)
#ament_target_dependencies(moveit2_control rclcpp moveit_ros_planning_interface angles tf2_geometry_msgs)

install(TARGETS
  pose_srvs
  moveit2_l_control
  moveit2_r_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/set_pose.py
  scripts/gaze_table.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
