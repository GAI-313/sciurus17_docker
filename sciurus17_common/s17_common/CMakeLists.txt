cmake_minimum_required(VERSION 3.8)
project(s17_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)
add_executable(pose_srvs src/pose_srvs.cpp)
ament_target_dependencies(pose_srvs rclcpp sensor_msgs trajectory_msgs control_msgs std_srvs)

install(TARGETS
  pose_srvs
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()