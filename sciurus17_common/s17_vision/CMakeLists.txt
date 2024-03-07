cmake_minimum_required(VERSION 3.8)
project(s17_vision)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(PCL 1.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(obstacle_detection src/obstacle_detection.cpp)
add_executable(space_finder src/space_finder.cpp)
ament_target_dependencies(obstacle_detection rclcpp sensor_msgs visualization_msgs)
ament_target_dependencies(space_finder rclcpp sensor_msgs geometry_msgs pcl_conversions std_srvs tf2_ros tf2_geometry_msgs)

target_link_libraries(obstacle_detection ${PCL_LIBRARIES})  # PCLのライブラリをリンク
target_link_libraries(space_finder ${PCL_LIBRARIES})  # PCLのライブラリをリンク

install(TARGETS
  space_finder
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
