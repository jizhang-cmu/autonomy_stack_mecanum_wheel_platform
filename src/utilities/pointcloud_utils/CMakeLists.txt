cmake_minimum_required(VERSION 3.8)
project(pointcloud_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Aggregate pointcloud node
add_executable(aggregate_pointcloud
  src/aggregate_pointcloud.cpp
)

ament_target_dependencies(aggregate_pointcloud
  rclcpp
  sensor_msgs
  std_srvs
  pcl_conversions
  pcl_ros
)

target_link_libraries(aggregate_pointcloud
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS
  aggregate_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
