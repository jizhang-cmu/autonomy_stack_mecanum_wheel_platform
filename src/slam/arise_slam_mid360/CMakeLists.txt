cmake_minimum_required(VERSION 3.5)
project(arise_slam_mid360)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++1z")
set(CMAKE_CXX_FLAGS_RELEASE "-O4 -Wall -g")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(TBB_INCLUDE_DIR "/usr/include/tbb")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra )
endif()

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(arise_slam_mid360_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(TBB REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(jsoncpp REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${TBB_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${livox_ros_driver2_INCLUDE_DIRS}
)

# Link directories
link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

set(GTSAM_LIBRARIES gtsam)


# AriseSlam Library
add_library(AriseSlamLib SHARED
  src/FeatureExtraction/featureExtraction.cpp
  src/FeatureExtraction/LidarKeypointExtractor.cpp
  src/FeatureExtraction/DepthImageKeypointExtractor.cpp
  src/ImuPreintegration/imuPreintegration.cpp
  
  src/LaserMapping/laserMapping.cpp
  src/LaserMapping/lidarOptimization.cpp
  src/LaserMapping/LocalMap.cpp
  src/LidarProcess/LidarSlam.cpp
  src/LidarProcess/pose_local_parameterization.cpp
  src/LidarProcess/SE3AbsolutatePoseFactor.cpp
  src/parameter/parameter.cpp
)

# Set up library include directory
target_include_directories(AriseSlamLib PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/AriseSlamLib>  
    $<INSTALL_INTERFACE:include/AriseSlamLib>  # <prefix>/include/mylib
)

target_link_libraries(AriseSlamLib
  rclcpp::rclcpp
  std_msgs::std_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  sensor_msgs::sensor_msgs
  image_transport::image_transport
  cv_bridge::cv_bridge
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  arise_slam_mid360_msgs::arise_slam_mid360_msgs
  visualization_msgs::visualization_msgs
  pcl_conversions::pcl_conversions
  livox_ros_driver2::livox_ros_driver2
)
target_link_libraries(AriseSlamLib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
 ${CERES_LIBRARIES}
)

# IMU PreIntegration node
add_executable(imu_preintegration_node src/imuPreintegration_node.cpp)
target_link_libraries(imu_preintegration_node rclcpp::rclcpp nav_msgs::nav_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros)
target_link_libraries(imu_preintegration_node
    gtsam
    AriseSlamLib
)



# Feature Extraction node
add_executable(feature_extraction_node src/featureExtraction_node.cpp)
target_link_libraries(feature_extraction_node rclcpp::rclcpp nav_msgs::nav_msgs sensor_msgs::sensor_msgs arise_slam_mid360_msgs::arise_slam_mid360_msgs livox_ros_driver2::livox_ros_driver2)
target_link_libraries(feature_extraction_node
  ${PCL_LIBRARIES}
  ${TBB_LIBRARIES}
  AriseSlamLib
)

# LiDAR Mapping node
add_executable(laser_mapping_node src/laserMapping_node.cpp)
target_link_libraries(laser_mapping_node rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs sensor_msgs::sensor_msgs geometry_msgs::geometry_msgs tf2::tf2 tf2_ros::tf2_ros)
target_link_libraries(laser_mapping_node
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    AriseSlamLib
)

# Install Exported Libraries
ament_export_targets(export_arise_slam HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS AriseSlamLib
  EXPORT export_arise_slam
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(AriseSlamLib)

#Install executables
install(TARGETS
  imu_preintegration_node
  feature_extraction_node
  laser_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
