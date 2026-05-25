cmake_minimum_required(VERSION 3.5)
project(teleop_joy_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

add_executable(teleopJoyController src/teleopJoyController.cpp)
target_link_libraries(teleopJoyController rclcpp::rclcpp std_msgs::std_msgs sensor_msgs::sensor_msgs geometry_msgs::geometry_msgs)
target_include_directories(teleopJoyController PRIVATE ${serial_INCLUDE_DIRS})
target_link_libraries(teleopJoyController ${serial_LIBRARIES})

install(TARGETS
  teleopJoyController
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
