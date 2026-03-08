cmake_minimum_required(VERSION 3.5)
project(domain_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
# Leverage rosbag2's generic type support utilities
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CompressedMsg.msg
)

add_library(${PROJECT_NAME}_lib SHARED
  src/${PROJECT_NAME}/component_manager.cpp
  src/${PROJECT_NAME}/compress_messages.cpp
  src/${PROJECT_NAME}/domain_bridge.cpp
  src/${PROJECT_NAME}/domain_bridge_options.cpp
  src/${PROJECT_NAME}/parse_domain_bridge_yaml_config.cpp
  src/${PROJECT_NAME}/qos_options.cpp
  src/${PROJECT_NAME}/service_bridge_options.cpp
  src/${PROJECT_NAME}/topic_bridge_options.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(CMAKE_COMPILER_IS_GNUCXX)
  target_link_libraries(${PROJECT_NAME}_lib stdc++fs)
endif()

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  rcutils
  rosbag2_cpp
  rosidl_typesupport_cpp
  yaml_cpp_vendor
  zstd
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_exec
  src/domain_bridge.cpp
)

ament_target_dependencies(${PROJECT_NAME}_exec
  "rclcpp_components")

set_target_properties(${PROJECT_NAME}_exec PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

target_link_libraries(${PROJECT_NAME}_exec
  ${PROJECT_NAME}_lib
)

# Only needed to build domain_bridge_rti_qos executable.
find_package(rti_connext_dds_cmake_module QUIET)
find_package(rcpputils QUIET)

set(executables ${PROJECT_NAME}_exec)

if(rti_connext_dds_cmake_module_FOUND AND rcpputils_FOUND)
  rti_find_connextpro()
  if(RTIConnextDDS_FOUND)
    add_executable(${PROJECT_NAME}_rti_exec
      src/domain_bridge_rti_qos.cpp
    )
    set_target_properties(${PROJECT_NAME}_rti_exec
      PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_rti_qos PREFIX "")
    target_link_libraries(${PROJECT_NAME}_rti_exec
      ${PROJECT_NAME}_lib
      RTIConnextDDS::c_api
    )

    ament_target_dependencies(${PROJECT_NAME}_rti_exec
      rcpputils
    )
    list(APPEND executables ${PROJECT_NAME}_rti_exec)
  endif()
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${executables}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY examples launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rcutils
  yaml_cpp_vendor
  zstd_vendor
  zstd
)

ament_package()
