cmake_minimum_required(VERSION 3.8)
project(ee_pose_broadcaster)

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/ee_pose_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  geometry_msgs
  std_msgs
  builtin_interfaces
)

# Eigen không dùng ament_target_dependencies, link thẳng target
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# visibility + pluginlib
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "EE_POSE_BROADCASTER_BUILDING_DLL"
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

pluginlib_export_plugin_description_file(controller_interface ee_pose_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  controller_interface
  rclcpp_lifecycle
  geometry_msgs
  realtime_tools
  Eigen3
  builtin_interfaces
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
