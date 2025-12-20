cmake_minimum_required(VERSION 3.8)
project(dhtbot_one)

# C standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++ standards
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

# Install resources (no code built in this package)
install(
  DIRECTORY
    config
    description
    launch
    meshes
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
