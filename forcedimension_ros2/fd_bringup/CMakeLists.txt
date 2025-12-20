cmake_minimum_required(VERSION 3.8)
project(fd_bringup)

# Chỉ có launch, không build code C/C++

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
