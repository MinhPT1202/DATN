# Bilateral Teleoperation for Mobile Robot (Novint Falcon)

## Giới thiệu dự án
Hệ thống điều khiển từ xa song phương cho robot di động sử dụng thiết bị haptic Novint Falcon. Người điều khiển nhận phản hồi lực trực tiếp từ môi trường.

## Công nghệ sử dụng
| Thành phần | Phiên bản | Mô tả |
|-----------|-----------|-------|
| ROS 2 | Jazzy | Nền tảng giao tiếp |
| Gazebo | Harmonic | Môi trường mô phỏng |
| Novint Falcon | Haptic Device | Thiết bị điều khiển song phương |

## Cài đặt ROS 2 Jazzy
Ubuntu 24.04  
Tài liệu:  
https://docs.ros.org/en/jazzy/index.html

## Cài đặt Gazebo Harmonic
```bash
sudo apt install ros-jazzy-gzharmonic

## Clone project
'''bash
cd ~/ros2_ws/src
git clone https://github.com/MinhPT1202/DATN.git

## build workspace
'''bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

