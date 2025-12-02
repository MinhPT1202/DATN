Bilateral Teleoperation for Mobile Robot (Novint Falcon)

Giới thiệu dự án

Hệ thống điều khiển từ xa song phương cho robot di động sử dụng thiết bị haptic Novint Falcon. Người điều khiển nhận được phản hồi lực từ môi trường, đảm bảo thao tác chính xác và tự nhiên.

Công nghệ sử dụng
Thành phần	Phiên bản	Mô tả
ROS 2	Jazzy	Nền tảng giao tiếp chính giữa các node
Gazebo	Harmonic	Mô phỏng 3D robot
Thiết bị điều khiển	Novint Falcon	Thiết bị phản hồi lực đóng vai trò Master
Cài đặt ROS 2 Jazzy

Ubuntu 24.04
Tài liệu chính thức:
https://docs.ros.org/en/jazzy/index.html

Cài đặt Gazebo Harmonic
sudo apt install ros-jazzy-gzharmonic

Cài đặt driver Novint Falcon

Sử dụng driver từ repo sau (branch humble):

https://github.com/ICube-Robotics/forcedimension_ros2/tree/humble

Làm đúng toàn bộ hướng dẫn trong repo để cài dependency và driver mức thấp.

Clone và build workspace
1. Clone repo
cd ~/ros2_ws/src
git clone https://github.com/MinhPT1202/DATN.git

2. Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

Chạy hệ thống
Terminal 1 – Novint Falcon
ros2 launch fd_bringup fd.launch.py

Terminal 2 – mô phỏng Gazebo
ros2 launch dhtbot_one launch_sim.launch.py

Terminal 3 – điều khiển song phương
ros2 launch robot_controller teleop_bilateral.launch.py
