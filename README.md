# Bilateral Teleoperation for Mobile Robot (Novint Falcon)

## Giới Thiệu Dự Án

Dự án này triển khai hệ thống **Điều khiển Từ xa Song phương (Bilateral Teleoperation)** cho một Robot Di động. Hệ thống này cho phép người điều khiển cảm nhận được lực phản hồi từ môi trường hoặc từ chính robot, mang lại trải nghiệm điều khiển chân thực và hiệu quả hơn.

Thiết bị chính được sử dụng để điều khiển là bộ tay cầm phản hồi lực **Novint Falcon**.

---

## Công Nghệ Nền Tảng

| Thành phần | Phiên bản | Mô tả |
| :--- | :--- | :--- |
| **Hệ điều hành Robot (ROS)** | **ROS 2 Humble Hawksbill** | Khung phần mềm chính để giao tiếp giữa các thành phần. |
| **Mô phỏng** | **Gazebo** | Môi trường mô phỏng 3D để kiểm tra và phát triển hệ thống. |
| **Thiết bị Điều khiển** | **Novint Falcon** | Tay cầm phản hồi lực (Haptic Device) đóng vai trò là Master. |

---

## Cài Đặt ROS2 Humble

OS only : Ubuntu 22.04

Thực hiện theo hướng dẫn chính thức:  
[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)


---

## Cài Đặt Gazebo

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

---

## Cài Đặt Driver Novint Falcon

Để đảm bảo Novint Falcon hoạt động chính xác trong môi trường ROS 2 Humble, chúng tôi sử dụng driver tùy chỉnh.

**Driver được sử dụng:**
* **Repository:** [https://github.com/ICube-Robotics/forcedimension_ros2/tree/humble](https://github.com/ICube-Robotics/forcedimension_ros2/tree/humble)
* **Chi nhánh (Branch):** `humble`

**Lưu ý quan trọng:** Hãy làm theo **tất cả các hướng dẫn** chi tiết trong repo này để cài đặt các thư viện phụ thuộc và driver cấp thấp trước khi xây dựng package ROS.

---

## Các Bước Thực Hiện (Clone & Build)

Các bước này giả định bạn đã cài đặt **ROS 2 Humble** và các thư viện cần thiết cho driver Novint Falcon (như đã đề cập ở trên).

### 1. Clone Repository

Sử dụng `git` để sao chép dự án vào workspace (không gian làm việc) của bạn.

```bash
# Clone repo 
cd ~/ros2_ws/src
git clone https://github.com/Tuschos/bilateral_teleop.git
```

### 2. Build toàn bộ workspace

```bash
colcon build --symlink-install 
source install/setup.bash
```

### 3. Chạy mô phỏng Gazebo và node điều khiển

**Terminal 1: Launch Novint Falcon**

```bash
ros2 launch fd_bringup fd.launch.py
```

**Terminal 2: Chạy mô phỏng**

```bash
ros2 launch dhtbot_one launch_sim.launch.py 
```

**Terminal 3: Launch các node điều khiển**

```bash
ros2 launch robot_controller teleop_bilateral.launch.py
```

