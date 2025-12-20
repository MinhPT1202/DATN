#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from control_msgs.msg import DynamicJointState
import csv
import time
import matplotlib.pyplot as plt


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.force_sub = self.create_subscription(
            Float64MultiArray,
            '/fd/fd_controller/commands',
            self.force_cb,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )
        self.fictitious_sub = self.create_subscription(
            Float64,
            '/delayed/fictitious_force',
            self.fictitious_cb,
            10
        )
        self.master_sub = self.create_subscription(
            DynamicJointState,
            '/delayed/fd/dynamic_joint_states',
            self.master_cb,
            10
        )

        self.csv_file = open('/home/minh/ros2_ws/src/data/data_log_a.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time', 'f_x', 'f_y', 'f_v',
            'pos_x', 'pos_y', 'v', 'omega',
            'Tm2s', 'Ts2m'
        ])

        # Trạng thái hiện tại
        self.f_m = [0.0, 0.0]  # [f_x, f_y]
        self.f_v = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.h1 = 0.0
        self.h2 = 0.0
        self.start_time = time.time()

        # Bộ nhớ cho việc vẽ đồ thị
        self.time_data = []
        self.pos_x_data = []
        self.pos_y_data = []
        self.fx_data = []
        self.fy_data = []

        # Timer 100 Hz
        self.timer = self.create_timer(0.01, self.log_data)

    def force_cb(self, msg: Float64MultiArray):
        # Lấy 2 thành phần đầu làm f_x, f_y
        self.f_m = msg.data[:2]

    def fictitious_cb(self, msg: Float64):
        self.f_v = msg.data

    def odom_cb(self, msg: Odometry):
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def master_cb(self, msg: DynamicJointState):
        # Giả định interface_values[0], [1] lần lượt là pos_x, pos_y
        self.pos_x = msg.interface_values[0].values[0]
        self.pos_y = msg.interface_values[1].values[0]
        # Độ trễ một chiều
        self.h1 = time.time() - (msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)

    def log_data(self):
        now = time.time() - self.start_time
        fx, fy = self.f_m

        # Ghi vào CSV
        row = [
            now,
            fx, fy,
            self.f_v,
            self.pos_x, self.pos_y,
            self.v, self.omega,
            self.h1,
            self.h2  # vẫn để cột Ts2m, hiện chưa dùng
        ]
        self.writer.writerow(row)

        # Lưu dữ liệu để vẽ
        self.time_data.append(now)
        self.pos_x_data.append(self.pos_x)
        self.pos_y_data.append(self.pos_y)
        self.fx_data.append(fx)
        self.fy_data.append(fy)

    def plot_data(self):
        if not self.time_data:
            return

        # Đồ thị quỹ đạo (pos_x, pos_y)
        plt.figure()
        plt.plot(self.pos_x_data, self.pos_y_data)
        plt.xlabel('pos_x')
        plt.ylabel('pos_y')
        plt.title('Trajectory (pos_x, pos_y)')
        plt.grid(True)
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig('/home/minh/ros2_ws/src/data/image/pos_xy.png')
        plt.close()

        # Đồ thị f_x theo thời gian
        plt.figure()
        plt.plot(self.time_data, self.fx_data)
        plt.xlabel('time [s]')
        plt.ylabel('f_x')
        plt.title('Force f_x vs time')
        plt.grid(True)
        plt.tight_layout()
        plt.savefig('/home/minh/ros2_ws/src/data/image/fx_t.png')
        plt.close()

        # Đồ thị f_y theo thời gian
        plt.figure()
        plt.plot(self.time_data, self.fy_data)
        plt.xlabel('time [s]')
        plt.ylabel('f_y')
        plt.title('Force f_y vs time')
        plt.grid(True)
        plt.tight_layout()
        plt.savefig('/home/minh/ros2_ws/src/data/image/fy_t.png')
        plt.close()

    def destroy_node(self):
        # Vẽ và lưu đồ thị trước khi đóng file
        try:
            self.plot_data()
        except Exception as e:
            # Nếu lỗi vẽ thì vẫn đóng file và hủy node bình thường
            self.get_logger().error(f'Plot error: {e}')
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
