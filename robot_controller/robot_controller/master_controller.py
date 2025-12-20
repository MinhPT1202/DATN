# master_controller.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class MasterControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('master_controller')

        # Hệ số điều khiển
        self.k_m = [15.0, 6.0]
        self.k_g = [10.0, 14.0] # bieu dien tuyen tinh tu vi tri endpoint den van toc cua mobile robot
        self.alpha_m = [18.0, 8.0] # daping
        self.k_p = [10.0, 5.0]  # spring

        # Trạng thái master & slave
        self.v = 0.0
        self.omega = 0.0
        self.pos_x = 0.0
        self.vel_x = 0.0
        self.pos_y = 0.0
        self.vel_y = 0.0
        self.f_m = [0.0, 0.0, 0.0]

        # Master (Falcon) không delay
        self.master_subscription = self.create_subscription(
            DynamicJointState,
            '/fd/dynamic_joint_states',
            self.master_callback,
            10,
        )

        # Robot sau delay
        self.slave_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.slave_callback,
            10,
        )

        # Lực gửi về Falcon
        self.force_master_publisher = self.create_publisher(
            Float64MultiArray,
            '/fd/fd_controller/commands',
            10,
        )

        # Vòng điều khiển 100 Hz
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def master_callback(self, msg: DynamicJointState) -> None:
        # fd_x
        self.pos_x = msg.interface_values[0].values[0]
        self.vel_x = msg.interface_values[0].values[1]
        # fd_y
        self.pos_y = msg.interface_values[1].values[0]
        self.vel_y = msg.interface_values[1].values[1]

    def slave_callback(self, msg: Odometry) -> None:
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def timer_callback(self) -> None:
        # Master position & velocity (đổi dấu do quy ước)
        pos_x = -self.pos_x
        vel_x = -self.vel_x
        pos_y = -self.pos_y
        vel_y = -self.vel_y

        # Robot velocities
        v = self.v
        omega = self.omega

        k_m = self.k_m
        k_g = self.k_g
        alpha_m = self.alpha_m
        k_p = self.k_p

        # Lực phản hồi theo luật đã chọn
        f_x = -k_m[0] * (k_g[0] * pos_x - v) - alpha_m[0] * vel_x - k_p[0] * vel_x  #luc master
        f_y = -k_m[1] * (k_g[1] * pos_y - omega) - alpha_m[1] * vel_y - k_p[1] * vel_y #luc slave

        # Trục z để 0
        self.f_m = [-f_x, -f_y, 0.0]

        msg = Float64MultiArray()
        msg.data = self.f_m
        self.force_master_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MasterControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()