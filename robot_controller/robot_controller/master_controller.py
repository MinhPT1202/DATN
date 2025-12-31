#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class MasterControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('master_controller')

        # ===== Hệ số điều khiển =====
        # Đồng bộ vận tốc
        self.k_m = [15.0, 6.0]

        # Map vị trí tay -> vận tốc tham chiếu robot
        self.k_g = [10.0, 14.0]

        # Damping tay master
        self.alpha_m = [15.0, 12.0]

        # Spring (độ cứng) tay master – nên rất nhỏ
        self.k_p = [0.01, 0.01]

        # Giới hạn lực cho an toàn
        self.max_force = 10.0

        # Trạng thái master & robot
        self.v = 0.0
        self.omega = 0.0
        self.pos_x = 0.0
        self.vel_x = 0.0
        self.pos_y = 0.0
        self.vel_y = 0.0

        # Master (Falcon) không delay
        self.master_subscription = self.create_subscription(
            DynamicJointState,
            '/fd/dynamic_joint_states',
            self.master_callback,
            10,
        )

        # Robot (nếu muốn Case C đúng bài thì đổi thành '/delayed/odom')
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
        self.pos_x = msg.interface_values[0].values[0]
        self.vel_x = msg.interface_values[0].values[1]
        self.pos_y = msg.interface_values[1].values[0]
        self.vel_y = msg.interface_values[1].values[1]

    def slave_callback(self, msg: Odometry) -> None:
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def timer_callback(self) -> None:
        # Đổi dấu do quy ước
        pos_x = -self.pos_x
        vel_x = -self.vel_x
        pos_y = -self.pos_y
        vel_y = -self.vel_y

        v = self.v
        omega = self.omega

        # Vận tốc tham chiếu
        v_ref = self.k_g[0] * pos_x
        omega_ref = self.k_g[1] * pos_y

        e_v = v_ref - v
        e_omega = omega_ref - omega

        # Lực phản hồi
        f_x = (
            -self.k_m[0] * e_v
            - self.alpha_m[0] * vel_x
            - self.k_p[0] * pos_x      # SPRING đúng: nhân với vị trí
        )
        f_y = (
            -self.k_m[1] * e_omega
            - self.alpha_m[1] * vel_y
            - self.k_p[1] * pos_y      # SPRING đúng: nhân với vị trí
        )

        # Giới hạn lực
        f_x = max(-self.max_force, min(self.max_force, f_x))
        f_y = max(-self.max_force, min(self.max_force, f_y))

        # Gửi về Falcon (đảo dấu lại theo driver)
        msg = Float64MultiArray()
        msg.data = [-f_x, -f_y, 0.0]
        self.force_master_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MasterControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
