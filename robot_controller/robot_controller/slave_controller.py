# slave_controller.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class SlaveControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('slave_controller')

        # Hệ số điều khiển
        self.k_g = [10.0, 15.0]

        # Trạng thái robot
        self.v = 0.0
        self.omega = 0.0
        self.a_v = 0.0
        self.a_omega = 0.0

        # Trạng thái master
        self.pos_x = 0.0
        self.pos_y = 0.0

        # Lực ảo + lực điều khiển
        self.f_v = 0.0
        self.f_s = [0.0, 0.0]

        # Master (haptic) sau delay
        self.master_subscription = self.create_subscription(
            DynamicJointState,
            '/delayed/fd/dynamic_joint_states',
            self.master_callback,
            10,
        )

        # Odom robot
        self.slave_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.slave_callback,
            10,
        )

        # Lực ảo fv
        self.fv_subscription = self.create_subscription(
            Float64,
            '/fictitious_force',
            self.fv_callback,
            10,
        )

        # Lệnh vận tốc cho diff_drive_controller (Jazzy)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        # Vòng điều khiển 100 Hz
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def master_callback(self, msg: DynamicJointState) -> None:
        # Lấy vị trí tay cầm sau delay, đảo dấu để phù hợp quy ước
        self.pos_x = -msg.interface_values[0].values[0]
        self.pos_y = -msg.interface_values[1].values[0]

    def slave_callback(self, msg: Odometry) -> None:
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def fv_callback(self, msg: Float64) -> None:
        self.f_v = msg.data

    def timer_callback(self) -> None:
        cmd_vel = Twist()

        # Luật điều khiển slave
        cmd_vel.linear.x = self.k_g[0] * self.pos_x - self.f_v
        cmd_vel.angular.z = self.k_g[1] * self.pos_y

        self.cmd_vel_publisher.publish(cmd_vel)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlaveControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()