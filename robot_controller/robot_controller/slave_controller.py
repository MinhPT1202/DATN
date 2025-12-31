#!/usr/bin/env python3
# slave_controller.py

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class SlaveControllerNode(Node):
    """
    Slave controller (mobile robot) – bám tay haptic + impedance fv.

    Luật điều khiển đơn giản:
        v_ref      = k_g_lin * x_m(t - h1)
        omega_ref  = k_g_ang * y_m(t - h1)

        v_cmd      = v_ref - k_z * f_v   (impedance theo lực hư cấu)
        omega_cmd  = omega_ref

    - Case A / B: đặt k_z = 0  => không impedance (f_v = 0).
    - Case C:     k_z > 0      => bật impedance.
    """

    def __init__(self) -> None:
        super().__init__('slave_controller')

        # ===== THAM SỐ ĐIỀU KHIỂN =====
        # Map vị trí tay -> vận tốc
        self.k_g_lin = self.declare_parameter('k_g_lin', 14.0).value
        self.k_g_ang = self.declare_parameter('k_g_ang', 14.0).value

        # Gain impedance (k_z). 0 -> tắt fv, >0 -> bật impedance.
        self.k_z = self.declare_parameter('k_z', 0.7).value

        # Giới hạn vận tốc
        self.max_v = self.declare_parameter('max_v', 0.8).value
        self.max_omega = self.declare_parameter('max_omega', 1.5).value

        # Dùng topic có delay hay không
        self.use_delay = self.declare_parameter('use_delay', True).value

        # ===== TRẠNG THÁI =====
        # Master (tay haptic sau delay h1)
        self.pos_x = 0.0   # x_m(t - h1)
        self.pos_y = 0.0

        # Robot
        self.v = 0.0
        self.omega = 0.0

        # Lực hư cấu LiDAR (sau delay h2)
        self.f_v = 0.0

        # ===== SUB / PUB =====
        if self.use_delay:
            master_topic = '/delayed/fd/dynamic_joint_states'
            odom_topic = '/delayed/odom'
            fv_topic = '/delayed/fictitious_force'
        else:
            master_topic = '/fd/dynamic_joint_states'
            odom_topic = '/odom'
            fv_topic = '/fictitious_force'

        # Master (haptic)
        self.master_subscription = self.create_subscription(
            DynamicJointState,
            master_topic,
            self.master_callback,
            10,
        )

        # Odom robot
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        # Lực hư cấu fv
        self.fv_subscription = self.create_subscription(
            Float64,
            fv_topic,
            self.fv_callback,
            10,
        )

        # Lệnh vận tốc cho diff_drive_controller
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        # Vòng điều khiển 100 Hz
        self.timer_ = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(
            f"SlaveControllerNode started (use_delay={self.use_delay}, k_z={self.k_z})"
        )

    # ==================== CALLBACKS ====================

    def master_callback(self, msg: DynamicJointState) -> None:
        # Vị trí tay cầm sau delay, đổi dấu cho cùng chiều với robot
        self.pos_x = -msg.interface_values[0].values[0]
        self.pos_y = -msg.interface_values[1].values[0]

    def odom_callback(self, msg: Odometry) -> None:
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def fv_callback(self, msg: Float64) -> None:
        self.f_v = msg.data

    # ==================== VÒNG ĐIỀU KHIỂN ====================

    def timer_callback(self) -> None:
        cmd = Twist()

        # Vận tốc tham chiếu từ master (đã có trễ h1 nếu use_delay=True)
        v_ref = self.k_g_lin * self.pos_x
        omega_ref = self.k_g_ang * self.pos_y

        # Impedance dựa trên fv (k_z = 0 -> Case A/B, k_z > 0 -> Case C)
        v_cmd = v_ref - self.k_z * self.f_v
        omega_cmd = omega_ref

        # Giới hạn vận tốc
        v_cmd = max(-self.max_v, min(self.max_v, v_cmd))
        omega_cmd = max(-self.max_omega, min(self.max_omega, omega_cmd))

        cmd.linear.x = v_cmd
        cmd.angular.z = omega_cmd

        self.cmd_vel_publisher.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlaveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
