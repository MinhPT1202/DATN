#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToAckermannControllers(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_ackermann")

        # Tham số cơ bản (có thể override bằng parameter.yaml)
        self.declare_parameter("wheelbase", 0.3)          # khoảng cách trục trước–sau [m]
        self.declare_parameter("wheel_radius", 0.04)      # bán kính bánh drive [m]
        self.declare_parameter("max_steering_angle", 0.6)
        self.declare_parameter("max_speed", 2.0)
        # Tăng thêm offset 0.5 vào tốc độ linear
        self.declare_parameter("linear_offset", 0.5)

        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.linear_offset = float(self.get_parameter("linear_offset").value)

        # Sub /cmd_vel (ABSOLUTE TOPIC)
        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_cb,
            10,
        )

        # Pub tới 2 controller
        self.pub_vel = self.create_publisher(
            Float64MultiArray,
            "/forward_velocity_controller/commands",
            10,
        )
        self.pub_steer = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10,
        )

    def cmd_vel_cb(self, msg: Twist):
        # Tốc độ thô từ /cmd_vel
        v_raw = msg.linear.x
        omega = msg.angular.z

        # Cộng thêm 0.5 vào tốc độ linear
        v = v_raw + self.linear_offset

        # Giới hạn tốc độ
        if v > self.max_speed:
            v = self.max_speed
        elif v < -self.max_speed:
            v = -self.max_speed

        # Tính góc lái đơn giản
        if abs(v) > 1e-3:
            steer = math.atan(self.wheelbase * omega / v)
        else:
            steer = 0.0

        # Giới hạn góc lái
        if steer > self.max_steering_angle:
            steer = self.max_steering_angle
        elif steer < -self.max_steering_angle:
            steer = -self.max_steering_angle

        # Tốc độ góc bánh sau (2 bánh cùng omega_wheel)
        omega_wheel = v / self.wheel_radius

        vel_msg = Float64MultiArray()
        vel_msg.data = [omega_wheel, omega_wheel]

        steer_msg = Float64MultiArray()
        steer_msg.data = [steer, steer]

        self.pub_vel.publish(vel_msg)
        self.pub_steer.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermannControllers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
