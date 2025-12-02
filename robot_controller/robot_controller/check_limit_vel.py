#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CheckLimitVelocityNode(Node):
    def __init__(self):
        super().__init__("check_limit_vel")

        self.wheel_min_vel = 0.18
        self.wheel_max_vel = 0.8
        self.robot_width = 0.175

        self.vel_in_sub = self.create_subscription(
            Twist, 
            "/cmd_vel", 
            self.vel_in_cb,
            10)

        self.vel_out_pub = self.create_publisher(
            Twist,
            "/diff_cont/cmd_vel_unstamped",
            10
        )

    def vel_in_cb(self, msg : Twist):
        vel_out = Twist()
        v = msg.linear.x
        omega = msg.angular.z
        
        # Tinh v moi banh
        v_right = v + self.robot_width / 2 * omega
        v_left = v - self.robot_width / 2 * omega
        self.get_logger().info(f"vr = {v_right:.3f}, vl = {v_left:.3f}")

        # Check vel
        if( not self.isValid_vel(v_right, self.wheel_min_vel, self.wheel_max_vel) ):
            v_right = 0.0
        if( not self.isValid_vel(v_left, self.wheel_min_vel, self.wheel_max_vel) ):
            v_left = 0.0

        # vel_out
        vel_out.linear.x = (v_right + v_left) / 2
        vel_out.angular.z = (v_right - v_left) / self.robot_width
        self.get_logger().info(f"v = {vel_out.linear.x:.3f}, omega = {vel_out.angular.z:.3f}")

        # Publish vel_out
        self.vel_out_pub.publish(vel_out)
        

    def isValid_vel(self, vel, v_min, v_max):
        return v_min <= abs(vel) <= v_max
        

def main(args=None):
    rclpy.init(args=args)
    node = CheckLimitVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
        
        