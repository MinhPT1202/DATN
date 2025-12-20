#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TalkerNode(Node):
    def __init__(self):
        super().__init__("cal_delay")
        self.counter_ = 0
        self.talker_sub_ = self.create_subscription(Float64,"/chatter_delayed",self.talker_callback,10)


    def talker_callback(self, msg : Float64):
        now = float (self.get_clock().now().nanoseconds) / 1e6
        time_delay = now - msg.data
        self.get_logger().info(f"Time delay: {time_delay}")

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    rclpy.shutdown()
