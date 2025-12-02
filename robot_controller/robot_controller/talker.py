#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker")
        self.talker_publisher_ = self.create_publisher(Float64,"/chatter_delayed",10)
        self.timer_ = self.create_timer(0.1,self.talker_callback)


    def talker_callback(self):
        msg = Float64()
        msg.data = float (self.get_clock().now().nanoseconds) / 1e6
        self.talker_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    rclpy.shutdown()