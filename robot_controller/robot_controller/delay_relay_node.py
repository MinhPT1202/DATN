#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collections import deque
import random

from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class DelayRelayNode(Node):
    def __init__(self):
        super().__init__('delay_relay_node')

        # Thời gian delay
        self.h1 = 0.0
        self.h2 = 0.0

        # Hàng đợi cho từng loại message
        self.queue_master = deque()
        self.queue_odom = deque()
        self.queue_ficforce = deque()

        # Subscriptions
        self.create_subscription(DynamicJointState,
                                 '/fd/dynamic_joint_states',
                                 self.master_callback,
                                 10)

        self.create_subscription(Odometry,
                                 '/diff_cont/odom',
                                 self.odom_callback,
                                 10)
        
        self.create_subscription(Float64,
                                 '/fictitious_force',
                                 self.ficforce_cb,
                                 10)

        # Publishers (topic sau delay)
        self.pub_master = self.create_publisher(DynamicJointState,
                                                '/delayed/fd/dynamic_joint_states',
                                                10)

        self.pub_odom = self.create_publisher(Odometry,
                                              '/delayed/diff_cont/odom',
                                              10)
        
        self.pub_ficforce = self.create_publisher(Float64,
                                              '/delayed/fictitious_force',
                                              10)


        # Timer xử lý gửi sau delay
        self.create_timer(0.005, self.timer_callback)  # 200 Hz

    def master_callback(self, msg : DynamicJointState):
        now = self.get_clock().now()
        self.queue_master.append((now, msg))

    def odom_callback(self, msg : Odometry):
        now = self.get_clock().now()
        self.queue_odom.append((now, msg))

    def ficforce_cb(self, msg : Float64):
        now = self.get_clock().now()
        self.queue_ficforce.append((now, msg))


    def timer_callback(self):
        
        #Random time delay
        self.h1 = random.uniform(0.15, 0.4)   # 150-400ms
        self.h2 = random.uniform(0.15, 0.4)  # 150-400ms
        now = self.get_clock().now()
        
        # Xử lý queue master với fixed delay
        while self.queue_master:
            t, msg = self.queue_master[0]
            if (now - t).nanoseconds * 1e-9 >= self.h1:
                self.pub_master.publish(msg)
                self.queue_master.popleft()
            else:
                break

        # Xử lý queue odom với variable delay
        while self.queue_odom:
            t, msg = self.queue_odom[0]
            if (now - t).nanoseconds * 1e-9 >= self.h2:
                self.pub_odom.publish(msg)
                self.queue_odom.popleft()
            else:
                break

        while self.queue_ficforce:
            t, msg = self.queue_ficforce[0]
            if (now - t).nanoseconds * 1e-9 >= self.h2:
                self.pub_ficforce.publish(msg)
                self.queue_ficforce.popleft()
            else:
                break


def main(args=None):
    rclpy.init(args=args)
    node = DelayRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()