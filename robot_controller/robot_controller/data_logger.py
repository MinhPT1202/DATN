#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64 
from nav_msgs.msg import Odometry
from control_msgs.msg import DynamicJointState
import csv
import time

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.force_sub = self.create_subscription(Float64MultiArray, '/fd/fd_controller/commands', self.force_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_cb, 10)
        self.fictitious_sub = self.create_subscription(Float64, '/delayed/fictitious_force', self.fictitious_cb, 10)
        self.master_sub = self.create_subscription(DynamicJointState, '/delayed/fd/dynamic_joint_states',self.master_cb,10)

        self.csv_file = open('/tmp/data_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'f_x', 'f_y', 'f_v', 'pos_x', 'pos_y', 'v', 'omega','Tm2s', "Ts2m"])

        self.f_m = [0.0, 0.0]
        self.f_v = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.h1 = 0.0
        self.h2 = 0.0
        self.start_time = time.time()

        self.timer = self.create_timer(0.01, self.log_data)  # 100Hz

    def force_cb(self, msg : Float64MultiArray):
        self.f_m = msg.data[:2]

    def fictitious_cb(self, msg : Float64):
        self.f_v = msg.data

    def odom_cb(self, msg : Odometry):
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def master_cb(self, msg : DynamicJointState):
        self.pos_x = msg.interface_values[0].values[0]
        self.pos_y = msg.interface_values[1].values[0]
        self.h1 = time.time() - (msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)

    def log_data(self):
        now = time.time() - self.start_time
        row = [now, *self.f_m, self.f_v, self.pos_x, self.pos_y, self.v, self.omega, self.h1]
        self.writer.writerow(row)

    def destroy_node(self):
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
