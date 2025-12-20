#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class FictitiousForceNode(Node):
    def __init__(self) -> None:
        super().__init__('fictitious_force_node')

        # ===== Parameters =====
        self.declare_parameter('robot_width', 0.3)     # [m]
        self.declare_parameter('delta', 0.2)           # [m]
        self.declare_parameter('force_gain', 12.0)
        self.declare_parameter('force_max', 0.9)
        self.declare_parameter('alpha', 0.7)           # low-pass

        self.c = float(self.get_parameter('robot_width').value)
        self.delta = float(self.get_parameter('delta').value)
        self.k = float(self.get_parameter('force_gain').value)
        self.fv_max = float(self.get_parameter('force_max').value)
        self.alpha = float(self.get_parameter('alpha').value)

        self.s_max = 0.7  # [m] look-ahead along path

        # ===== Robot state =====
        self.v = 0.0
        self.omega = 0.0

        # ===== Filtered fictitious force =====
        self.fv_fil = 0.0

        # ===== Subscriptions =====
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10
        )

        # ===== Publisher =====
        self.pub_fv = self.create_publisher(
            Float64,
            '/fictitious_force',
            10
        )

        self.get_logger().info("FictitiousForceNode started")

    # ======================================================================

    def odom_cb(self, msg: Odometry) -> None:
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    # ======================================================================

    def scan_cb(self, scan: LaserScan) -> None:
        if not scan.ranges:
            return

        v = self.v
        omega = self.omega
        c = self.c
        delta = self.delta
        k = self.k

        # ===== Predicted path radius =====
        if abs(omega) < 1e-3:
            r = math.inf
        else:
            r = abs(v / omega)

        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        fv_raw = 0.0
        valid_count = 0

        for l_i, theta_i in zip(ranges, angles):
            if not math.isfinite(l_i) or l_i <= 0.0:
                continue

            # only forward sector
            if not (-1.5 < theta_i < 1.5):
                continue

            # ===== Geometry =====
            if not math.isfinite(r):
                s_i = l_i * math.cos(theta_i)
                d_i = abs(l_i * math.sin(theta_i))
            else:
                theta_eff = -theta_i if omega < 0.0 else theta_i

                tmp = (
                    l_i*l_i + r*r
                    - 2.0*l_i*r*math.cos(1.57 - theta_eff)
                )
                if tmp <= 0.0:
                    continue

                center_to_li = math.sqrt(tmp)
                d_i = abs(center_to_li - r)

                denom = 2.0 * center_to_li * r
                if denom <= 0.0:
                    continue

                cos_phi = (
                    center_to_li*center_to_li + r*r - l_i*l_i
                ) / denom
                cos_phi = max(-1.0, min(1.0, cos_phi))
                phi = math.acos(cos_phi)

                s_i = abs(r * phi)

            # ===== Weight p_i =====
            if d_i <= c * 0.5:
                p_i = 1.0
            elif d_i < c * 0.5 + delta:
                p_i = 0.5 * (
                    1.0 + math.cos(
                        math.pi * (d_i - c * 0.5) / delta
                    )
                )
            else:
                continue

            if s_i < self.s_max:
                fv_raw += p_i * (self.s_max - s_i)
                valid_count += 1

        if valid_count > 0:
            fv_raw = k * fv_raw / valid_count
        else:
            fv_raw = 0.0

        # ===== Low-pass filter =====
        self.fv_fil = (
            self.alpha * self.fv_fil
            + (1.0 - self.alpha) * fv_raw
        )

        # ===== Saturation =====
        self.fv_fil = max(
            -self.fv_max,
            min(self.fv_fil, self.fv_max)
        )

        # ===== Publish =====
        msg = Float64()
        msg.data = float(self.fv_fil)
        self.pub_fv.publish(msg)

        # ===== Debug log =====
        self.get_logger().info(
            f"v={self.v:.3f}, omega={self.omega:.3f}, fv={self.fv_fil:.3f}"
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FictitiousForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
