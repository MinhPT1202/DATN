#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, List

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class FictitiousForceNode(Node):
    """
    Tính lực hư cấu fv theo mục 5 của bài báo.
    Đầu vào: /odom, /scan
    Đầu ra:  /fictitious_force  (Float64, fv theo trục x của robot)
    """

    def __init__(self) -> None:
        super().__init__('fictitious_force_node')

        # ===== Parameters =====
        self.declare_parameter('robot_width', 0.30)       # c  [m]
        self.declare_parameter('delta', 0.20)             # δ  [m]
        self.declare_parameter('force_gain', 12.0)        # k
        self.declare_parameter('force_max', 0.9)          # |fv|max
        self.declare_parameter('alpha', 0.7)              # low-pass 0..1
        self.declare_parameter('s_max', 0.7)              # look-ahead [m]
        self.declare_parameter('forward_angle', 1.5)      # ±rad sector
        self.declare_parameter('min_speed', 0.01)         # v min để kích fv

        self.c: float = float(self.get_parameter('robot_width').value)
        self.delta: float = float(self.get_parameter('delta').value)
        self.k: float = float(self.get_parameter('force_gain').value)
        self.fv_max: float = float(self.get_parameter('force_max').value)
        self.alpha: float = float(self.get_parameter('alpha').value)
        self.s_max: float = float(self.get_parameter('s_max').value)
        self.forward_angle: float = float(
            self.get_parameter('forward_angle').value
        )
        self.min_speed: float = float(self.get_parameter('min_speed').value)

        # ===== Robot state =====
        self.v: float = 0.0
        self.omega: float = 0.0

        # ===== Filtered fictitious force =====
        self.fv_fil: float = 0.0

        # For throttled logging
        self.log_counter: int = 0

        # ===== Subscriptions =====
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # ===== Publisher =====
        self.pub_fv = self.create_publisher(Float64, '/fictitious_force', 10)

        self.get_logger().info('FictitiousForceNode started')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def odom_cb(self, msg: Odometry) -> None:
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def scan_cb(self, scan: LaserScan) -> None:
        # Không tiến lên -> không tạo lực hư cấu
        if self.v <= self.min_speed:
            self.fv_fil = 0.0
            self.publish_fv()
            return

        if not scan.ranges:
            return

        v = self.v
        omega = self.omega
        c = self.c
        delta = self.delta
        k = self.k

        # ----- Bán kính quỹ đạo dự đoán -----
        if abs(omega) < 1e-3:
            r = math.inf      # coi như đi thẳng
        else:
            r = abs(v / omega)

        ranges = np.asarray(scan.ranges, dtype=float)
        idx = np.isfinite(ranges) & (ranges > 0.0)
        if not np.any(idx):
            self.fv_fil = 0.0
            self.publish_fv()
            return

        ranges = ranges[idx]
        angles = scan.angle_min + np.arange(len(scan.ranges))[idx] * scan.angle_increment

        fv_raw = 0.0
        valid_count = 0

        for l_i, theta_i in zip(ranges, angles):
            # chỉ xét vùng phía trước
            if not (-self.forward_angle < theta_i < self.forward_angle):
                continue

            # ---------- Hình học ----------
            if not math.isfinite(r):
                # Đi thẳng
                s_i = l_i * math.cos(theta_i)
                d_i = abs(l_i * math.sin(theta_i))

            else:
                # Quay cong, r = |v/omega|
                # Đổi chiều góc theo hướng quay
                theta_eff = -theta_i if omega < 0.0 else theta_i

                # Khoảng cách từ tâm quỹ đạo đến điểm đo
                tmp = l_i * l_i + r * r - 2.0 * l_i * r * math.cos(1.57 - theta_eff)
                if tmp <= 0.0:
                    continue

                center_to_li = math.sqrt(tmp)
                d_i = abs(center_to_li - r)  # khoảng cách tới quỹ đạo

                denom = 2.0 * center_to_li * r
                if denom <= 0.0:
                    continue

                cos_phi = (center_to_li * center_to_li + r * r - l_i * l_i) / denom
                cos_phi = max(-1.0, min(1.0, cos_phi))
                phi = math.acos(cos_phi)

                s_i = abs(r * phi)  # chiều dài cung từ robot đến điểm

            # ---------- Trọng số p_i(d_i) ----------
            if d_i <= 0.5 * c:
                p_i = 1.0
            elif d_i < 0.5 * c + delta:
                p_i = 0.5 * (1.0 + math.cos(math.pi * (d_i - 0.5 * c) / delta))
            else:
                continue  # p_i = 0

            # Chỉ xét vật cản trong khoảng [0, s_max]
            if s_i < self.s_max:
                fv_raw += p_i * (self.s_max - s_i)
                valid_count += 1

        # Nếu có tia hợp lệ -> tính fv, ngược lại fv = 0
        if valid_count > 0:
            fv_raw = k * fv_raw / float(valid_count)
        else:
            fv_raw = 0.0

        # ---------- Low-pass filter ----------
        self.fv_fil = self.alpha * self.fv_fil + (1.0 - self.alpha) * fv_raw

        # ---------- Saturation ----------
        self.fv_fil = max(-self.fv_max, min(self.fv_fil, self.fv_max))

        # ---------- Publish ----------
        self.publish_fv()

        # Log thưa (mỗi ~20 callback)
        self.log_counter += 1
        if self.log_counter >= 20:
            self.log_counter = 0
            self.get_logger().info(
                f'v={self.v:.3f}, omega={self.omega:.3f}, fv={self.fv_fil:.3f}'
            )

    # ------------------------------------------------------------------ #

    def publish_fv(self) -> None:
        msg = Float64()
        msg.data = float(self.fv_fil)
        self.pub_fv.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
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
