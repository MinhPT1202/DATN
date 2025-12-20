#!/usr/bin/env python3
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


msg = """
Control Your Robot (ROS2)
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return f"currently:\tspeed {speed}\tturn {turn}"


class TeleopNode(Node):
    def __init__(self):
        super().__init__('turtlebot_teleop')
        # publish ra /cmd_vel (không phải ~cmd_vel)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 5)

        self.speed = 0.5
        self.turn = 1.0

        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.target_speed = 0.0
        self.target_turn = 0.0
        self.control_speed = 0.0
        self.control_turn = 0.0

    def loop(self, settings):
        print(msg)
        print(vels(self.speed, self.turn))

        while True:
            key = getKey(settings)

            if key in moveBindings:
                self.x, self.th = moveBindings[key]
                self.count = 0
            elif key in speedBindings:
                self.speed *= speedBindings[key][0]
                self.turn *= speedBindings[key][1]
                self.count = 0

                print(vels(self.speed, self.turn))
                if self.status == 14:
                    print(msg)
                self.status = (self.status + 1) % 15
            elif key == ' ' or key == 'k':
                self.x = 0
                self.th = 0
                self.control_speed = 0.0
                self.control_turn = 0.0
            else:
                self.count += 1
                if self.count > 4:
                    self.x = 0
                    self.th = 0
                if key == '\x03':  # CTRL-C
                    break

            self.target_speed = self.speed * self.x
            self.target_turn = self.turn * self.th

            # ramp speed
            if self.target_speed > self.control_speed:
                self.control_speed = min(self.target_speed, self.control_speed + 0.8)
            elif self.target_speed < self.control_speed:
                self.control_speed = max(self.target_speed, self.control_speed - 0.8)
            else:
                self.control_speed = self.target_speed

            # ramp turn
            if self.target_turn > self.control_turn:
                self.control_turn = min(self.target_turn, self.control_turn + 1.5)
            elif self.target_turn < self.control_turn:
                self.control_turn = max(self.target_turn, self.control_turn - 1.5)
            else:
                self.control_turn = self.target_turn

            twist = Twist()
            twist.linear.x = float(self.control_speed)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(self.control_turn)

            self.pub.publish(twist)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()

    try:
        node.loop(settings)
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        node.pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
