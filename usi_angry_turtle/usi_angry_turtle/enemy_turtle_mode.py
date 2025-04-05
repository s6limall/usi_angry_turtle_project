#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future

import sys
import csv
import os
import random
from math import pow, sin, cos, atan2, sqrt

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn, Kill
from std_srvs.srv import Empty


class Enemy_Turtle_Node(Node):
    def __init__(self):
        # Creates a node with name 'move2goal'
        super().__init__("enemy_turtle")

        self.current_pose = None

        self.usi_pattern = [
            [1.0, 1.0],
            [5.0, 2.0],
            [3.0, 10.0],
            [10.0, 10.0],
            [5.0, 5.0],
        ]
        self.current_pattern_index = 0

        self.current_goal = self.usi_pattern[0]

        self.vel_publisher = self.create_publisher(Twist, "/enemy/cmd_vel", 10)

    def start_moving(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(0.5, self.move_callback)
        return

    def move_callback(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = random.uniform(0.5, 1.0)
        cmd_vel.angular.z = random.uniform(-2.0, 2.0)
        self.vel_publisher.publish(cmd_vel)


def main():

    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    node = Enemy_Turtle_Node()
    done = node.start_moving()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
