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
from ament_index_python.packages import get_package_share_directory


class State_Manager(Node):
    def __init__(self, k1, k2, m):
        # Creates a node with name 'move2goal'
        super().__init__("move2goal")

        self.current_pose = None
        self.enemy_pose = None

        self.k1 = k1
        self.k2 = k2
        self.m = m

        base_path = os.path.dirname(os.path.abspath(__file__))

        circle_fn = "waypoints_c.csv"
        s_fn = "waypoints_s.csv"

        self.circle_pattern = self.read_csv(circle_fn)
        self.s_pattern = self.read_csv(s_fn)

        self.current_pattern = "Circle"
        self.usi_pattern = self.circle_pattern
        self.current_pattern_index = 0
        self.last_checkpoint = 0
        self.current_goal = self.usi_pattern[0]
        self.state = "RETURNING"
        self.reached = False
        
        # Publishers
        self.goal_pose_publisher = self.create_publisher(Pose, "/turtle1/goal_pose", 10)

        # Subscribers
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, "/enemy/pose", self.pose_callback_2, 10
        )
        self.goal_reached_subscriber = self.create_subscription(
            String, "/turtle1/goal_reached", self.reached_callback, 10
        )

        # Services
        self.client = self.create_client(SetPen, "/turtle1/set_pen")
        self.clear_client = self.create_client(Empty, "/clear")
        self.kill_client = self.create_client(Kill, "/kill")
        self.spawn_client = self.create_client(Spawn, "/spawn")

    def read_csv(self, filename):
        """Read a CSV file and return the data as a list of lists."""
        package_path = get_package_share_directory("usi_angry_turtle")
        file_path = os.path.join(package_path, "data", filename)

        data = []
        with open(file_path, "r") as file:
            reader = csv.reader(file)
            for row in reader:
                data.append([float(val) for val in row])

        return data

    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)

    def pose_callback_2(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.enemy_pose = msg
        self.enemy_pose.x = round(self.enemy_pose.x, 4)
        self.enemy_pose.y = round(self.enemy_pose.y, 4)

        self.aggresive_state_handler()

    def set_pen(self, pen_off):
        request = SetPen.Request()
        request.off = int(pen_off)
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 8
        future = self.client.call_async(request)

    def reached_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        print(f"Current state: {self.state} and goal: {self.current_goal}")

        array_len = len(self.usi_pattern)
        if self.state == "WRITING":
            if self.current_pattern_index == array_len - 1:
                if self.current_pattern == "Circle":
                    self.usi_pattern = self.s_pattern
                    self.current_pattern = "S"
                else:
                    self.usi_pattern = self.circle_pattern
                    self.current_pattern = "Circle"
                self.current_pattern_index = 0
                self.state = "RETURNING"

            self.last_checkpoint = self.current_pattern_index
            self.current_pattern_index = (self.current_pattern_index + 1) % array_len
            self.current_goal = self.usi_pattern[self.current_pattern_index]

        elif self.state == "RETURNING":
            self.state = "WRITING"
            self.current_pattern_index = self.last_checkpoint
            self.current_goal = self.usi_pattern[self.current_pattern_index]
        self.reached = True

    def start_moving(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(0.1, self.move_callback)
        self.set_pen(1)
        self.clear_client.call_async(Empty.Request())
        self.spawn_enemy()
        self.kill_enemy()
        self.spawn_enemy()
        return

    def spawn_enemy(self):
        request = Spawn.Request()
        request.x = random.uniform(1.0, 5.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = 0.0
        request.name = "enemy"
        future = self.spawn_client.call_async(request)

    def kill_enemy(self):
        request = Kill.Request()
        request.name = "enemy"

        # Send the kill request asynchronously and handle the result in a callback
        future = self.kill_client.call_async(request)
        future.add_done_callback(self.kill_enemy_callback)  # Add callback

    def kill_enemy_callback(self, future):
        try:
            self.spawn_enemy()
        except Exception as e:
            print(f"Error killing enemy: {e}")

    def move_callback(self):
        """Callback called every time a new Pose message is received by the subscriber."""
        if self.state != "WRITING":
            self.set_pen(1)
        else:
            self.set_pen(0)

        if self.state == "RETURNING":
            tgt_pose = Pose()
            tgt_pose.x = self.usi_pattern[self.last_checkpoint][0]
            tgt_pose.y = self.usi_pattern[self.last_checkpoint][1]
            self.goal_pose_publisher.publish(tgt_pose)
        else:
            tgt_pose = Pose()
            tgt_pose.x = self.current_goal[0]
            tgt_pose.y = self.current_goal[1]
            self.goal_pose_publisher.publish(tgt_pose)

    def aggresive_state_handler(self):

        if self.enemy_pose is None:
            return
        if self.current_pose is None:
            return

        # Calculate the distance between the current pose and the enemy pose
        distance = sqrt(
            pow(self.enemy_pose.x - self.current_pose.x, 2)
            + pow(self.enemy_pose.y - self.current_pose.y, 2)
        )

        if self.state == "WRITING":
            if distance < self.k2:
                self.reached = True
                self.state = "ATTACKING"

        elif self.state == "ATTACKING":
            if distance < self.k1:
                self.state = "RETURNING"
                self.kill_enemy()
                self.enemy_pose = None
                return

            if self.reached:
                x_pred = self.enemy_pose.x + cos(self.enemy_pose.theta) * self.m
                y_pred = self.enemy_pose.y + sin(self.enemy_pose.theta) * self.m

                if x_pred < 1:
                    x_pred = 1.0
                elif x_pred > 11:
                    x_pred = 11.0
                if y_pred < 1:
                    y_pred = 1.0
                elif y_pred > 11:
                    y_pred = 11.0
                self.current_goal = [float(x_pred), float(y_pred)]

                self.reached = False

        # TODO distance based on exercise


def main():
    k1 = 1
    k2 = 3
    m = 1

    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = State_Manager(k1, k2, m)
    done = node.start_moving()

    # Keep processings events until the turtle has reached the goal
    # rclpy.spin_until_future_complete(node, done)

    # Alternatively, if you don't want to exit unless someone manually shuts down the node
    rclpy.spin(node)


if __name__ == "__main__":
    main()
