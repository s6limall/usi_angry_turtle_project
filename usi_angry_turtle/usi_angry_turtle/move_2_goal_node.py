#!/usr/bin/env python3
#  move2goal_node.py
#  Velocity controller that moves a Turtlesim turtle toward a user-specified goal position.
#
#  Elia Cereda <elia.cereda@idsia.ch>
#  Simone Arreghini <simone.arreghini@idsia.ch>
#  Dario Mantegazza <dario.mantegazza@idsia.ch>
#  Mirko Nava <mirko.nava@idsia.ch>
#
#  Copyright (C) 2019-2025 IDSIA, USI-SUPSI
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import rclpy
from rclpy.node import Node
from rclpy.task import Future

import sys
from math import pow, sin, cos, atan2, sqrt

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose


class Move2GoalNode(Node):
    def __init__(self, goal_pose, tolerance):
        # Creates a node with name 'move2goal'
        super().__init__("move2goal")

        # Create attributes to store the goal and current poses and tolerance
        self.goal_pose = None
        self.tolerance = tolerance
        self.current_pose = None
        self.goal_reached = False

        # Create a publisher for the topic '/turtle1/cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.goal_reached_publisher = self.create_publisher(
            String, "/turtle1/goal_reached", 10
        )

        # Create a subscriber to the topic '/turtle1/pose', which will call self.pose_callback every
        # time a message of type Pose is received
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.goal_pose_subscriber = self.create_subscription(
            Pose, "/turtle1/goal_pose", self.update_goal, 10
        )

    def update_goal(self, pose):
        # Update the goal pose with a new one
        self.goal_pose = pose
        # self.get_logger().info(f"New goal pose set: {self.goal_pose.x}, {self.goal_pose.y}")
        self.goal_reached = False

    def start_moving(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(0.1, self.move_callback)

        return

    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)

    def move_callback(self):
        """Callback called periodically by the timer to publish a new command."""

        if self.current_pose is None:
            # Wait until we receive the current pose of the turtle for the first time
            return

        if self.goal_pose is None:
            # Wait until we receive the goal pose of the turtle for the first time
            return

        goal_theta = self.steering_angle(self.goal_pose, self.current_pose)

        if abs(self.angular_difference(goal_theta, self.current_pose.theta)) >= 0.01:
            print(f"Adjusting Angle")
            # If the turtle is not facing the goal, rotate it
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.angular_vel(self.goal_pose, self.current_pose)

            # Publish the command
            self.vel_publisher.publish(cmd_vel)

        elif (
            self.euclidean_distance(self.goal_pose, self.current_pose) >= self.tolerance
        ):
            if self.current_pose.angular_velocity > 0.1:
                return
            print(
                f"Adjusting Distance {self.angular_difference(goal_theta, self.current_pose.theta)}"
            )
            # We still haven't reached the goal pose. Use a proportional controller to compute velocities
            # that will move the turtle towards the goal (https://en.wikipedia.org/wiki/Proportional_control)

            # Twist represents 3D linear and angular velocities, in turtlesim we only care about 2 dimensions:
            # linear velocity along the x-axis (forward) and angular velocity along the z-axis (yaw angle)
            cmd_vel = Twist()
            cmd_vel.linear.x = self.linear_vel(self.goal_pose, self.current_pose)
            cmd_vel.angular.z = self.angular_vel(self.goal_pose, self.current_pose)

            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            # self.get_logger().info("Goal reached, shutting down...")

            # Stop the turtle
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.vel_publisher.publish(cmd_vel)

            self.goal_reached = True
            self.goal_reached_publisher.publish(String(data="Goeal reached!"))

    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(
            pow((goal_pose.x - current_pose.x), 2)
            + pow((goal_pose.y - current_pose.y), 2)
        )

    def angular_difference(self, goal_theta, current_theta):
        """Compute shortest rotation from orientation current_theta to orientation goal_theta"""
        output_theta = atan2(
            sin(goal_theta - current_theta), cos(goal_theta - current_theta)
        )
        return output_theta

    def linear_vel(self, goal_pose, current_pose, constant=4):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

    def angular_vel(self, goal_pose, current_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        goal_theta = self.steering_angle(goal_pose, current_pose)
        return constant * self.angular_difference(goal_theta, current_pose.theta)


def main():
    # Get the input from the user.
    goal_pose = Pose()
    goal_pose.x = 5.0
    goal_pose.y = 5.0

    tolerance = 0.01

    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = Move2GoalNode(goal_pose, tolerance)
    done = node.start_moving()

    # Keep processings events until the turtle has reached the goal
    # rclpy.spin_until_future_complete(node, done)

    # Alternatively, if you don't want to exit unless someone manually shuts down the node
    rclpy.spin(node)


if __name__ == "__main__":
    main()
