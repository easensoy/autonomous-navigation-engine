#!/usr/bin/env python3
"""Publishes the robot's observed trajectory for RViz2: the simulator's ground-truth poses
(ground_truth/odom), kept every 2 cm of travel, as nav_msgs/Path on ane/observed_trajectory.

It reads ground truth directly, not the navigator's (fault-injected) feedback, so the trajectory also
shows how the robot moved while the navigator had no pose.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class TrajectoryPath(Node):
    def __init__(self):
        super().__init__("trajectory_path")
        self.spacing = float(self.declare_parameter("spacing_m", 0.02).value)
        self.path = Path()
        self.path.header.frame_id = self.declare_parameter("frame", "map").value
        self.publisher = self.create_publisher(Path, "ane/observed_trajectory", 5)
        self.create_subscription(Odometry, "ground_truth/odom", self.on_odom, 50)
        self.create_timer(0.2, self.publish)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            if math.hypot(p.x - last.x, p.y - last.y) < self.spacing:
                return
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.path.header.frame_id
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)

    def publish(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path)


def main():
    rclpy.init()
    node = TrajectoryPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
