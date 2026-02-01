#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class DWAPlanner(Node):

    def __init__(self):
        super().__init__('dwa_planner_node')

        # Robot limits (TurtleBot3 Burger)
        self.max_vel = 0.22
        self.max_yaw_rate = 2.84

        self.dt = 0.1
        self.predict_time = 2.0

        self.v_samples = 5
        self.w_samples = 11

        self.w_goal = 1.0
        self.w_obstacle = 1.5
        self.goal_tolerance = 0.25  # meters
        self.goal_reached = False


        # State
        self.odom = None
        self.scan = None
        self.goal = None

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("DWA Planner Python Node Started")

    # ---------------- Callbacks ----------------

    def odom_callback(self, msg):
        self.odom = msg

    def scan_callback(self, msg):
        self.scan = msg

    def goal_callback(self, msg):
        self.goal = msg
        self.goal_reached = False
        self.get_logger().info("New goal received")


    # ---------------- Utils ----------------

    def get_robot_pose(self):
        pos = self.odom.pose.pose.position
        ori = self.odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (ori.w * ori.z + ori.x * ori.y),
            1.0 - 2.0 * (ori.y**2 + ori.z**2)
        )
        return pos.x, pos.y, yaw

    def predict_trajectory(self, x, y, yaw, v, w):
        traj = []
        t = 0.0
        while t <= self.predict_time:
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj.append((x, y))
            t += self.dt
        return traj

    def goal_cost(self, traj):
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        lx, ly = traj[-1]
        return math.hypot(gx - lx, gy - ly)

    def obstacle_cost(self, traj):
        if self.scan is None:
            return 0.0
        min_range = min(self.scan.ranges)
        if min_range <= 0.2:
            return float('inf')
        return 1.0 / min_range

    def trajectory_cost(self, traj):
        return (
            self.w_goal * self.goal_cost(traj) +
            self.w_obstacle * self.obstacle_cost(traj)
        )

    def create_marker(self, traj, mid, r, g, b):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "dwa"
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02

        m.color.a = 0.8
        m.color.r = r
        m.color.g = g
        m.color.b = b

        for x, y in traj:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            m.points.append(p)

        return m
    def is_goal_reached(self, x, y):
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        return math.hypot(gx - x, gy - y) < self.goal_tolerance


    # ---------------- DWA Loop ----------------

    def control_loop(self):
        if self.odom is None or self.scan is None or self.goal is None:
            return

        x, y, yaw = self.get_robot_pose()

        if self.is_goal_reached(x, y):
            if not self.goal_reached:
                self.get_logger().info("🎯 Goal reached. Stopping robot.")
                self.goal_reached = True

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        best_cost = float('inf')
        best_v = 0.0
        best_w = 0.0
        best_traj = None

        markers = MarkerArray()
        mid = 0

        for v in np.linspace(0.0, self.max_vel, self.v_samples):
            for w in np.linspace(-self.max_yaw_rate, self.max_yaw_rate, self.w_samples):

                traj = self.predict_trajectory(x, y, yaw, v, w)
                cost = self.trajectory_cost(traj)

                markers.markers.append(self.create_marker(traj, mid, 0.0, 0.0, 1.0))
                mid += 1

                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_w = w
                    best_traj = traj

        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

        if best_traj:
            markers.markers.append(self.create_marker(best_traj, mid, 1.0, 0.0, 0.0))

        self.marker_pub.publish(markers)

        self.get_logger().info(
            f"cmd_vel -> v={best_v:.2f}, w={best_w:.2f}, cost={best_cost:.2f}"
        )


def main():
    rclpy.init()
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
