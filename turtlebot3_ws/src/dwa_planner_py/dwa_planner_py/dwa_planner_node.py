#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class DWAPlanner(Node):

    def __init__(self):
        super().__init__('dwa_planner_node')

        # ================= Robot constraints =================
        self.max_vel = 0.22
        self.max_yaw_rate = 2.84
        self.dt = 0.1
        self.predict_time = 2.0

        # Sampling resolution
        self.v_samples = 9
        self.w_samples = 17

        # ================= Cost weights =================
        self.w_goal = 2.0
        self.w_obstacle = 2.5      # ↑ stronger obstacle influence
        self.w_heading = 0.2


        self.goal_tolerance = 0.15
        self.robot_radius = 0.22

        # ================= State =================
        self.odom = None
        self.scan = None
        self.goal = None
        self.goal_reached = False

        # ================= ROS Interfaces =================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("DWA Planner (Corrected Version) Started")

    # ================= Callbacks =================

    def odom_callback(self, msg):
        self.odom = msg

    def scan_callback(self, msg):
        self.scan = msg

    def goal_callback(self, msg):
        self.goal = msg
        self.goal_reached = False
        self.get_logger().info("New goal received")

    # ================= Helpers =================

    def get_robot_pose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )
        return p.x, p.y, yaw

    def predict_trajectory(self, x, y, yaw, v, w):
        traj = []
        for _ in np.arange(0, self.predict_time, self.dt):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj.append((x, y, yaw))
        return traj

    def get_obstacle_points(self, rx, ry, ryaw):
        points = []
        if self.scan is None:
            return points

        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if self.scan.range_min < r < self.scan.range_max:
                ox = rx + r * math.cos(ryaw + angle)
                oy = ry + r * math.sin(ryaw + angle)
                points.append((ox, oy))
            angle += self.scan.angle_increment
        return points

    def calculate_cost(self, traj, v, obs_points):
        tx, ty, tyaw = traj[-1]
        gx, gy = self.goal.pose.position.x, self.goal.pose.position.y

        # ---------- Goal cost ----------
        goal_cost = math.hypot(gx - tx, gy - ty)

        # ---------- Obstacle cost ----------
        min_dist = float('inf')
        for px, py, _ in traj:
            for ox, oy in obs_points:
                d = math.hypot(px - ox, py - oy)
                if d < self.robot_radius:
                    return 100.0  # collision penalty
                min_dist = min(min_dist, d)

        min_dist = max(min_dist, 0.2)
        obstacle_cost = 1.0 / min_dist

        # ---------- Heading cost ----------
        desired_yaw = math.atan2(gy - ty, gx - tx)
        heading_error = abs(math.atan2(
            math.sin(desired_yaw - tyaw),
            math.cos(desired_yaw - tyaw)
        ))

        # ---------- Stop penalty (prevents freezing) ----------
        stop_cost = 1.5 if v < 0.05 else 0.0

        velocity_reward = (self.max_vel - v)   # penalize slow motion

        return (
            self.w_goal * goal_cost +
            self.w_obstacle * obstacle_cost +
            self.w_heading * heading_error +
            stop_cost +
            0.8 * velocity_reward
        )


    # ================= Visualization =================

    def create_marker(self, traj, mid, is_best=False):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "dwa_traj"
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04 if is_best else 0.01

        m.color.a = 1.0
        m.color.r = 1.0 if is_best else 0.0
        m.color.g = 0.0 if is_best else 0.5
        m.color.b = 0.0 if is_best else 1.0

        for x, y, _ in traj:
            p = Point()
            p.x, p.y, p.z = x, y, 0.02
            m.points.append(p)

        return m

    # ================= Main Control Loop =================

    def control_loop(self):
        if self.odom is None or self.scan is None or self.goal is None:
            return

        rx, ry, ryaw = self.get_robot_pose()

        # ----- Goal reached -----
        if math.hypot(self.goal.pose.position.x - rx,
                      self.goal.pose.position.y - ry) < self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info("Goal reached. Robot stopped.")
                self.goal_reached = True
            self.cmd_pub.publish(Twist())
            return

        obs_points = self.get_obstacle_points(rx, ry, ryaw)

        best_cost = float('inf')
        best_v, best_w = 0.0, 0.0
        best_traj = None

        markers = MarkerArray()
        mid = 0

        # ----- DWA Sampling (ALLOW v = 0) -----
        for v in np.linspace(0.0, self.max_vel, self.v_samples):
            for w in np.linspace(-self.max_yaw_rate, self.max_yaw_rate, self.w_samples):
                traj = self.predict_trajectory(rx, ry, ryaw, v, w)
                cost = self.calculate_cost(traj, v, obs_points)

                if mid % 5 == 0:
                    markers.markers.append(self.create_marker(traj, mid))
                mid += 1

                if cost < best_cost:
                    best_cost = cost
                    best_v, best_w = v, w
                    best_traj = traj

        # ----- Recovery behavior -----
        if best_traj is None or best_cost == float('inf'):
            self.get_logger().warn("No valid trajectory → rotating in place")
            cmd = Twist()
            cmd.angular.z = 0.6
            # Prevent endless rotation
            if abs(best_w) > 0.6 and best_v < 0.03:
                best_v = 0.05
            self.cmd_pub.publish(cmd)
            return

        # ----- Execute best command -----
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

        markers.markers.append(self.create_marker(best_traj, 999, True))
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
