#!/usr/bin/env python3
"""Exploration autonome du rover ORIGINES : il fait le tour de l'arène
pendant que slam_toolbox dessine la carte.

On réutilise l'évitement d'obstacle (palier C) : au lieu d'un seul but,
on lui donne une suite de points de passage.
"""
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from origines_control.avoidance_node import AvoidanceNode


class ExploreNode(AvoidanceNode):

    def __init__(self):
        super().__init__('explore_node')
        # Points de passage (x1, y1, x2, y2, ...) dans le repère "odom"
        # (origine = point de départ du rover, à x = -1,5 m dans le monde).
        # Tour de l'arène à 0,8 m des murs.
        self.declare_parameter(
            'waypoints', [0.3, 1.2, 2.7, 1.2, 2.7, -1.2, 0.3, -1.2, 0.3, 0.0])
        self.declare_parameter('wp_tolerance', 0.3)
        self.declare_parameter('arena_cells', 6300)   # arène 4 m x 4 m à 5 cm

        flat = list(self.get_parameter('waypoints').value)
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        self.wp_tol = float(self.get_parameter('wp_tolerance').value)
        self.arena_cells = float(self.get_parameter('arena_cells').value)
        self.index = 0
        self.finished = False
        self.known_cells = 0
        self.set_goal()

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.report_timer = self.report_timer = self.create_timer(1.0, self.report)

    def set_goal(self):
        self.goal_x, self.goal_y = self.waypoints[self.index]
        self.arrived = False
        self.get_logger().info(
            f'Point de passage {self.index + 1}/{len(self.waypoints)} : '
            f'({self.goal_x:.1f}, {self.goal_y:.1f})')

    def on_map(self, msg):
        self.known_cells = sum(1 for v in msg.data if v == 0)

    def report(self):
        pct = min(100.0, 100.0 * self.known_cells / self.arena_cells)
        self.get_logger().info(
            f'CARTE {pct:5.1f} %  ({self.known_cells} cellules)')

    def control_loop(self):
        if self.pose is not None and not self.finished:
            x, y, _ = self.pose
            if math.hypot(self.goal_x - x, self.goal_y - y) < self.wp_tol:
                self.index += 1
                if self.index >= len(self.waypoints):
                    self.finished = True
                    self.arrived = True
                    self.stop()
                    self.report_timer.cancel()
                    self.report()
                    self.report_timer.cancel()
                    self.report()
                    self.get_logger().info('Tour terminé : la carte est prête.')
                    return
                self.set_goal()
        super().control_loop()


def main(args=None):
    rclpy.init(args=args)
    node = ExploreNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
