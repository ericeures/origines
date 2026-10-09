#!/usr/bin/env python3
"""Évitement d'obstacle réactif du rover ORIGINES (palier C).

Toutes les 0,1 s : on teste des directions autour du rover, on garde celles
où le couloir devant lui est libre d'après le LiDAR, et on prend celle qui
est la plus proche de la direction du but.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def normalise(angle):
    """Ramène un angle dans l'intervalle [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class AvoidanceNode(Node):

    def __init__(self):
        super().__init__('avoidance_node')

        # Réglages (modifiables au lancement avec -p nom:=valeur)
        # Le but est exprimé dans le repère "odom" : l'origine est le point
        # de départ du rover. Départ à x = -1,5 m dans le monde, donc
        # un but à 2,8 m correspond à x = +1,3 m dans l'arène.
        self.declare_parameter('goal_x', 2.8)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('speed', 0.2)           # vitesse d'avance (m/s)
        self.declare_parameter('safe_distance', 0.6)   # distance libre exigée (m)
        self.declare_parameter('half_width', 0.15)     # demi-largeur du couloir (m)
        self.declare_parameter('tolerance', 0.15)      # distance d'arrivée (m)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.speed = float(self.get_parameter('speed').value)
        self.safe = float(self.get_parameter('safe_distance').value)
        self.half = float(self.get_parameter('half_width').value)
        self.tol = float(self.get_parameter('tolerance').value)

        self.pose = None            # (x, y, cap du rover)
        self.points = None          # points du LiDAR, repère du rover
        self.prev_heading = None    # dernier cap choisi (évite d'hésiter)
        self.arrived = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Évitement prêt. But : ({self.goal_x:.1f}, {self.goal_y:.1f}) m')

    # ---------- Réception des capteurs ----------

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def on_scan(self, msg):
        """Convertit chaque mesure (distance, angle) en point (x, y)."""
        pts = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or r < msg.range_min:
                continue
            r = min(r, msg.range_max)   # "rien vu" (inf) devient portée max
            a = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
        self.points = pts

    # ---------- Décision ----------

    def is_free(self, theta):
        """Le couloir dans la direction theta (rad, repère du rover) est-il libre ?"""
        c, s = math.cos(theta), math.sin(theta)
        for x, y in self.points:
            ahead = x * c + y * s          # distance devant, le long de theta
            side = -x * s + y * c          # écart latéral par rapport à theta
            if 0.0 < ahead < self.safe and abs(side) < self.half:
                return False
        return True

    def control_loop(self):
        if self.pose is None or self.points is None:
            return
        if self.arrived:
            self.stop()
            return

        x, y, yaw = self.pose
        dx, dy = self.goal_x - x, self.goal_y - y

        if math.hypot(dx, dy) < self.tol:
            self.arrived = True
            self.stop()
            self.get_logger().info('But atteint, le rover s\'arrête.')
            return

        # Direction du but, dans le repère du rover
        goal_err = normalise(math.atan2(dy, dx) - yaw)

        # On teste les directions de -90° à +90° et on garde la meilleure
        best, best_cost = None, float('inf')
        for k in range(-90, 91, 5):
            theta = math.radians(k)
            if not self.is_free(theta):
                continue
            cost = abs(normalise(theta - goal_err))
            if self.prev_heading is not None:
                cost += 0.3 * abs(normalise(yaw + theta - self.prev_heading))
            if cost < best_cost:
                best, best_cost = theta, cost

        cmd = Twist()
        if best is None:
            # Tout est bloqué devant : on pivote sur place
            cmd.angular.z = 0.8
            self.get_logger().info(
                'Bloqué, je pivote.', throttle_duration_sec=2.0)
        else:
            self.prev_heading = yaw + best
            cmd.angular.z = max(-1.0, min(1.0, 2.0 * best))
            cmd.linear.x = self.speed * max(0.0, math.cos(best))
            self.get_logger().info(
                f'Cap choisi : {math.degrees(best):+.0f}° '
                f'(but à {math.degrees(goal_err):+.0f}°)',
                throttle_duration_sec=1.0)
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
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
