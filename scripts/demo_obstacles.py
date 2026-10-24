#!/usr/bin/env python3
"""ORIGINES : le rover va de A a B, des obstacles surprise apparaissent devant lui."""
import argparse
import math
import random
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

WORLD = 'arena'
MAP_TO_GZ = (-1.5, 0.0)   # carte -> Gazebo : le rover part de x = -1,5 m dans Gazebo
CRATE = (0.5, 0.0)        # la caisse fixe (repere Gazebo)
LIMIT = 1.5               # on reste loin des murs (murs a +-2,0 m)
SIZE = 0.25               # cote des obstacles surprise (m)


def gz_call(service, reqtype, req):
    r = subprocess.run(
        ['gz', 'service', '-s', service, '--reqtype', reqtype,
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
        capture_output=True, text=True)
    return 'true' in r.stdout


def remove_obstacle(name):
    gz_call(f'/world/{WORLD}/remove', 'gz.msgs.Entity', f'name: "{name}" type: 2')


def spawn_obstacle(name, gx, gy):
    box = f'<box><size>{SIZE} {SIZE} {SIZE}</size></box>'
    sdf = (f"<sdf version='1.9'><model name='{name}'><static>true</static>"
           f"<pose>{gx:.3f} {gy:.3f} {SIZE / 2:.3f} 0 0 0</pose><link name='l'>"
           f"<collision name='c'><geometry>{box}</geometry></collision>"
           f"<visual name='v'><geometry>{box}</geometry><material>"
           f"<ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse>"
           f"</material></visual></link></model></sdf>")
    return gz_call(f'/world/{WORLD}/create', 'gz.msgs.EntityFactory', f'sdf: "{sdf}"')


def pick_spot(rng, x, y, yaw, goal):
    """Point aleatoire devant le rover (coordonnees carte). Renvoie None si rien de libre."""
    for _ in range(40):
        ahead = rng.uniform(0.9, 1.3)
        side = rng.uniform(-0.15, 0.15)
        mx = x + ahead * math.cos(yaw) - side * math.sin(yaw)
        my = y + ahead * math.sin(yaw) + side * math.cos(yaw)
        gx, gy = mx + MAP_TO_GZ[0], my + MAP_TO_GZ[1]
        if abs(gx) > LIMIT or abs(gy) > LIMIT:
            continue
        if math.hypot(gx - CRATE[0], gy - CRATE[1]) < 0.55:
            continue
        if math.hypot(mx - goal[0], my - goal[1]) < 0.7:   # ne pas boucher le but
            continue
        return mx, my, gx, gy
    return None


class Demo(Node):
    def __init__(self):
        super().__init__('demo_obstacles')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pose = None
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, qos)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_pose(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.pose = (p.position.x, p.position.y, yaw)

    def wait_for(self, cond, timeout, what):
        t0 = time.time()
        while not cond():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                raise SystemExit(f'Echec : {what} introuvable apres {timeout:.0f} s.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', nargs=2, type=float, default=[3.0, 0.0],
                    metavar=('X', 'Y'), help='but dans la carte (defaut : 3.0 0.0)')
    ap.add_argument('--n', type=int, default=2, help="nombre d'obstacles surprise")
    ap.add_argument('--seed', type=int, default=None)
    args = ap.parse_args()
    seed = args.seed if args.seed is not None else random.randrange(10000)
    rng = random.Random(seed)
    print(f'Graine aleatoire : {seed}  (relancer avec --seed {seed} pour rejouer)')

    rclpy.init()
    node = Demo()
    node.wait_for(lambda: node.pose is not None, 20, 'la pose AMCL')
    node.wait_for(lambda: node.client.wait_for_server(timeout_sec=0.1), 20, 'Nav2')
    for i in range(1, 6):
        remove_obstacle(f'obstacle_{i}')

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = args.goal
    goal.pose.pose.orientation.w = 1.0
    print(f'But envoye : ({args.goal[0]:.1f} ; {args.goal[1]:.1f})')
    send = node.client.send_goal_async(goal, feedback_callback=lambda f: setattr(
        node, 'remaining', f.feedback.distance_remaining))
    node.wait_for(send.done, 10, 'la reponse de Nav2')
    handle = send.result()
    if not handle.accepted:
        raise SystemExit('But refuse par Nav2.')
    result = handle.get_result_async()

    t0 = time.time()
    next_at = t0 + rng.uniform(5, 8)
    placed, last_print = 0, 0.0
    node.remaining = float('nan')
    try:
        while not result.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_print > 2:
                print(f'  distance restante : {node.remaining:.2f} m')
                last_print = now
            if placed < args.n and now >= next_at and node.pose:
                spot = pick_spot(rng, *node.pose, args.goal)
                placed += 1
                next_at = now + rng.uniform(4, 6)
                if spot is None:
                    print(f'(obstacle {placed} : pas de place libre, ignore)')
                elif spawn_obstacle(f'obstacle_{placed}', spot[2], spot[3]):
                    print(f'>>> OBSTACLE SURPRISE {placed} pose en ({spot[0]:.2f} ; {spot[1]:.2f})')
                else:
                    print(f'(obstacle {placed} : Gazebo a refuse la pose, ignore)')
    except KeyboardInterrupt:
        handle.cancel_goal_async()
        print('Interrompu.')
    status = result.result().status if result.done() else None
    nom = {GoalStatus.STATUS_SUCCEEDED: 'ARRIVE',
           GoalStatus.STATUS_ABORTED: 'ECHEC',
           GoalStatus.STATUS_CANCELED: 'ANNULE'}.get(status, str(status))
    print(f'=== {nom} en {time.time() - t0:.0f} s ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
