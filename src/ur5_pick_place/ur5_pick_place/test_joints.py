#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

class JointWaveDemo(Node):
    def __init__(self):
        super().__init__('joint_wave_demo')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Attente du contrôleur...')
        self.client.wait_for_server()
        self.get_logger().info('Contrôleur prêt ✅')

    def build_trajectory(self):
        """
        Génère une trajectoire sinusoïdale complète d'un seul coup.
        - 200 points sur 40 secondes → 1 point toutes les 0.2s
        - Le contrôleur interpole entre les points → mouvement parfaitement lisse
        """
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        n_points = 200      # nombre de waypoints
        duration  = 40.0    # durée totale en secondes → très lent

        for i in range(n_points):
            t = i * (2 * math.pi / n_points)   # de 0 à 2π → un cycle complet

            positions = [
                0.5  * math.sin(t),
                -1.0 + 0.2 * math.sin(t * 0.8),
                0.4  * math.sin(t * 1.2),
                -1.2 + 0.3 * math.sin(t * 1.5),
                0.3  * math.sin(t * 2.0),
                0.5  * math.sin(t * 2.5),
            ]

            # Vitesses = dérivée de la sinusoïde → transitions ultra douces
            velocities = [
                0.5  * math.cos(t),
                0.2  * 0.8  * math.cos(t * 0.8),
                0.4  * 1.2  * math.cos(t * 1.2),
                0.3  * 1.5  * math.cos(t * 1.5),
                0.3  * 2.0  * math.cos(t * 2.0),
                0.5  * 2.5  * math.cos(t * 2.5),
            ]

            # Timestamp de ce point dans la trajectoire
            time_sec = duration * i / n_points
            sec  = int(time_sec)
            nsec = int((time_sec - sec) * 1e9)

            point = JointTrajectoryPoint()
            point.positions  = positions
            point.velocities = velocities
            point.time_from_start = Duration(sec=sec, nanosec=nsec)

            traj.points.append(point)

        return traj

    def run_forever(self):
        self.get_logger().info('Envoi de la trajectoire complète...')

        while rclpy.ok():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = self.build_trajectory()

            future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Trajectoire refusée par le contrôleur !')
                return

            self.get_logger().info('Trajectoire acceptée, exécution en cours...')

            # Attendre la fin du cycle complet avant de recommencer
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            self.get_logger().info('Cycle terminé, recommence ✅')


def main():
    rclpy.init()
    node = JointWaveDemo()
    try:
        node.run_forever()
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt demandé par l\'utilisateur')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
