#!/usr/bin/env python3
"""
joint_wave_demo.py — Test sinusoidal de tous les joints du UR5
Utilise directement ros2_control via FollowJointTrajectory
sans passer par MoveIt2.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Nom exact du contrôleur ros2_control
ACTION_SERVER = '/ur5_arm_controller/follow_joint_trajectory'

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Position de départ sûre (identique à initial_positions.yaml)
START_POS = [0.5, -1.57, 1.57, -1.57, -1.57, 0.0]


class JointWaveDemo(Node):
    def __init__(self):
        super().__init__('joint_wave_demo')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_SERVER
        )
        self.get_logger().info(f'Attente de {ACTION_SERVER} ...')
        self.client.wait_for_server()
        self.get_logger().info('Contrôleur ros2_control prêt ✅')

    def build_trajectory(self, n_points=200, duration=40.0):
        """
        Génère une trajectoire sinusoïdale autour de la position de départ.
        - n_points waypoints sur `duration` secondes
        - Amplitudes choisies pour rester dans les limites du UR5
        - Premier point = position de départ pour éviter les sauts
        """
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        # Amplitudes de chaque joint (radians)
        # Conservatrices pour rester dans l'espace de travail
        amplitudes = [0.4, 0.3, 0.4, 0.3, 0.3, 0.4]

        # Fréquences relatives
        freqs = [1.0, 0.8, 1.2, 1.5, 2.0, 2.5]

        for i in range(n_points):
            t = i * (2 * math.pi / n_points)

            positions = [
                START_POS[j] + amplitudes[j] * math.sin(t * freqs[j])
                for j in range(6)
            ]

            velocities = [
                amplitudes[j] * freqs[j] * math.cos(t * freqs[j])
                * (2 * math.pi / duration)
                for j in range(6)
            ]

            time_sec = duration * i / n_points
            sec  = int(time_sec)
            nsec = int((time_sec - sec) * 1e9)

            # Forcer t=0 pour le premier point à t=0.01s minimum
            if i == 0:
                sec  = 0
                nsec = 10_000_000  # 10ms

            point = JointTrajectoryPoint()
            point.positions       = positions
            point.velocities      = velocities
            point.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(point)

        # Ajouter un point final identique au premier pour boucler proprement
        first = traj.points[0]
        last  = JointTrajectoryPoint()
        last.positions       = [START_POS[j] + amplitudes[j] * math.sin(0.0)
                                 for j in range(6)]
        last.velocities      = [0.0] * 6
        last.time_from_start = Duration(
            sec=int(duration),
            nanosec=0
        )
        traj.points.append(last)

        return traj

    def run_forever(self):
        cycle = 0
        self.get_logger().info('Démarrage de la démo sinusoïdale — Ctrl+C pour arrêter')

        while rclpy.ok():
            cycle += 1
            self.get_logger().info(f'\n=== Cycle {cycle} — envoi trajectoire ===')

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = self.build_trajectory(n_points=200, duration=40.0)

            future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error('Trajectoire refusée !')
                self.get_logger().error(
                    'Vérifiez que ur5_arm_controller est actif : '
                    'ros2 control list_controllers'
                )
                return

            self.get_logger().info('Trajectoire acceptée ✅ — exécution en cours...')

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            if result:
                code = result.result.error_code
                if code == 0:
                    self.get_logger().info(f'Cycle {cycle} terminé ✅')
                else:
                    self.get_logger().warn(f'Cycle {cycle} terminé avec code={code}')
            else:
                self.get_logger().warn(f'Cycle {cycle} — pas de résultat')


def main():
    rclpy.init()
    node = JointWaveDemo()
    try:
        node.run_forever()
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt par Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
