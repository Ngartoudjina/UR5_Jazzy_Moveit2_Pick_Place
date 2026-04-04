#!/usr/bin/env python3

# ==========================================================
# Test simple des articulations du UR5
# Chaque articulation bouge séparément afin de comprendre
# quel joint correspond à quel mouvement.
# ==========================================================

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


# Liste des articulations du UR5
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
class JointTest(Node):

    def __init__(self):
        super().__init__('joint_test')

        # Connexion à MoveIt
        self.client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info('Attente de MoveIt...')
        self.client.wait_for_server()
        self.get_logger().info('MoveIt prêt ✅')

    def move_joints(self, values):
        """
        values = liste de 6 angles en radians
        """

        goal = MoveGroup.Goal()

        goal.request.group_name = 'ur5_arm'
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10

        constraints = Constraints()

        # Associer chaque angle à chaque articulation
        for name, value in zip(JOINT_NAMES, values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Mouvement refusé')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('Mouvement terminé ✅')



def main():
    rclpy.init()

    node = JointTest()

    # Position de départ : tous les joints à 0
    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    node.get_logger().info('Retour à HOME')
    node.move_joints(home)
    time.sleep(2)
    # --------------------------------------------------
    # 1) shoulder_pan_joint
    # Tourne toute la base gauche/droite
    # --------------------------------------------------
    node.get_logger().info('Test shoulder_pan_joint')
    node.move_joints([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(2)

    node.move_joints(home)
    time.sleep(2)

    # --------------------------------------------------
    # 2) shoulder_lift_joint
    # Monte ou descend le bras principal
    # --------------------------------------------------
    node.get_logger().info('Test shoulder_lift_joint')
    node.move_joints([0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(2)

    node.move_joints(home)
    time.sleep(2)

    # --------------------------------------------------
    # 3) elbow_joint
    # Plie ou déplie le coude
    # --------------------------------------------------
    node.get_logger().info('Test elbow_joint')
    node.move_joints([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    time.sleep(2)

    node.move_joints(home)
    time.sleep(2)
    # --------------------------------------------------
    # 4) wrist_1_joint
    # Oriente le poignet vers le haut/bas
    # --------------------------------------------------
    node.get_logger().info('Test wrist_1_joint')
    node.move_joints([0.0, 0.0, 0.0, -1.0, 0.0, 0.0])
    time.sleep(2)

    node.move_joints(home)
    time.sleep(2)

    # --------------------------------------------------
    # 5) wrist_2_joint
    # Rotation latérale du poignet
    # --------------------------------------------------
    node.get_logger().info('Test wrist_2_joint')
    node.move_joints([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    time.sleep(2)

    node.move_joints(home)
    time.sleep(2)

    # --------------------------------------------------
    # 6) wrist_3_joint
    # Fait tourner la pince sur elle-même
    # --------------------------------------------------
    node.get_logger().info('Test wrist_3_joint')
    node.move_joints([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    time.sleep(2)

    node.move_joints(home)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()