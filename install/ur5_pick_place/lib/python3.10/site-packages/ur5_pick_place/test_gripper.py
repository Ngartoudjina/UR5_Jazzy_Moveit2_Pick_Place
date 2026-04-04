#!/usr/bin/env python3

# ==========================================================
# Test simple de la pince Robotiq 85
# Ouvre la pince, attend 2 secondes, puis la ferme
# ==========================================================

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


# Valeurs de la pince
GRIPPER_OPEN = 0.0      # pince ouverte
GRIPPER_CLOSE = 0.72    # pince fermée
GRIPPER_FORCE = 40.0    # force appliquée


class GripperTest(Node):

    def __init__(self):
        super().__init__('gripper_test')

        # Connexion au contrôleur de la pince
        self.client = ActionClient(
            self,
            GripperCommand,
            '/robotiq_85_gripper_controller/gripper_cmd'
        )

        self.get_logger().info('Attente du contrôleur de pince...')
        self.client.wait_for_server()
        self.get_logger().info('Contrôleur trouvé ✅')

    def send_gripper(self, position):
        """
        Envoie une commande à la pince.

        position = 0.0   -> ouverte
        position = 0.72  -> fermée
        """

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_FORCE

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(f'Commande envoyée : {position}')
def main():
    rclpy.init()

    node = GripperTest()

    # Ouvrir la pince
    node.get_logger().info('Ouverture de la pince')
    node.send_gripper(GRIPPER_OPEN)

    time.sleep(2)

    # Fermer la pince
    node.get_logger().info('Fermeture de la pince')
    node.send_gripper(GRIPPER_CLOSE)

    time.sleep(2)

    # Réouvrir la pince
    node.get_logger().info('Réouverture de la pince')
    node.send_gripper(GRIPPER_OPEN)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()