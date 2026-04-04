#!/usr/bin/env python3
"""
Test infini de la pince Robotiq 85 via MoveIt2.
Ouvre et ferme la pince en boucle jusqu'a Ctrl+C.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

SUCCESS = MoveItErrorCodes.SUCCESS
GRIPPER_OPEN   = 0.0
GRIPPER_CLOSED = 0.72
PAUSE          = 1.5   # secondes entre chaque mouvement


class GripperTest(Node):
    def __init__(self):
        super().__init__('gripper_test')
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Attente de /move_action ...')
        self.client.wait_for_server()
        self.get_logger().info('Connecte ✅')
        self.cycle = 0

    def move_gripper(self, position: float, label: str) -> bool:
        goal = MoveGroup.Goal()
        req  = goal.request
        req.group_name                      = 'hand'
        req.allowed_planning_time           = 10.0
        req.num_planning_attempts           = 10
        req.max_velocity_scaling_factor     = 0.5
        req.max_acceleration_scaling_factor = 0.5
        req.pipeline_id     = 'ompl'
        req.start_state.is_diff = True

        jc = JointConstraint()
        jc.joint_name      = 'robotiq_85_left_knuckle_joint'
        jc.position        = float(position)
        jc.tolerance_above = 0.05
        jc.tolerance_below = 0.05
        jc.weight          = 1.0

        c = Constraints()
        c.joint_constraints.append(jc)
        req.goal_constraints = [c]

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().error('Goal rejete')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val

        ok = (code == SUCCESS)
        icon = '✅' if ok else '⚠️ '
        self.get_logger().info(f'{icon} [{self.cycle}] {label} (code={code})')
        return ok


def main():
    rclpy.init()
    node = GripperTest()

    node.get_logger().info('=== Test Gripper infini — Ctrl+C pour arreter ===')

    try:
        while rclpy.ok():
            node.cycle += 1
            node.get_logger().info(f'\n--- Cycle {node.cycle} ---')

            # Ouverture
            node.get_logger().info('🟢 Ouverture...')
            node.move_gripper(GRIPPER_OPEN, 'OPEN')
            time.sleep(PAUSE)

            # Fermeture
            node.get_logger().info('🔴 Fermeture...')
            node.move_gripper(GRIPPER_CLOSED, 'CLOSE')
            time.sleep(PAUSE)

    except KeyboardInterrupt:
        node.get_logger().info(f'\n=== Arrete apres {node.cycle} cycles ===')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
