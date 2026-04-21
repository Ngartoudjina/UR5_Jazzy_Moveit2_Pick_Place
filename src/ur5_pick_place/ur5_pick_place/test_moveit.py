#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class MoveItIKClient(Node):

    def __init__(self):
        super().__init__("moveit_ik_client")

        # Service IK MoveIt
        self.cli = self.create_client(GetPositionIK, "/compute_ik")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /compute_ik service...")

        self.get_logger().info("MoveIt IK Client ready")

    def compute_ik(self, x, y, z, qx, qy, qz, qw):

        # 1. Pose cible
        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # 2. Request IK
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur5_arm"
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout.sec = 2

        # IMPORTANT : robot state vide (MoveIt le remplit)
        request.ik_request.robot_state = RobotState()

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error("❌ IK failed")
            return None

        # 3. Extraction joints
        joint_state = response.solution.joint_state

        result = dict(zip(joint_state.name, joint_state.position))

        self.get_logger().info("✅ IK SUCCESS")
        for k, v in result.items():
            self.get_logger().info(f"{k}: {v:.3f}")

        return result


def main():
    rclpy.init()

    node = MoveItIKClient()

    node.compute_ik(
        0.3, 0.0, 0.2,
        0.0, 0.707, 0.0, 0.707
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
