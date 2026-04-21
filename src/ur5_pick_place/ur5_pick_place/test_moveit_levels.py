#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from moveit_msgs.action import MoveGroup

from rclpy.action import ActionClient


class MoveItLevels(Node):

    def __init__(self):
        super().__init__("moveit_levels")

        # =========================
        # LEVEL 1 : IK SERVICE
        # =========================
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for IK service...")

        # =========================
        # LEVEL 2 : MOVE GROUP ACTION
        # =========================
        self.move_client = ActionClient(self, MoveGroup, "move_action")

        self.get_logger().info("MoveIt Levels Ready")

    # =========================================================
    # 🔹 LEVEL 1 : IK (POSE → JOINTS)
    # =========================================================
    def level_1_ik(self, x, y, z, qx, qy, qz, qw):

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur5_arm"
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state = RobotState()
        request.ik_request.timeout.sec = 2

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error("❌ IK FAILED")
            return None

        joints = dict(zip(
            res.solution.joint_state.name,
            res.solution.joint_state.position
        ))

        self.get_logger().info("===== LEVEL 1 IK =====")
        for k, v in joints.items():
            self.get_logger().info(f"{k}: {v:.3f}")

        return joints

    # =========================================================
    # 🔹 LEVEL 2 : PLANNING (POSE → TRAJECTORY → JOINTS)
    # =========================================================
    def level_2_plan(self, x, y, z, qx, qy, qz, qw):

        self.move_client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request.group_name = "ur5_arm"

        goal.request.goal_constraints.append(
            self.create_pose_constraint(x, y, z, qx, qy, qz, qw)
        )

        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0

        self.get_logger().info("Sending planning request...")

        # 1. envoyer goal
        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return None

        # 2. résultat
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        traj = result.planned_trajectory.joint_trajectory

        if len(traj.points) == 0:
            self.get_logger().error("❌ No trajectory found")
            return None

        last = traj.points[-1]

        joints = dict(zip(
            traj.joint_names,
            last.positions
        ))

        self.get_logger().info("===== LEVEL 2 PLANNING =====")
        for k, v in joints.items():
            self.get_logger().info(f"{k}: {v:.3f}")

        return joints

    # =========================================================
    # Helper : Pose constraint
    # =========================================================
    def create_pose_constraint(self, x, y, z, qx, qy, qz, qw):

        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose

        constraints = Constraints()

        # POSITION
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link"

        sphere = SolidPrimitive()
        sphere.type = sphere.SPHERE
        sphere.dimensions = [0.01]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        pos.constraint_region.primitives.append(sphere)
        pos.constraint_region.primitive_poses.append(pose)
        pos.weight = 1.0

        # ORIENTATION
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"

        ori.orientation.x = qx
        ori.orientation.y = qy
        ori.orientation.z = qz
        ori.orientation.w = qw

        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight = 1.0

        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(ori)

        return constraints


# =========================================================
# MAIN
# =========================================================
def main():
    rclpy.init()
    node = MoveItLevels()

    # =========================
    # LEVEL 1
    # =========================
    node.level_1_ik(
        0.3, 0.0, 0.2,
        0.0, 0.707, 0.0, 0.707
    )

    # =========================
    # LEVEL 2
    # =========================
    node.level_2_plan(
        0.3, 0.0, 0.2,
        0.0, 0.707, 0.0, 0.707
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
