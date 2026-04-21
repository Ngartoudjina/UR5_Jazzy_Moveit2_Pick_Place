#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from moveit_msgs.action import MoveGroup

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class MoveItFullPipeline(Node):

    def __init__(self):
        super().__init__("moveit_full_pipeline")

        # =========================
        # IK SERVICE
        # =========================
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting IK service...")

        # =========================
        # MOVEIT PLANNING
        # =========================
        self.move_client = ActionClient(self, MoveGroup, "move_action")

        # =========================
        # EXECUTION CONTROLLER
        # =========================
        self.exec_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory"
        )

        self.get_logger().info("FULL PIPELINE READY")

    # =========================================================
    # 🔹 1. IK
    # =========================================================
    def compute_ik(self, x, y, z, qx, qy, qz, qw):

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur5_arm"
        req.ik_request.pose_stamped = pose
        req.ik_request.robot_state = RobotState()
        req.ik_request.timeout.sec = 2

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()

        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error("IK FAILED")
            return None

        joints = dict(zip(
            res.solution.joint_state.name,
            res.solution.joint_state.position
        ))

        self.get_logger().info("===== IK DONE =====")
        return joints

    # =========================================================
    # 🔹 2. PLANNING
    # =========================================================
    def plan(self, x, y, z, qx, qy, qz, qw):

        self.move_client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request.group_name = "ur5_arm"

        goal.request.goal_constraints.append(
            self.create_constraint(x, y, z, qx, qy, qz, qw)
        )

        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5

        self.get_logger().info("Planning trajectory...")

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Planning rejected")
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        traj = result.planned_trajectory.joint_trajectory

        if len(traj.points) == 0:
            self.get_logger().error("No trajectory")
            return None

        self.get_logger().info("===== PLANNING DONE =====")

        return traj

    # =========================================================
    # 🔹 3. EXECUTION (SIMULATION)
    # =========================================================
    def execute(self, trajectory):

        self.exec_client.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("Executing trajectory...")

        future = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Execution rejected")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("===== EXECUTION DONE =====")

    # =========================================================
    # helper constraint
    # =========================================================
    def create_constraint(self, x, y, z, qx, qy, qz, qw):

        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose

        c = Constraints()

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

        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"

        ori.orientation = pose.orientation
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight = 1.0

        c.position_constraints.append(pos)
        c.orientation_constraints.append(ori)

        return c
        
    def add_ground(self):

        co = CollisionObject()
        co.id = "ground"
        co.header.frame_id = "world"

        box = SolidPrimitive()
        box.type = box.BOX
        box.dimensions = [2.0, 2.0, 0.01]  # grand sol plat

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.005

        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        self.scene_pub = self.create_publisher(
            CollisionObject,
            "/collision_object",
            10
        )

        self.scene_pub.publish(co)

        self.get_logger().info("Ground added to planning scene")


# =========================================================
# MAIN PIPELINE
# =========================================================
def main():

    rclpy.init()
    node = MoveItFullPipeline()

    x, y, z = 0.3, 0.0, 0.2
    qx, qy, qz, qw = 0.0, 0.707, 0.0, 0.707

    # 1. IK
    node.compute_ik(x, y, z, qx, qy, qz, qw)

    # 2. PLANNING
    traj = node.plan(x, y, z, qx, qy, qz, qw)

    # 3. EXECUTION
    if traj is not None:
        node.execute(traj)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
