#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene

from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient


class MoveItScene(Node):

    def __init__(self):
        super().__init__("moveit_scene_node")

        # MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")

        # Planning scene service
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene"
        )

        self.get_logger().info("MoveIt Scene Node Ready")

    # -------------------------
    # ADD OBJECT TO SCENE
    # -------------------------
    def add_collision_object(self):

        self.get_logger().info("Adding table + object...")

        scene = PlanningScene()
        scene.is_diff = True

        # ===== TABLE =====
        table = CollisionObject()
        table.id = "table"
        table.header.frame_id = "world"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.2, 0.8, 0.05]

        table_pose = Pose()
        table_pose.position.x = 0.1
        table_pose.position.y = 0.0
        table_pose.position.z = -0.025

        table.primitives.append(box)
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD

        # ===== OBJECT =====
        obj = CollisionObject()
        obj.id = "box_object"
        obj.header.frame_id = "world"

        box2 = SolidPrimitive()
        box2.type = SolidPrimitive.BOX
        box2.dimensions = [0.05, 0.05, 0.10]

        obj_pose = Pose()
        obj_pose.position.x = 0.5
        obj_pose.position.y = 0.1
        obj_pose.position.z = 0.10

        obj.primitives.append(box2)
        obj.primitive_poses.append(obj_pose)
        obj.operation = CollisionObject.ADD

        scene.world.collision_objects.append(table)
        scene.world.collision_objects.append(obj)

        # send to MoveIt
        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self.scene_client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("Scene updated ✅")

    # -------------------------
    # MOVE ARM
    # -------------------------
    def move_to_pose(self):

        self.get_logger().info("Sending goal...")

        goal = MoveGroup.Goal()

        goal.request.group_name = "ur5_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0

        # target pose
        from moveit_msgs.msg import Constraints, PositionConstraint
        from shape_msgs.msg import SolidPrimitive

        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = "wrist_3_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]

        pc.constraint_region.primitives.append(primitive)

        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.2
        pose.position.z = 0.3

        pc.constraint_region.primitive_poses.append(pose)

        constraints.position_constraints.append(pc)
        goal.request.goal_constraints.append(constraints)

        self.move_group_client.wait_for_server()

        future = self.move_group_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Motion done ✅")


def main():

    rclpy.init()
    node = MoveItScene()

    # 1. Add objects
    node.add_collision_object()

    # 2. Move robot
    node.move_to_pose()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
