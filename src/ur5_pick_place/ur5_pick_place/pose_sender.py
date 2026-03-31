#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

SUCCESS = 1


class PoseSender(Node):
    def __init__(self):
        super().__init__('pose_sender')

        self.VEL  = 0.20
        self.ACC  = 0.20
        self.POS_TOL = 0.15

        self.PREPICK_Z  = 0.25
        self.GRASP_Z    = 0.10
        self.LIFT_Z     = 0.40
        self.PREPLACE_Z = 0.25
        self.PLACE_Z    = 0.10
        self.RETREAT_Z  = 0.40
        self.PLACE_X    = 0.35
        self.PLACE_Y    = 0.25

        self.PLANNER_ID            = 'RRTConnectkConfigDefault'
        self.ALLOWED_PLANNING_TIME = 20.0
        self.NUM_PLANNING_ATTEMPTS = 20

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for /move_action...')
        self.client.wait_for_server()
        self.get_logger().info('Connected to /move_action ✅')

        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Waiting for /apply_planning_scene...')
        self.scene_client.wait_for_service()
        self.get_logger().info('Connected to /apply_planning_scene ✅')

        self.latest_object_pose = None
        self.create_subscription(PoseStamped, '/object_pose', self.object_pose_cb, 10)

    def object_pose_cb(self, msg):
        self.latest_object_pose = msg

    def _build_goal(self, x, y, z, qx, qy, qz, qw):
        target = PoseStamped()
        target.header.frame_id = 'world'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)
        target.pose.orientation.x = float(qx)
        target.pose.orientation.y = float(qy)
        target.pose.orientation.z = float(qz)
        target.pose.orientation.w = float(qw)

        goal = MoveGroup.Goal()
        req  = goal.request
        req.group_name                      = 'ur5_arm'
        req.num_planning_attempts           = int(self.NUM_PLANNING_ATTEMPTS)
        req.allowed_planning_time           = float(self.ALLOWED_PLANNING_TIME)
        req.max_velocity_scaling_factor     = float(self.VEL)
        req.max_acceleration_scaling_factor = float(self.ACC)
        req.pipeline_id = 'ompl'
        req.planner_id  = self.PLANNER_ID

        pc = PositionConstraint()
        pc.header    = target.header
        pc.link_name = 'wrist_3_link'
        tol_box = SolidPrimitive()
        tol_box.type       = SolidPrimitive.BOX
        tol_box.dimensions = [self.POS_TOL, self.POS_TOL, self.POS_TOL]
        pc.constraint_region.primitives.append(tol_box)
        pc.constraint_region.primitive_poses.append(target.pose)
        pc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        req.goal_constraints = [c]
        return goal

    def _send_goal_and_wait(self, goal):
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return -1
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val

    def go(self, x, y, z, qx, qy, qz, qw, label='') -> bool:
        self.get_logger().info(f'➡️  {label}  x={x:.3f}, y={y:.3f}, z={z:.3f}')
        for attempt in (1, 2, 3):
            goal = self._build_goal(x, y, z, qx, qy, qz, qw)
            if attempt == 2:
                goal.request.allowed_planning_time = 30.0
            if attempt == 3:
                goal.request.allowed_planning_time = 45.0
                goal.request.num_planning_attempts = 30
            code = self._send_goal_and_wait(goal)
            if code == SUCCESS:
                return True
            self.get_logger().info(f'   (retry {attempt}/3, code={code})')
        self.get_logger().warn(f'❌ Failed: {label}')
        return False

    def wait_for_object_pose(self, timeout_sec=10.0):
        start = time.time()
        while rclpy.ok() and self.latest_object_pose is None and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_object_pose is not None

    def get_object_xy(self):
        x, y = 0.35, 0.00
        if self.latest_object_pose is not None:
            x = self.latest_object_pose.pose.position.x
            y = self.latest_object_pose.pose.position.y
        return x, y

    def _apply_scene(self, aco, label):
        req = ApplyPlanningScene.Request()
        req.scene.robot_state.attached_collision_objects.append(aco)
        req.scene.is_diff = True
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        ok   = bool(resp and resp.success)
        self.get_logger().info(f'{label} {"✅" if ok else "❌"}')
        return ok

    def attach_box(self):
        aco = AttachedCollisionObject()
        aco.link_name        = 'wrist_3_link'
        aco.object.id        = 'box'
        aco.object.operation = CollisionObject.ADD
        return self._apply_scene(aco, '📎 Attached box')

    def detach_box(self):
        aco = AttachedCollisionObject()
        aco.link_name        = 'wrist_3_link'
        aco.object.id        = 'box'
        aco.object.operation = CollisionObject.REMOVE
        return self._apply_scene(aco, '🧷 Detached box')

    def open_gripper(self):
        self.get_logger().info('Gripper OPEN 🟢')

    def close_gripper(self):
        self.get_logger().info('Gripper CLOSE 🔴')


def main(args=None):
    rclpy.init(args=args)
    node = PoseSender()
    qx, qy, qz, qw = -0.707, 0.0, 0.0, 0.707

    try:
        ok = node.wait_for_object_pose(timeout_sec=10.0)
        if not ok:
            node.get_logger().info('No /object_pose -> using fallback')

        obj_x, obj_y = node.get_object_xy()

        node.open_gripper()
        if not node.go(0.30, 0.00, 0.50, qx, qy, qz, qw, label='HOME'):
            return
        time.sleep(0.5)

        if not node.go(obj_x, obj_y, node.PREPICK_Z, qx, qy, qz, qw, label='PRE-PICK'):
            return
        time.sleep(0.4)

        if not node.go(obj_x, obj_y, node.GRASP_Z, qx, qy, qz, qw, label='DESCEND'):
            return
        time.sleep(0.4)

        node.close_gripper()
        time.sleep(0.3)
        node.attach_box()
        time.sleep(0.4)

        if not node.go(obj_x, obj_y, node.LIFT_Z, qx, qy, qz, qw, label='LIFT'):
            return
        time.sleep(0.4)

        if not node.go(node.PLACE_X, node.PLACE_Y, node.PREPLACE_Z, qx, qy, qz, qw, label='PRE-PLACE'):
            return
        time.sleep(0.4)

        if not node.go(node.PLACE_X, node.PLACE_Y, node.PLACE_Z, qx, qy, qz, qw, label='PLACE'):
            return
        time.sleep(0.3)

        node.open_gripper()
        time.sleep(0.3)
        node.detach_box()
        time.sleep(0.3)

        node.go(node.PLACE_X, node.PLACE_Y, node.RETREAT_Z, qx, qy, qz, qw, label='RETREAT')
        node.get_logger().info('✅ Pick & Place complete!')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
