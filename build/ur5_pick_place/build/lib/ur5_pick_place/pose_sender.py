#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint,
                              AttachedCollisionObject, CollisionObject,
                              MoveItErrorCodes)
from moveit_msgs.srv import ApplyPlanningScene

SUCCESS = MoveItErrorCodes.SUCCESS


class PoseSender(Node):
    def __init__(self):
        super().__init__('pose_sender')

        self.VEL     = 0.20
        self.ACC     = 0.20
        self.ALLOWED_PLANNING_TIME = 30.0
        self.NUM_PLANNING_ATTEMPTS = 30

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for /move_action...')
        self.client.wait_for_server()
        self.get_logger().info('Connected to /move_action ✅')

        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Waiting for /apply_planning_scene...')
        self.scene_client.wait_for_service()
        self.get_logger().info('Connected to /apply_planning_scene ✅')

        self.latest_object_pose = None
        self.create_subscription(
            PoseStamped, '/object_pose', self.object_pose_cb, 10)

        # Noms des joints du bras
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        # ============================================================
        # Séquence pick & place en coordonnées articulaires
        # Position initiale : [0, -1.57, 1.57, -1.57, -1.57, 0]
        # shoulder_pan | shoulder_lift | elbow | wrist_1 | wrist_2 | wrist_3
        # ============================================================
        self.HOME      = [ 0.0,  -1.57,  1.57, -1.57, -1.57,  0.0]
        self.PRE_PICK  = [ 0.0,  -1.00,  1.20, -1.80, -1.57,  0.0]
        self.DESCEND   = [ 0.0,  -0.80,  1.50, -2.20, -1.57,  0.0]
        self.LIFT      = [ 0.0,  -1.00,  1.20, -1.80, -1.57,  0.0]
        self.PRE_PLACE = [ 1.57, -1.00,  1.20, -1.80, -1.57,  0.0]
        self.PLACE     = [ 1.57, -0.80,  1.50, -2.20, -1.57,  0.0]
        self.RETREAT   = [ 1.57, -1.00,  1.20, -1.80, -1.57,  0.0]

    def object_pose_cb(self, msg):
        self.latest_object_pose = msg

    def _build_joint_goal(self, positions, tol=0.05):
        goal = MoveGroup.Goal()
        req  = goal.request
        req.group_name                      = 'ur5_arm'
        req.num_planning_attempts           = int(self.NUM_PLANNING_ATTEMPTS)
        req.allowed_planning_time           = float(self.ALLOWED_PLANNING_TIME)
        req.max_velocity_scaling_factor     = float(self.VEL)
        req.max_acceleration_scaling_factor = float(self.ACC)
        req.pipeline_id = 'ompl'
        req.planner_id  = 'RRTConnectkConfigDefault'

        c = Constraints()
        for name, pos in zip(self.joint_names, positions):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(pos)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight          = 1.0
            c.joint_constraints.append(jc)

        req.goal_constraints = [c]
        return goal

    def _send_and_wait(self, goal):
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh or not gh.accepted:
            return -1
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.error_code.val

    def go(self, positions, label='', tol=0.05) -> bool:
        self.get_logger().info(
            f'➡️  {label}  [{", ".join(f"{p:.2f}" for p in positions)}]')
        for attempt in (1, 2, 3):
            goal = self._build_joint_goal(positions, tol)
            if attempt == 2:
                goal.request.allowed_planning_time = 45.0
            if attempt == 3:
                goal.request.allowed_planning_time = 60.0
                goal.request.num_planning_attempts = 50
            code = self._send_and_wait(goal)
            if code == SUCCESS:
                self.get_logger().info(f'   ✅ {label}')
                return True
            self.get_logger().info(f'   retry {attempt}/3  code={code}')
        self.get_logger().warn(f'❌ Failed: {label}')
        return False

    def wait_for_object_pose(self, timeout=10.0):
        t0 = time.time()
        while rclpy.ok() and not self.latest_object_pose \
                and (time.time()-t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_object_pose is not None

    def _apply_scene(self, aco, label):
        req = ApplyPlanningScene.Request()
        req.scene.robot_state.attached_collision_objects.append(aco)
        req.scene.is_diff = True
        fut = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        ok = bool(fut.result() and fut.result().success)
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


def main(args=None):
    rclpy.init(args=args)
    node = PoseSender()

    try:
        node.wait_for_object_pose(10.0)

        # HOME — identique à la position initiale (pas de mouvement parasite)
        node.get_logger().info('Gripper OPEN 🟢')
        if not node.go(node.HOME, label='HOME'):
            return
        time.sleep(0.5)

        if not node.go(node.PRE_PICK, label='PRE-PICK'):
            return
        time.sleep(0.4)

        if not node.go(node.DESCEND, label='DESCEND'):
            return
        time.sleep(0.3)

        node.get_logger().info('Gripper CLOSE 🔴')
        node.attach_box()
        time.sleep(0.4)

        if not node.go(node.LIFT, label='LIFT'):
            return
        time.sleep(0.4)

        if not node.go(node.PRE_PLACE, label='PRE-PLACE'):
            return
        time.sleep(0.4)

        if not node.go(node.PLACE, label='PLACE'):
            return
        time.sleep(0.3)

        node.get_logger().info('Gripper OPEN 🟢')
        node.detach_box()
        time.sleep(0.3)

        node.go(node.RETREAT, label='RETREAT')
        node.get_logger().info('✅ Pick & Place complete!')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
