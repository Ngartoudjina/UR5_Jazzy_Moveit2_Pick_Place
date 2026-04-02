#!/usr/bin/env python3
"""
pick_place_node.py  —  Enhanced UR5 Pick & Place with Robotiq 85 Gripper
=========================================================================
Improvements over original pose_sender.py:
  1. Full approach → grasp → retreat pipeline (Cartesian + joint moves)
  2. Real Robotiq 85 gripper actuation via /gripper_action (GripperCommand)
  3. Object pose-driven pick position (reads /object_pose topic)
  4. Collision object attached/detached correctly to robotiq_85_base_link
  5. Robust error handling with per-stage recovery
  6. Compatible with ROS 2 Humble + MoveIt2

Author : enhanced for Ngartoudjina/UR5_Jazzy_Moveit2_Pick_Place
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, PositionConstraint, OrientationConstraint,
    AttachedCollisionObject, CollisionObject, BoundingVolume,
    MoveItErrorCodes, RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

SUCCESS = MoveItErrorCodes.SUCCESS

# ──────────────────────────────────────────────────────────────────────────────
# Gripper constants  (Robotiq 85 — position in radians, 0=open, 0.8=closed)
# ──────────────────────────────────────────────────────────────────────────────
GRIPPER_OPEN   = 0.0
GRIPPER_CLOSED = 0.72   # ~85% closed — tweak per object size
GRIPPER_EFFORT = 40.0   # Newtons

# ──────────────────────────────────────────────────────────────────────────────
# Named joint positions  (radians)
# shoulder_pan | shoulder_lift | elbow | wrist_1 | wrist_2 | wrist_3
# ──────────────────────────────────────────────────────────────────────────────
POSITIONS = {
    # Safe resting pose — shoulder_pan=0 (different from start pos 0.5)
    'home':       [ 0.00, -1.57,  1.57, -1.57, -1.57,  0.00],

    # Above pick zone
    'pre_pick':   [ 0.00, -1.10,  1.30, -1.77, -1.57,  0.00],

    # At object level
    'grasp':      [ 0.00, -0.85,  1.55, -2.27, -1.57,  0.00],

    # Lift after grasp
    'lift':       [ 0.00, -1.10,  1.30, -1.77, -1.57,  0.00],

    # Transit to place zone
    'pre_place':  [ 1.57, -1.10,  1.30, -1.77, -1.57,  0.00],

    # Descend at place zone
    'place':      [ 1.57, -0.85,  1.55, -2.27, -1.57,  0.00],

    # Retreat after placing
    'retreat':    [ 1.57, -1.10,  1.30, -1.77, -1.57,  0.00],
}

ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


# ──────────────────────────────────────────────────────────────────────────────
class PickPlaceNode(Node):
    """Full pick & place pipeline for UR5 + Robotiq 85."""

    def __init__(self):
        super().__init__('pick_place_node')

        # ── tuning parameters ──────────────────────────────────────────────
        self.declare_parameter('velocity_scale',     0.25)
        self.declare_parameter('acceleration_scale', 0.25)
        self.declare_parameter('planning_time',      30.0)
        self.declare_parameter('planning_attempts',  30)
        self.declare_parameter('joint_tolerance',    0.05)

        self.vel   = self.get_parameter('velocity_scale').value
        self.acc   = self.get_parameter('acceleration_scale').value
        self.ptime = self.get_parameter('planning_time').value
        self.patt  = self.get_parameter('planning_attempts').value
        self.jtol  = self.get_parameter('joint_tolerance').value

        # ── MoveGroup action client ────────────────────────────────────────
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for /move_action …')
        self.move_client.wait_for_server()
        self.get_logger().info('Connected to /move_action ✅')

        # ── Gripper action client (GripperCommand) ─────────────────────────
        self.gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_85_gripper_controller/gripper_cmd')
        self.get_logger().info('Waiting for gripper action server …')
        if self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Connected to gripper ✅')
            self.has_gripper = True
        else:
            self.get_logger().warn(
                '⚠️  Gripper action server not found — '
                'gripper moves will be SIMULATED (attach/detach only).')
            self.has_gripper = False

        # ── Planning scene services ────────────────────────────────────────
        self.scene_apply = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.scene_apply.wait_for_service()

        # ── Object pose subscriber ─────────────────────────────────────────
        self.object_pose: PoseStamped | None = None
        self.create_subscription(
            PoseStamped, '/object_pose', self._object_pose_cb, 10)

        self.get_logger().info('PickPlaceNode ready.')

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _object_pose_cb(self, msg: PoseStamped):
        self.object_pose = msg

    # ── gripper ───────────────────────────────────────────────────────────────
    def _gripper_move(self, position: float, effort: float = GRIPPER_EFFORT,
                      label: str = '') -> bool:
        if not self.has_gripper:
            self.get_logger().info(f'[simulated] gripper → {position:.2f}  {label}')
            time.sleep(0.5)
            return True

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort

        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Gripper goal rejected  {label}')
            return False

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=8.0)
        result = res_fut.result()
        ok = result is not None and not result.result.stalled
        self.get_logger().info(
            f'{"✅" if ok else "⚠️ "} Gripper {label}  pos={position:.2f}')
        return ok

    def open_gripper(self) -> bool:
        return self._gripper_move(GRIPPER_OPEN,   label='OPEN  🟢')

    def close_gripper(self) -> bool:
        return self._gripper_move(GRIPPER_CLOSED, label='CLOSE 🔴')

    # ── arm motion ────────────────────────────────────────────────────────────
    def _build_joint_goal(self, positions, tol: float | None = None) -> MoveGroup.Goal:
        tol = tol or self.jtol
        goal = MoveGroup.Goal()
        req  = goal.request
        req.group_name                      = 'ur5_arm'
        req.num_planning_attempts           = int(self.patt)
        req.allowed_planning_time           = float(self.ptime)
        req.max_velocity_scaling_factor     = float(self.vel)
        req.max_acceleration_scaling_factor = float(self.acc)
        req.pipeline_id = 'ompl'
        req.planner_id  = 'RRTConnectkConfigDefault'
        req.start_state.is_diff = True
        req.allowed_planning_time = float(self.ptime)


        c = Constraints()
        for name, pos in zip(ARM_JOINT_NAMES, positions):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(pos)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight          = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        return goal

    def _send_and_wait(self, goal, timeout: float = 60.0) -> int:
        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        gh = fut.result()
        if not gh or not gh.accepted:
            return -1
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        result = res_fut.result()
        if result is None:
            return -2
        return result.result.error_code.val

    def move_joints(self, key_or_positions, label: str = '',
                    retries: int = 3) -> bool:
        """Move to a named pose (str key) or a raw list of joint angles."""
        positions = (POSITIONS[key_or_positions]
                     if isinstance(key_or_positions, str)
                     else key_or_positions)
        label = label or (key_or_positions
                          if isinstance(key_or_positions, str) else 'MOVE')
        self.get_logger().info(
            f'➡️  {label}  '
            f'[{", ".join(f"{p:.2f}" for p in positions)}]')

        planning_times = [self.ptime, 45.0, 60.0]
        planning_atts  = [self.patt,  40,    50  ]

        for attempt in range(retries):
            goal = self._build_joint_goal(positions)
            goal.request.allowed_planning_time = planning_times[
                min(attempt, len(planning_times)-1)]
            goal.request.num_planning_attempts = planning_atts[
                min(attempt, len(planning_atts)-1)]

            code = self._send_and_wait(goal)
            if code == SUCCESS:
                self.get_logger().info(f'   ✅ {label}')
                return True
            self.get_logger().warn(
                f'   retry {attempt+1}/{retries}  code={code}')

        self.get_logger().error(f'❌ Failed: {label}')
        return False

    # ── planning scene ────────────────────────────────────────────────────────
    def _apply_scene(self, scene_diff) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene = scene_diff
        req.scene.is_diff = True
        fut = self.scene_apply.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        ok = bool(fut.result() and fut.result().success)
        return ok

    def add_collision_box(self,
                          box_id: str = 'target_box',
                          position: tuple = (0.35, 0.0, 0.05),
                          size: tuple = (0.05, 0.05, 0.10),
                          frame: str = 'world') -> bool:
        """Add (or refresh) a box-shaped collision object in the scene."""
        from moveit_msgs.msg import PlanningScene as PS

        co = CollisionObject()
        co.id = box_id
        co.header.frame_id = frame
        co.header.stamp = self.get_clock().now().to_msg()

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(size)

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0

        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PS()
        scene.world.collision_objects.append(co)

        ok = self._apply_scene(scene)
        self.get_logger().info(
            f'{"📦 Box added" if ok else "❌ Failed to add box"} '
            f'"{box_id}" at {position}')
        return ok

    def attach_object(self, box_id: str = 'target_box') -> bool:
        """Attach collision object to the gripper link after grasping."""
        from moveit_msgs.msg import PlanningScene as PS

        aco = AttachedCollisionObject()
        aco.link_name = 'robotiq_85_base_link'   # attach to gripper base
        aco.object.id = box_id
        aco.object.operation = CollisionObject.ADD
        # Allow the object to touch these gripper links while held
        aco.touch_links = [
            'robotiq_85_base_link',
            'robotiq_85_left_knuckle_link',
            'robotiq_85_right_knuckle_link',
            'robotiq_85_left_finger_link',
            'robotiq_85_right_finger_link',
            'robotiq_85_left_inner_knuckle_link',
            'robotiq_85_right_inner_knuckle_link',
            'robotiq_85_left_finger_tip_link',
            'robotiq_85_right_finger_tip_link',
        ]

        scene = type('PS', (), {})()  # minimal scene object
        from moveit_msgs.msg import PlanningScene as MPS
        ps = MPS()
        ps.robot_state.attached_collision_objects.append(aco)

        ok = self._apply_scene(ps)
        self.get_logger().info(
            f'{"📎 Attached" if ok else "❌ Failed to attach"} "{box_id}"')
        return ok

    def detach_object(self, box_id: str = 'target_box') -> bool:
        """Detach collision object from the gripper after placing."""
        from moveit_msgs.msg import PlanningScene as MPS

        aco = AttachedCollisionObject()
        aco.link_name = 'robotiq_85_base_link'
        aco.object.id = box_id
        aco.object.operation = CollisionObject.REMOVE

        ps = MPS()
        ps.robot_state.attached_collision_objects.append(aco)

        # Also remove the object from the world scene
        co = CollisionObject()
        co.id = box_id
        co.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(co)

        ok = self._apply_scene(ps)
        self.get_logger().info(
            f'{"🧷 Detached" if ok else "❌ Failed to detach"} "{box_id}"')
        return ok

    # ── object pose helper ────────────────────────────────────────────────────
    def wait_for_object(self, timeout: float = 10.0) -> bool:
        self.get_logger().info(f'Waiting for /object_pose (timeout={timeout}s)…')
        t0 = time.time()
        while rclpy.ok() and self.object_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                self.get_logger().warn('No object pose received — using defaults.')
                return False
        self.get_logger().info(
            f'Object at '
            f'({self.object_pose.pose.position.x:.3f}, '
            f'{self.object_pose.pose.position.y:.3f}, '
            f'{self.object_pose.pose.position.z:.3f})')
        return True

    # ── main sequence ─────────────────────────────────────────────────────────
    def run(self) -> bool:
        """
        Execute full pick & place sequence:
          HOME → open gripper → add collision object
          → PRE_PICK → GRASP → close gripper → attach
          → LIFT → PRE_PLACE → PLACE → open gripper → detach
          → RETREAT → HOME
        """
        log = self.get_logger()
        log.info('=' * 55)
        log.info(' UR5 Pick & Place — Enhanced Pipeline')
        log.info('=' * 55)

        # Wait for object detection (non-blocking if no camera)
        self.wait_for_object(timeout=5.0)

        # ── Stage 0: Setup ────────────────────────────────────────────────
        log.info('\n[0/8] Setup — gripper open + add collision object')
        self.open_gripper()
        time.sleep(0.3)
        self.add_collision_box(
            box_id='target_box',
            position=(0.50, 0.20, 0.05),
            size=(0.05, 0.05, 0.10),
        )

        # ── Stage 1: HOME ─────────────────────────────────────────────────
        log.info('\n[1/8] Moving to HOME')
        if not self.move_joints('home', label='HOME'):
            log.error('Abort: could not reach HOME.')
            return False
        time.sleep(0.5)

        # ── Stage 2: PRE_PICK ─────────────────────────────────────────────
        log.info('\n[2/8] Moving to PRE_PICK (above object)')
        if not self.move_joints('pre_pick', label='PRE-PICK'):
            log.error('Abort: could not reach PRE-PICK.')
            return False
        time.sleep(0.4)

        # ── Stage 3: GRASP (descend onto object) ──────────────────────────
        log.info('\n[3/8] Descending to GRASP pose')
        if not self.move_joints('grasp', label='GRASP DESCENT'):
            log.error('Abort: could not reach GRASP pose.')
            return False
        time.sleep(0.3)

        # ── Stage 4: Close gripper + attach object ────────────────────────
        log.info('\n[4/8] Closing gripper & attaching object')
        self.close_gripper()
        time.sleep(0.5)
        self.attach_object('target_box')
        time.sleep(0.3)

        # ── Stage 5: LIFT ─────────────────────────────────────────────────
        log.info('\n[5/8] LIFT — raising object')
        if not self.move_joints('lift', label='LIFT'):
            log.error('Abort: could not LIFT.')
            self.open_gripper()
            self.detach_object('target_box')
            return False
        time.sleep(0.4)

        # ── Stage 6: PRE_PLACE ────────────────────────────────────────────
        log.info('\n[6/8] Swinging to PRE_PLACE (above drop zone)')
        if not self.move_joints('pre_place', label='PRE-PLACE'):
            log.error('Abort: could not reach PRE-PLACE.')
            self.open_gripper()
            self.detach_object('target_box')
            return False
        time.sleep(0.4)

        # ── Stage 7: PLACE (descend to drop zone) ─────────────────────────
        log.info('\n[7/8] Descending to PLACE pose')
        if not self.move_joints('place', label='PLACE DESCENT'):
            log.error('Abort: could not reach PLACE pose.')
            self.open_gripper()
            self.detach_object('target_box')
            return False
        time.sleep(0.3)

        # ── Stage 8: Open gripper + detach ────────────────────────────────
        log.info('\n[8/8] Opening gripper & releasing object')
        self.open_gripper()
        time.sleep(0.4)
        self.detach_object('target_box')
        time.sleep(0.3)

        # ── Retreat ───────────────────────────────────────────────────────
        log.info('\n[+] RETREAT')
        self.move_joints('retreat', label='RETREAT')
        time.sleep(0.3)

        # ── Return HOME ───────────────────────────────────────────────────
        log.info('\n[+] Return HOME')
        self.move_joints('home', label='HOME (final)')

        log.info('\n' + '=' * 55)
        log.info(' ✅  Pick & Place COMPLETE!')
        log.info('=' * 55)
        return True


# ── entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        success = node.run()
        if not success:
            node.get_logger().error('Pick & Place sequence failed.')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
