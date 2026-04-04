#!/usr/bin/env python3
"""
pick_place_node.py — UR5 Pick & Place avec cinématique inverse automatique
=========================================================================
La position de l'objet (lue sur /object_pose) est convertie automatiquement
en angles articulaires via le service /compute_ik de MoveIt2.

Pipeline :
  1. Lire la pose 3D de l'objet (x, y, z) depuis /object_pose
  2. Appeler compute_ik pour trouver les angles joints correspondants
  3. Planifier et exécuter la séquence pick & place
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint,
    AttachedCollisionObject, CollisionObject,
    MoveItErrorCodes, RobotState,
    PositionIKRequest,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from shape_msgs.msg import SolidPrimitive

SUCCESS = MoveItErrorCodes.SUCCESS

GRIPPER_OPEN   = 0.0
GRIPPER_CLOSED = 0.72
GRIPPER_EFFORT = 40.0

ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Position HOME fixe (bras replié, position sûre)
HOME = [0.00, -1.57, 1.57, -1.57, -1.57, 0.00]

# Position de depot (fixe, pas besoin d'IK)
PLACE_JOINTS    = [1.57, -0.85, 1.55, -2.27, -1.57, 0.00]
PREPLACE_JOINTS = [1.57, -1.10, 1.30, -1.77, -1.57, 0.00]
RETREAT_JOINTS  = [1.57, -1.10, 1.30, -1.77, -1.57, 0.00]


class PickPlaceNode(Node):
    """Pick & Place avec IK automatique pour le UR5 + Robotiq 85."""

    def __init__(self):
        super().__init__('pick_place_node')

        self.declare_parameter('velocity_scale',     0.10)
        self.declare_parameter('acceleration_scale', 0.10)
        self.declare_parameter('planning_time',      30.0)
        self.declare_parameter('planning_attempts',  30)
        self.declare_parameter('joint_tolerance',    0.05)
        self.declare_parameter('pre_pick_height',    0.15)
        self.declare_parameter('grasp_height',       0.02)

        self.vel        = self.get_parameter('velocity_scale').value
        self.acc        = self.get_parameter('acceleration_scale').value
        self.ptime      = self.get_parameter('planning_time').value
        self.patt       = self.get_parameter('planning_attempts').value
        self.jtol       = self.get_parameter('joint_tolerance').value
        self.pre_height = self.get_parameter('pre_pick_height').value
        self.grasp_h    = self.get_parameter('grasp_height').value

        # MoveGroup
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for /move_action …')
        self.move_client.wait_for_server()
        self.get_logger().info('Connected to /move_action ✅')

        # Service IK
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('Waiting for /compute_ik …')
        self.ik_client.wait_for_service()
        self.get_logger().info('Connected to /compute_ik ✅')

        # Gripper
        self.gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_85_gripper_controller/gripper_cmd')
        if self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.has_gripper = True
            self.get_logger().info('Connected to gripper ✅')
        else:
            self.has_gripper = False
            self.get_logger().warn('⚠️  Gripper simulé (attach/detach only)')

        # Planning scene
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.scene_client.wait_for_service()

        # Object pose
        self.object_pose = None
        self.create_subscription(
            PoseStamped, '/object_pose', self._object_cb, 10)

        self.get_logger().info('PickPlaceNode prêt ✅')

    # ── Callback objet ────────────────────────────────────────────────────────
    def _object_cb(self, msg):
        self.object_pose = msg

    # ── Cinématique inverse ────────────────────────────────────────────────────
    def compute_ik(self, x, y, z, qx=0.0, qy=0.707, qz=0.0, qw=0.707,
                   timeout=5.0):
        """
        Calcule les angles articulaires pour atteindre la pose (x, y, z).
        Orientation par défaut : outil pointant vers le bas (qy=0.707, qw=0.707).
        Retourne une liste de 6 angles en radians, ou None si impossible.
        """
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()

        ik_req.group_name = 'ur5_arm'
        ik_req.avoid_collisions = True
        ik_req.timeout.sec = int(timeout)
        ik_req.timeout.nanosec = 0

        target = PoseStamped()
        target.header.frame_id = 'world'
        target.header.stamp    = self.get_clock().now().to_msg()
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)
        target.pose.orientation.x = float(qx)
        target.pose.orientation.y = float(qy)
        target.pose.orientation.z = float(qz)
        target.pose.orientation.w = float(qw)

        ik_req.pose_stamped = target
        req.ik_request = ik_req

        fut = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout + 2.0)

        if not fut.result():
            self.get_logger().error('IK service timeout')
            return None

        resp = fut.result()
        if resp.error_code.val != SUCCESS:
            self.get_logger().error(
                f'IK failed — code={resp.error_code.val} '
                f'pour pose ({x:.3f}, {y:.3f}, {z:.3f})')
            return None

        # Extraire les angles dans l'ordre des joints du bras
        joint_map = dict(zip(
            resp.solution.joint_state.name,
            resp.solution.joint_state.position
        ))
        joints = [joint_map.get(n, 0.0) for n in ARM_JOINT_NAMES]
        self.get_logger().info(
            f'IK ✅ pour ({x:.3f},{y:.3f},{z:.3f}) → '
            f'[{", ".join(f"{j:.2f}" for j in joints)}]')
        return joints

    # ── Mouvement articulaire ─────────────────────────────────────────────────
    def _build_goal(self, positions, tol=None):
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

    def _send_and_wait(self, goal, timeout=90.0):
        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        gh = fut.result()
        if not gh or not gh.accepted:
            return -1
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        r = rf.result()
        return r.result.error_code.val if r else -2

    def move_to(self, positions, label='', retries=3) -> bool:
        self.get_logger().info(
            f'➡️  {label}  [{", ".join(f"{p:.2f}" for p in positions)}]')
        times = [self.ptime, 45.0, 60.0]
        atts  = [self.patt,  40,    50  ]
        for attempt in range(retries):
            goal = self._build_goal(positions)
            goal.request.allowed_planning_time = times[min(attempt, 2)]
            goal.request.num_planning_attempts = atts[min(attempt, 2)]
            code = self._send_and_wait(goal)
            if code == SUCCESS:
                self.get_logger().info(f'   ✅ {label}')
                return True
            self.get_logger().warn(f'   retry {attempt+1}/{retries}  code={code}')
        self.get_logger().error(f'❌ {label} échoué')
        return False

    # ── Gripper ───────────────────────────────────────────────────────────────
    def open_gripper(self):
        if not self.has_gripper:
            self.get_logger().info('[sim] Gripper OPEN 🟢')
            return True
        goal = GripperCommand.Goal()
        goal.command.position   = GRIPPER_OPEN
        goal.command.max_effort = GRIPPER_EFFORT
        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return True

    def close_gripper(self):
        if not self.has_gripper:
            self.get_logger().info('[sim] Gripper CLOSE 🔴')
            return True
        goal = GripperCommand.Goal()
        goal.command.position   = GRIPPER_CLOSED
        goal.command.max_effort = GRIPPER_EFFORT
        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return True

    # ── Planning scene ────────────────────────────────────────────────────────
    def _apply(self, scene):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        req.scene.is_diff = True
        fut = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)

    def add_box(self, box_id, position, size=(0.05, 0.05, 0.10)):
        from moveit_msgs.msg import PlanningScene
        co = CollisionObject()
        co.id = box_id
        co.header.frame_id = 'world'
        co.header.stamp    = self.get_clock().now().to_msg()
        prim = SolidPrimitive()
        prim.type       = SolidPrimitive.BOX
        prim.dimensions = list(size)
        pose = Pose()
        pose.position.x    = float(position[0])
        pose.position.y    = float(position[1])
        pose.position.z    = float(position[2])
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        ps = PlanningScene()
        ps.world.collision_objects.append(co)
        ok = self._apply(ps)
        self.get_logger().info(
            f'{"📦" if ok else "❌"} Box "{box_id}" @ {position}')
        return ok

    def attach_box(self, box_id):
        from moveit_msgs.msg import PlanningScene
        aco = AttachedCollisionObject()
        aco.link_name        = 'robotiq_85_base_link'
        aco.object.id        = box_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = [
            'robotiq_85_base_link',
            'robotiq_85_left_finger_link', 'robotiq_85_right_finger_link',
            'robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link',
        ]
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects.append(aco)
        ok = self._apply(ps)
        self.get_logger().info(f'{"📎" if ok else "❌"} Attached "{box_id}"')
        return ok

    def detach_box(self, box_id):
        from moveit_msgs.msg import PlanningScene
        aco = AttachedCollisionObject()
        aco.link_name        = 'robotiq_85_base_link'
        aco.object.id        = box_id
        aco.object.operation = CollisionObject.REMOVE
        co = CollisionObject()
        co.id        = box_id
        co.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects.append(aco)
        ps.world.collision_objects.append(co)
        ok = self._apply(ps)
        self.get_logger().info(f'{"🧷" if ok else "❌"} Detached "{box_id}"')
        return ok

    # ── Séquence principale ───────────────────────────────────────────────────
    def run(self) -> bool:
        log = self.get_logger()
        log.info('=' * 55)
        log.info(' UR5 Pick & Place — IK automatique')
        log.info('=' * 55)

        # Attendre la pose de l'objet
        log.info('Attente de /object_pose …')
        t0 = time.time()
        while rclpy.ok() and self.object_pose is None and time.time()-t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Position de l'objet
        if self.object_pose:
            ox = self.object_pose.pose.position.x
            oy = self.object_pose.pose.position.y
            oz = self.object_pose.pose.position.z
            log.info(f'📍 Objet détecté à ({ox:.3f}, {oy:.3f}, {oz:.3f})')
        else:
            ox, oy, oz = 0.40, 0.00, 0.05
            log.warn(f'⚠️  Fallback position ({ox}, {oy}, {oz})')

        # ── Calcul IK automatique ─────────────────────────────────────────
        log.info('\n📐 Calcul cinématique inverse …')

        # Orientation outil vers le bas : qy=0.707, qw=0.707
        qx, qy, qz, qw = 0.0, 0.707, 0.0, 0.707

        pre_pick_joints = self.compute_ik(ox, oy, oz + self.pre_height,
                                          qx, qy, qz, qw)
        grasp_joints    = self.compute_ik(ox, oy, oz + self.grasp_h,
                                          qx, qy, qz, qw)

        if pre_pick_joints is None or grasp_joints is None:
            log.error('❌ IK impossible pour la position de l\'objet.')
            log.error('   Vérifiez que l\'objet est dans l\'espace de travail du UR5.')
            return False

        # ── Stage 0 : Setup ───────────────────────────────────────────────
        log.info('\n[0/8] Setup')
        self.open_gripper()
        self.add_box('target_box', (ox, oy, oz))

        # ── Stage 1 : HOME ────────────────────────────────────────────────
        log.info('\n[1/8] HOME')
        if not self.move_to(HOME, 'HOME'):
            return False
        time.sleep(0.5)

        # ── Stage 2 : PRE-PICK (IK calculé) ──────────────────────────────
        log.info(f'\n[2/8] PRE-PICK (IK) — au-dessus de ({ox:.2f},{oy:.2f})')
        if not self.move_to(pre_pick_joints, 'PRE-PICK'):
            return False
        time.sleep(0.4)

        # ── Stage 3 : GRASP (IK calculé) ─────────────────────────────────
        log.info(f'\n[3/8] DESCEND (IK) — vers ({ox:.2f},{oy:.2f},{oz:.2f})')
        if not self.move_to(grasp_joints, 'GRASP'):
            return False
        time.sleep(0.3)

        # ── Stage 4 : Fermer gripper + attacher ───────────────────────────
        log.info('\n[4/8] GRASP + ATTACH')
        self.close_gripper()
        time.sleep(0.5)
        self.attach_box('target_box')
        time.sleep(0.3)

        # ── Stage 5 : LIFT (retour PRE-PICK) ─────────────────────────────
        log.info('\n[5/8] LIFT')
        if not self.move_to(pre_pick_joints, 'LIFT'):
            self.open_gripper()
            self.detach_box('target_box')
            return False
        time.sleep(0.4)

        # ── Stage 6 : PRE-PLACE ───────────────────────────────────────────
        log.info('\n[6/8] PRE-PLACE')
        if not self.move_to(PREPLACE_JOINTS, 'PRE-PLACE'):
            self.open_gripper()
            self.detach_box('target_box')
            return False
        time.sleep(0.4)

        # ── Stage 7 : PLACE ───────────────────────────────────────────────
        log.info('\n[7/8] PLACE')
        if not self.move_to(PLACE_JOINTS, 'PLACE'):
            self.open_gripper()
            self.detach_box('target_box')
            return False
        time.sleep(0.3)

        # ── Stage 8 : Ouvrir gripper + détacher ───────────────────────────
        log.info('\n[8/8] RELEASE')
        self.open_gripper()
        time.sleep(0.4)
        self.detach_box('target_box')
        time.sleep(0.3)

        # ── Retreat + HOME ────────────────────────────────────────────────
        log.info('\n[+] RETREAT')
        self.move_to(RETREAT_JOINTS, 'RETREAT')
        time.sleep(0.3)
        log.info('\n[+] HOME final')
        self.move_to(HOME, 'HOME final')

        log.info('\n' + '=' * 55)
        log.info(' ✅  Pick & Place COMPLETE!')
        log.info('=' * 55)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        success = node.run()
        if not success:
            node.get_logger().error('Séquence échouée.')
    except KeyboardInterrupt:
        node.get_logger().info('Arrêté par l\'utilisateur.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
