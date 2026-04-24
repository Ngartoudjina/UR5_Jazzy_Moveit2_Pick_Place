#!/usr/bin/env python3
"""
pick_place_node.py — UR5 Pick & Place avec cinématique inverse automatique
=========================================================================
Pipeline :
  1. Lire la pose 3D de l'objet (x, y, z) depuis /object_pose
  2. Appeler compute_ik pour trouver les angles joints correspondants
  3. Planifier et exécuter la séquence pick & place

CORRECTIONS APPLIQUÉES (rapport Avril 2026 + correctifs runtime) :
  [BUG 1.1] moveit_controllers.yaml : action_ns gripper_cmd (fichier séparé)
  [BUG 1.2] MultiThreadedExecutor dans main()
  [BUG 1.3] time.sleep() remplace rclpy.spin_once() dans la boucle /object_pose
  [BUG 1.4] _wait_future() : attente Future compatible avec executor externe —
             spin_until_future_complete(self,...) est INTERDIT quand le node est
             déjà géré par un MultiThreadedExecutor (deadlock / INVALID_MOTION_PLAN)
  [BUG 2.1] attach/detach sur wrist_3_link (au lieu de robotiq_85_base_link)
  [BUG 2.2] grasp_height=0.16 (offset TCP Robotiq 85 correctement compensé)
  [BUG 2.3] Table ajoutée dans la scène de collision MoveIt2
  [BUG 2.6] Normalisation angles IK dans [-pi, pi] — KDL retourne parfois des
             solutions équivalentes hors plage (ex: -4.45, 5.92 rad) qui
             déclenchent INVALID_MOTION_PLAN (code -4)
  [QUALITE 3.2] Messages d'erreur distincts dans _send_and_wait()
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
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

# Position HOME fixe (bras replié, position sure)
HOME = [0.50, -1.57, 1.57, -1.57, -1.57, 0.00]

# Position de depot (fixe, pas besoin d'IK)
PLACE_JOINTS    = [1.57, -0.85, 1.55, -2.27, -1.57, 0.00]
PREPLACE_JOINTS = [1.57, -1.10, 1.30, -1.77, -1.57, 0.00]
RETREAT_JOINTS  = [1.57, -1.10, 1.30, -1.77, -1.57, 0.00]

# Links du gripper Robotiq 85 pour touch_links [FIX 2.1]
GRIPPER_LINKS = [
    'robotiq_85_base_link',
    'robotiq_85_left_knuckle_link',       'robotiq_85_right_knuckle_link',
    'robotiq_85_left_finger_link',        'robotiq_85_right_finger_link',
    'robotiq_85_left_inner_knuckle_link', 'robotiq_85_right_inner_knuckle_link',
    'robotiq_85_left_finger_tip_link',    'robotiq_85_right_finger_tip_link',
]


def _normalize_angle(angle: float) -> float:
    """
    [FIX 2.6] Ramene un angle dans [-pi, pi] par decalage de 2*pi.
    KDL retourne parfois des solutions hors plage equivalentes.
    Ex: -4.45 rad -> 1.83 rad ; 5.92 rad -> -0.36 rad
    MoveIt2 les rejette avec INVALID_MOTION_PLAN (code -4) car ils
    violent les joint_limits du SRDF (qui attendend [-pi, pi]).
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _normalize_joints(joints):
    """Normalise tous les angles d'une solution IK dans [-pi, pi]."""
    return [_normalize_angle(j) for j in joints]


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
        # [FIX 2.2] grasp_height corrige : 0.02 -> 0.16 m
        self.declare_parameter('grasp_height',       0.16)

        self.vel        = self.get_parameter('velocity_scale').value
        self.acc        = self.get_parameter('acceleration_scale').value
        self.ptime      = self.get_parameter('planning_time').value
        self.patt       = self.get_parameter('planning_attempts').value
        self.jtol       = self.get_parameter('joint_tolerance').value
        self.pre_height = self.get_parameter('pre_pick_height').value
        self.grasp_h    = self.get_parameter('grasp_height').value

        # MoveGroup
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for /move_action ...')
        self.move_client.wait_for_server()
        self.get_logger().info('Connected to /move_action OK')

        # Service IK
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('Waiting for /compute_ik ...')
        self.ik_client.wait_for_service()
        self.get_logger().info('Connected to /compute_ik OK')

        # Gripper
        self.gripper_client = ActionClient(
            self, GripperCommand, '/hand_controller/gripper_cmd')
        if self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.has_gripper = True
            self.get_logger().info('Connected to gripper OK')
        else:
            self.has_gripper = False
            self.get_logger().warn('Gripper simule (attach/detach only)')

        # Planning scene
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.scene_client.wait_for_service()

        # Object pose
        self.object_pose = None
        self.create_subscription(
            PoseStamped, '/object_pose', self._object_cb, 10)

        self.get_logger().info('PickPlaceNode pret')

    # ── [FIX 1.4] Attente Future sans spin_until_future_complete ─────────────
    def _wait_future(self, future, timeout: float = 10.0) -> bool:
        """
        Attente passive d'un Future, compatible MultiThreadedExecutor.

        spin_until_future_complete(self, future) prend le verrou interne du node
        et tente de le spinner depuis le thread appelant. Quand le node est deja
        gere par un MultiThreadedExecutor dans un autre thread :
          - double-acquisition du verrou -> deadlock, OU
          - race condition sur la file de callbacks -> INVALID_MOTION_PLAN (code -4)

        Solution : polling avec time.sleep(0.05). L'executor tourne librement
        dans ses threads workers et resout les futures ; on attend juste le flag done.
        """
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.05)
        return True

    # ── Callback objet ────────────────────────────────────────────────────────
    def _object_cb(self, msg):
        self.object_pose = msg

    # ── Cinematique inverse ───────────────────────────────────────────────────
    def compute_ik(self, x, y, z,
                   qx=0.0, qy=0.707, qz=0.0, qw=0.707,
                   timeout=5.0):
        """
        Calcule les angles articulaires pour atteindre la pose (x, y, z).
        Retourne une liste de 6 angles normalises dans [-pi, pi], ou None.
        """
        req    = GetPositionIK.Request()
        ik_req = PositionIKRequest()

        ik_req.group_name       = 'ur5_arm'
        ik_req.avoid_collisions = True
        ik_req.timeout.sec      = int(timeout)
        ik_req.timeout.nanosec  = 0

        # Seed HOME -> guide KDL vers une solution canonique proche de [-pi, pi]
        seed = RobotState()
        seed.joint_state.name     = ARM_JOINT_NAMES
        seed.joint_state.position = list(HOME)
        ik_req.robot_state = seed

        target = PoseStamped()
        target.header.frame_id    = 'world'
        target.header.stamp       = self.get_clock().now().to_msg()
        target.pose.position.x    = float(x)
        target.pose.position.y    = float(y)
        target.pose.position.z    = float(z)
        target.pose.orientation.x = float(qx)
        target.pose.orientation.y = float(qy)
        target.pose.orientation.z = float(qz)
        target.pose.orientation.w = float(qw)

        ik_req.pose_stamped = target
        req.ik_request      = ik_req

        fut = self.ik_client.call_async(req)
        if not self._wait_future(fut, timeout=timeout + 2.0):  # [FIX 1.4]
            self.get_logger().error('IK service timeout')
            return None

        resp = fut.result()
        if resp is None or resp.error_code.val != SUCCESS:
            code = resp.error_code.val if resp else 'None'
            self.get_logger().error(
                f'IK failed code={code} pour ({x:.3f},{y:.3f},{z:.3f})')
            return None

        joint_map = dict(zip(
            resp.solution.joint_state.name,
            resp.solution.joint_state.position,
        ))
        raw    = [joint_map.get(n, 0.0) for n in ARM_JOINT_NAMES]
        joints = _normalize_joints(raw)  # [FIX 2.6]

        self.get_logger().info(
            f'IK OK ({x:.3f},{y:.3f},{z:.3f})'
            f' raw=[{", ".join(f"{r:.2f}" for r in raw)}]'
            f' norm=[{", ".join(f"{j:.2f}" for j in joints)}]')
        return joints

    # ── Construction goal MoveGroup ───────────────────────────────────────────
    def _build_goal(self, positions, tol=None, planning_time=None):
        tol = tol or self.jtol
        goal = MoveGroup.Goal()
        req  = goal.request

        req.group_name = 'ur5_arm'
        req.workspace_parameters.header.frame_id = 'world'
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x =  2.0
        req.workspace_parameters.max_corner.y =  2.0
        req.workspace_parameters.max_corner.z =  2.0

        req.num_planning_attempts           = self.patt
        req.allowed_planning_time           = planning_time or self.ptime
        req.max_velocity_scaling_factor     = self.vel
        req.max_acceleration_scaling_factor = self.acc
        req.start_state.is_diff             = True

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

    # ── Envoi action + attente resultat ──────────────────────────────────────
    def _send_and_wait(self, goal, timeout=90.0):
        """[FIX 1.4 + FIX 3.2] Attente compatible MTEX + messages distincts."""
        fut_goal = self.move_client.send_goal_async(goal)
        if not self._wait_future(fut_goal, timeout=timeout):
            self.get_logger().error(
                'ECHEC send_goal timeout — /move_action ne repond pas. '
                'MoveGroup est-il lance ? (ros2 launch ur5_moveit moveit.launch.py)')
            return -1

        gh = fut_goal.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(
                'ECHEC goal refuse par MoveGroup — verifiez le SRDF '
                'et que les positions cibles sont dans les limites.')
            return -1

        fut_result = gh.get_result_async()
        if not self._wait_future(fut_result, timeout=timeout):
            self.get_logger().error(
                f'ECHEC result timeout ({timeout}s) — '
                'augmentez goal_time dans ros2_controllers.yaml.')
            return -2

        r = fut_result.result()
        return r.result.error_code.val if r else -2

    # ── Mouvement joint-space avec retry ──────────────────────────────────────
    def move_to(self, positions, label='', retries=3) -> bool:
        self.get_logger().info(
            f'-> {label}  [{", ".join(f"{p:.2f}" for p in positions)}]')

        for attempt in range(retries):
            goal = self._build_goal(
                positions,
                planning_time=self.ptime + attempt * 5.0)
            code = self._send_and_wait(goal)

            if code == SUCCESS:
                self.get_logger().info(f'   OK {label}')
                return True

            self.get_logger().warn(
                f'   retry {attempt+1}/{retries}  code={code}')
            time.sleep(0.5)

        self.get_logger().error(f'ECHEC {label} apres {retries} tentatives')
        return False

    # ── Gripper ───────────────────────────────────────────────────────────────
    def _gripper_cmd(self, position: float) -> bool:
        if not self.has_gripper:
            state = 'OPEN' if position == GRIPPER_OPEN else 'CLOSE'
            self.get_logger().info(f'[sim] Gripper {state}')
            return True
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = GRIPPER_EFFORT
        fut = self.gripper_client.send_goal_async(goal)
        self._wait_future(fut, timeout=5.0)  # [FIX 1.4]
        return True

    def open_gripper(self):  return self._gripper_cmd(GRIPPER_OPEN)
    def close_gripper(self): return self._gripper_cmd(GRIPPER_CLOSED)

    # ── Planning scene ────────────────────────────────────────────────────────
    def _apply(self, scene):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        req.scene.is_diff = True
        fut = self.scene_client.call_async(req)
        self._wait_future(fut, timeout=5.0)  # [FIX 1.4]
        return bool(fut.result() and fut.result().success)

    def add_box(self, box_id, position, size=(0.05, 0.05, 0.10)):
        from moveit_msgs.msg import PlanningScene
        co = CollisionObject()
        co.id              = box_id
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
        self.get_logger().info(f'{"[BOX]" if ok else "[ERR]"} "{box_id}" @ {position}')
        return ok

    def attach_box(self, box_id):
        """[FIX 2.1] Attacher sur wrist_3_link, tip du groupe ur5_arm."""
        from moveit_msgs.msg import PlanningScene
        aco = AttachedCollisionObject()
        aco.link_name        = 'wrist_3_link'  # [FIX 2.1]
        aco.object.id        = box_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links      = GRIPPER_LINKS   # [FIX 2.1]
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects.append(aco)
        ok = self._apply(ps)
        self.get_logger().info(f'{"[ATT]" if ok else "[ERR]"} Attached "{box_id}"')
        return ok

    def detach_box(self, box_id):
        """[FIX 2.1] Detacher depuis wrist_3_link."""
        from moveit_msgs.msg import PlanningScene
        aco = AttachedCollisionObject()
        aco.link_name        = 'wrist_3_link'  # [FIX 2.1]
        aco.object.id        = box_id
        aco.object.operation = CollisionObject.REMOVE
        co = CollisionObject()
        co.id        = box_id
        co.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects.append(aco)
        ps.world.collision_objects.append(co)
        ok = self._apply(ps)
        self.get_logger().info(f'{"[DET]" if ok else "[ERR]"} Detached "{box_id}"')
        return ok

    # ── Sequence principale ───────────────────────────────────────────────────
    def run(self) -> bool:
        log = self.get_logger()
        log.info('=' * 55)
        log.info(' UR5 Pick & Place — IK automatique')
        log.info('=' * 55)

        # [FIX 1.3] Attente passive — PAS de spin_once()
        log.info('Attente de /object_pose ...')
        t0 = time.time()
        while rclpy.ok() and self.object_pose is None and time.time() - t0 < 10.0:
            time.sleep(0.1)

        if self.object_pose:
            ox = self.object_pose.pose.position.x
            oy = self.object_pose.pose.position.y
            oz = self.object_pose.pose.position.z
            log.info(f'Objet a ({ox:.3f}, {oy:.3f}, {oz:.3f})')
        else:
            ox, oy, oz = 0.40, 0.00, 0.05
            log.warn(f'Fallback position ({ox}, {oy}, {oz})')

        # IK
        log.info('Calcul cinematique inverse ...')
        qx, qy, qz, qw = 0.0, 0.707, 0.0, 0.707   # outil vers le bas

        pre_pick_joints = self.compute_ik(ox, oy, oz + self.pre_height, qx, qy, qz, qw)
        grasp_joints    = self.compute_ik(ox, oy, oz + self.grasp_h,    qx, qy, qz, qw)

        if pre_pick_joints is None or grasp_joints is None:
            log.error('IK impossible — objet hors espace de travail UR5.')
            return False

        # Stage 0 — Setup
        log.info('[0/8] Setup')
        self.open_gripper()
        self.add_box('target_box', (ox, oy, oz))
        self.add_box('table', (0.65, 0.0, -0.025), size=(0.8, 0.8, 0.05))  # [FIX 2.3]

        # Stage 1 — HOME
        log.info('[1/8] HOME')
        if not self.move_to(HOME, 'HOME'):
            return False
        time.sleep(0.5)

        # Stage 2 — PRE-PICK
        log.info('[2/8] PRE-PICK')
        if not self.move_to(pre_pick_joints, 'PRE-PICK'):
            return False
        time.sleep(0.4)

        # Stage 3 — GRASP DESCENT
        log.info('[3/8] DESCEND GRASP')
        if not self.move_to(grasp_joints, 'GRASP'):
            return False
        time.sleep(0.3)

        # Stage 4 — Close + Attach
        log.info('[4/8] CLOSE + ATTACH')
        self.close_gripper()
        time.sleep(0.5)
        self.attach_box('target_box')
        time.sleep(0.3)

        # Stage 5 — LIFT
        log.info('[5/8] LIFT')
        if not self.move_to(pre_pick_joints, 'LIFT'):
            self.open_gripper(); self.detach_box('target_box')
            return False
        time.sleep(0.4)

        # Stage 6 — PRE-PLACE
        log.info('[6/8] PRE-PLACE')
        if not self.move_to(PREPLACE_JOINTS, 'PRE-PLACE'):
            self.open_gripper(); self.detach_box('target_box')
            return False
        time.sleep(0.4)

        # Stage 7 — PLACE
        log.info('[7/8] PLACE')
        if not self.move_to(PLACE_JOINTS, 'PLACE'):
            self.open_gripper(); self.detach_box('target_box')
            return False
        time.sleep(0.3)

        # Stage 8 — Release
        log.info('[8/8] RELEASE')
        self.open_gripper()
        time.sleep(0.4)
        self.detach_box('target_box')
        time.sleep(0.3)

        # Retreat + HOME
        self.move_to(RETREAT_JOINTS, 'RETREAT')
        time.sleep(0.3)
        self.move_to(HOME, 'HOME final')

        log.info('=' * 55)
        log.info(' Pick & Place COMPLETE!')
        log.info('=' * 55)
        return True


# ── main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()

    # [FIX 1.2] MultiThreadedExecutor — callbacks action/service dans threads workers
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # run() dans un thread dedie ; executor.spin() dans le thread principal
    # Toutes les attentes dans run() utilisent _wait_future() [FIX 1.4]
    result_box = [None]

    def _run():
        result_box[0] = node.run()

    worker = threading.Thread(target=_run, daemon=True)
    worker.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Arrete par utilisateur.')
    finally:
        worker.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    if result_box[0] is False:
        import sys
        sys.exit(1)


if __name__ == '__main__':
    main()
