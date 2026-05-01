#!/usr/bin/env python3
"""
pick_place_node.py — UR5 Pick & Place fluide & moderne
========================================================
Pipeline cartésien :
  1. Lecture pose objet sur /object_pose
  2. IK warm-start (seed = état joints courant)
  3. PRE-PICK et PRE-PLACE en planning joint-space (RRTConnect)
  4. DESCENTE / LIFT en /compute_cartesian_path → ligne droite outil
  5. Exécution via /execute_trajectory (action MoveIt2)

Choix de design (les WHY non évidents) :
  - Seed IK = joints courants : KDL converge sur la solution la plus proche
    de la pose actuelle → trajectoires courtes, pas de "détour" articulaire.
  - Tolérance contrainte ±1.2 rad : assez pour résoudre, pas assez pour
    autoriser des configurations "bras retourné" qui produisent de longs
    mouvements articulaires.
  - Cartesian path pour approche/lift : descente verticale exacte au-dessus
    de l'objet, indépendamment de la cinématique. Le RRT générerait une
    courbe parasite.
  - allowed_execution_duration_scaling reste géré par moveit.launch.py.
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
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints, JointConstraint,
    AttachedCollisionObject, CollisionObject,
    MoveItErrorCodes, RobotState, PositionIKRequest, PlanningScene,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetPositionIK, GetCartesianPath,
)

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

# Source unique de vérité — alignée avec initial_positions.yaml.
HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

# Seeds de secours si la seed "joints courants" ne converge pas.
IK_SEED_RIGHT = [ 0.20, -1.30,  1.00, -1.50, -1.5708, 0.0]
IK_SEED_LEFT  = [-0.60, -1.30,  1.00, -1.50, -1.5708, 0.0]
IK_SEED_FRONT = [ 0.00, -1.20,  1.20, -1.57, -1.5708, 0.0]

# ──────────────────────────────────────────────────────────────────────────
# GÉOMÉTRIE wrist_3_link → tool0 (TCP du gripper Robotiq 85)
# ──────────────────────────────────────────────────────────────────────────
# Chaîne URDF :
#   wrist_3_link → robotiq_85_base_link  : xyz=(0, 0.0823, 0)  rpy=(π/2, 0, π/2)
#   robotiq_85_base_link → tool0         : xyz=(0, 0, 0.130)   rpy=(0, 0, 0)
#
# La rotation rpy=(π/2,0,π/2) aligne Z_robotiq sur X_wrist3.
# tool0 est donc à (0.130, 0.0823, 0) dans le repère wrist_3.
#
# Pour saisir un objet par le HAUT, on impose tool0 orienté Z vers le bas :
#   q_tool0 = (1, 0, 0, 0) — rotation 180° autour de X → +Z_tool0 = -Z_world
# Cela donne pour wrist_3 (calcul matriciel via R_world_tool0 * R_wb^T) :
#   q_wrist3 = (0.5, 0.5, -0.5, 0.5)
# Et l'offset (en monde) de tool0 par rapport à wrist_3 :
#   tool0 - wrist_3 = (+0.0823, 0, -0.130)   (X arrière vers la base, Z bas)
# Donc :   wrist_3 = tool0 + (-0.0823, 0, +0.130)
WRIST3_GRASP_QUAT = (0.5, 0.5, -0.5, 0.5)
TOOL0_OFFSET_X = 0.0823   # |Δx| wrist_3 → tool0 (en monde, gripper bas)
TOOL0_OFFSET_Z = 0.130    # |Δz| wrist_3 → tool0 (en monde, gripper bas)


class PickPlaceNode(Node):
    """Pick & Place UR5 + Robotiq 85 — IK warm-start + Cartesian fluide."""

    def __init__(self):
        super().__init__('pick_place_node')

        self.declare_parameter('velocity_scale',     0.45)
        self.declare_parameter('acceleration_scale', 0.35)
        self.declare_parameter('planning_time',      8.0)
        self.declare_parameter('planning_attempts',  6)
        self.declare_parameter('joint_tolerance',    0.02)

        # pre_pick_height / grasp_height : hauteur de tool0 (TCP) au-dessus
        # du CENTRE de l'objet (object_pose.z).
        #   pre_pick = oz + pre_pick_height  → survol
        #   grasp    = oz + grasp_height     → niveau de saisie (TCP au centre objet)
        self.declare_parameter('pre_pick_height', 0.15)
        self.declare_parameter('grasp_height',    0.0)
        self.declare_parameter('cartesian_step',  0.005)

        self.declare_parameter('place_x', 0.50)
        self.declare_parameter('place_y', -0.30)
        self.declare_parameter('place_z', 0.05)

        gp = self.get_parameter
        self.vel        = gp('velocity_scale').value
        self.acc        = gp('acceleration_scale').value
        self.ptime      = gp('planning_time').value
        self.patt       = gp('planning_attempts').value
        self.jtol       = gp('joint_tolerance').value
        self.pre_h      = gp('pre_pick_height').value
        self.grasp_h    = gp('grasp_height').value
        self.cart_step  = gp('cartesian_step').value
        self.place_xyz  = (gp('place_x').value, gp('place_y').value, gp('place_z').value)

        # ── État joints courant (par nom — robuste à l'ordre du topic) ──
        self._current_joints = {n: 0.0 for n in ARM_JOINT_NAMES}
        self._js_received = False
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # ── Action / services MoveIt ──────────────────────────────────────
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        self.get_logger().info('Connexion MoveIt2 …')
        self.move_client.wait_for_server()
        self.exec_client.wait_for_server()
        self.ik_client.wait_for_service()
        self.cart_client.wait_for_service()
        self.scene_client.wait_for_service()
        self.get_logger().info('MoveIt2 prêt ✅')

        # ── Gripper ───────────────────────────────────────────────────────
        self.gripper_client = ActionClient(
            self, GripperCommand, '/hand_controller/gripper_cmd')
        self.has_gripper = self.gripper_client.wait_for_server(timeout_sec=5.0)
        if self.has_gripper:
            self.get_logger().info('Gripper connecté ✅')
        else:
            self.get_logger().warn('⚠️  Gripper simulé (attach/detach uniquement)')

        # ── Subscription pose objet ───────────────────────────────────────
        self.object_pose = None
        self.create_subscription(PoseStamped, '/object_pose', self._object_cb, 10)

        self.get_logger().info(
            f'Paramètres : vel={self.vel:.2f} acc={self.acc:.2f} '
            f'pre_h={self.pre_h:.2f} grasp_h={self.grasp_h:.3f} '
            f'place=({self.place_xyz[0]:.2f},{self.place_xyz[1]:.2f},{self.place_xyz[2]:.2f})')

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────
    def _object_cb(self, msg: PoseStamped):
        self.object_pose = msg

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._current_joints:
                self._current_joints[name] = pos
        self._js_received = True

    def _joints_now(self) -> list:
        return [self._current_joints[n] for n in ARM_JOINT_NAMES]

    # ──────────────────────────────────────────────────────────────────────
    # Helpers pose
    # ──────────────────────────────────────────────────────────────────────
    @staticmethod
    def _pose(tool0_x, tool0_y, tool0_z, quat=WRIST3_GRASP_QUAT) -> Pose:
        """Construit le Pose cible de wrist_3_link pour positionner tool0 en
        (tool0_x, tool0_y, tool0_z) avec orientation Z vers le bas (saisie verticale).

        L'offset URDF wrist_3 → tool0 (gripper pointant vers le bas en monde) est
        appliqué ici : wrist_3 = tool0 + (-0.0823, 0, +0.130).
        Ainsi l'appelant raisonne directement en coordonnées TCP — pas de calcul
        d'offset à dupliquer dans chaque cible (PRE-PICK, GRASP, PLACE…).
        """
        p = Pose()
        p.position.x = float(tool0_x) - TOOL0_OFFSET_X
        p.position.y = float(tool0_y)
        p.position.z = float(tool0_z) + TOOL0_OFFSET_Z
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
            float(q) for q in quat)
        return p

    def _robot_state_now(self) -> RobotState:
        rs = RobotState()
        rs.joint_state.name = list(ARM_JOINT_NAMES)
        rs.joint_state.position = self._joints_now()
        rs.is_diff = False
        return rs

    # ──────────────────────────────────────────────────────────────────────
    # IK
    # ──────────────────────────────────────────────────────────────────────
    def _compute_ik_once(self, pose: Pose, seed: list, timeout=1.0) -> list | None:
        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name       = 'ur5_arm'
        ik.avoid_collisions = True
        ik.timeout.sec      = int(timeout)
        ik.timeout.nanosec  = int((timeout - int(timeout)) * 1e9)

        rs = RobotState()
        rs.joint_state.name = list(ARM_JOINT_NAMES)
        rs.joint_state.position = list(seed)
        ik.robot_state = rs

        target = PoseStamped()
        target.header.frame_id = 'world'
        target.header.stamp    = self.get_clock().now().to_msg()
        target.pose = pose
        ik.pose_stamped = target
        req.ik_request = ik

        fut = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout + 1.5)
        if not fut.result():
            return None
        resp = fut.result()
        if resp.error_code.val != SUCCESS:
            return None

        m = dict(zip(resp.solution.joint_state.name,
                     resp.solution.joint_state.position))
        return [m.get(n, 0.0) for n in ARM_JOINT_NAMES]

    @staticmethod
    def _normalize_joints(joints: list) -> list:
        """Ramène chaque angle dans (-π, π] pour éviter les solutions multi-révolutions
        qui causent des collisions lors de l'interpolation RRT et des sauts IK cartésiens."""
        result = []
        for angle in joints:
            angle = angle % (2.0 * math.pi)
            if angle > math.pi:
                angle -= 2.0 * math.pi
            result.append(angle)
        return result

    def compute_ik(self, x, y, z, label='', extra_seeds=None) -> list | None:
        """IK warm-start : essaie d'abord les joints courants, puis fallbacks."""
        pose = self._pose(x, y, z)
        seeds = [self._joints_now()]
        if extra_seeds:
            seeds += list(extra_seeds)
        seeds += [HOME, IK_SEED_RIGHT, IK_SEED_LEFT, IK_SEED_FRONT]

        # Élimine les doublons proches (norme L1 < 0.05)
        unique = []
        for s in seeds:
            if not any(sum(abs(a-b) for a, b in zip(s, u)) < 0.05 for u in unique):
                unique.append(list(s))

        for i, seed in enumerate(unique):
            j = self._compute_ik_once(pose, seed)
            if j is not None:
                j = self._normalize_joints(j)
                self.get_logger().info(
                    f'IK ✅ {label} (seed {i+1}/{len(unique)}) → '
                    f'[{", ".join(f"{v:.2f}" for v in j)}]')
                return j

        self.get_logger().error(
            f'❌ IK impossible {label} ({x:.3f},{y:.3f},{z:.3f}) — '
            f'pose hors espace de travail ?')
        return None

    # ──────────────────────────────────────────────────────────────────────
    # Joint-space planning (RRTConnect via /move_action)
    # ──────────────────────────────────────────────────────────────────────
    def _build_joint_goal(self, positions, planning_time=None) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = 'ur5_arm'
        r.workspace_parameters.header.frame_id = 'world'
        r.num_planning_attempts = self.patt
        r.allowed_planning_time = planning_time or self.ptime
        r.max_velocity_scaling_factor = self.vel
        r.max_acceleration_scaling_factor = self.acc
        r.start_state = self._robot_state_now()

        c = Constraints()
        for name, pos in zip(ARM_JOINT_NAMES, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = self.jtol
            jc.tolerance_below = self.jtol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints = [c]
        return goal

    def _send_move_goal(self, goal, timeout=60.0) -> int:
        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if not gh:
            return -1
        if not gh.accepted:
            return -2
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        if not rf.result():
            return -3
        return rf.result().result.error_code.val

    def move_joints(self, positions, label='', retries=3) -> bool:
        self.get_logger().info(
            f'➡️  {label} [{", ".join(f"{p:.2f}" for p in positions)}]')
        for attempt in range(retries):
            goal = self._build_joint_goal(
                positions, planning_time=self.ptime + attempt * 4.0)
            code = self._send_move_goal(goal)
            if code == SUCCESS:
                return True
            self.get_logger().warn(f'   retry {attempt+1}/{retries} (code={code})')
        self.get_logger().error(f'❌ {label} échoué')
        return False

    # ──────────────────────────────────────────────────────────────────────
    # Cartesian planning (/compute_cartesian_path → /execute_trajectory)
    # ──────────────────────────────────────────────────────────────────────
    def cartesian_move(self, target_pose: Pose, label='',
                       max_step=None, min_fraction=0.98) -> bool:
        """Plan + execute trajectoire cartésienne linéaire vers target_pose.

        min_fraction relevé à 0.98 : pour la saisie/dépose, accepter une
        trajectoire partielle (e.g. 85%) signifierait s'arrêter au-dessus de
        l'objet — le gripper ne descendrait pas jusqu'au TCP visé.
        """
        max_step = max_step or self.cart_step

        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.header.stamp    = self.get_clock().now().to_msg()
        req.start_state     = self._robot_state_now()
        req.group_name      = 'ur5_arm'
        # tip réel du chain défini dans le SRDF — wrist_3_link.
        req.link_name       = 'wrist_3_link'
        req.waypoints       = [target_pose]
        req.max_step        = float(max_step)
        # jump_threshold=0 désactive la détection de saut articulaire — sûr ici
        # car la trajectoire est courte et le warm-start évite les flips.
        req.jump_threshold  = 0.0
        req.avoid_collisions = True

        fut = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        if not fut.result():
            self.get_logger().error(f'❌ {label} : compute_cartesian_path timeout')
            return False
        resp = fut.result()
        frac = resp.fraction
        if frac < min_fraction:
            self.get_logger().error(
                f'❌ {label} : Cartesian {frac*100:.0f}% (< {min_fraction*100:.0f}%) — '
                f'le TCP n\'atteindrait pas la cible.')
            return False
        self.get_logger().info(f'   📏 {label} cartésien {frac*100:.0f}% planifié')

        # Reparametrisation vitesse/accel (la trajectoire renvoyée par
        # GetCartesianPath utilise par défaut les pleines vitesses joint_limits).
        traj = self._scale_trajectory(resp.solution, self.vel, self.acc)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        gh_fut = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_fut, timeout_sec=5.0)
        gh = gh_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'❌ {label} : ExecuteTrajectory rejeté')
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)
        if not rf.result():
            self.get_logger().error(f'❌ {label} : execution timeout')
            return False
        ok = rf.result().result.error_code.val == SUCCESS
        if ok:
            self.get_logger().info(f'   ✅ {label}')
        else:
            self.get_logger().error(
                f'❌ {label} : execution code={rf.result().result.error_code.val}')
        return ok

    @staticmethod
    def _scale_trajectory(traj, vel_scale: float, acc_scale: float):
        """Étire le timing de la trajectoire pour respecter vel/acc scaling.
        GetCartesianPath retourne une trajectoire à pleine vitesse — ce scaling
        produit le mouvement plus lent et fluide attendu par le contrôleur."""
        if not traj.joint_trajectory.points:
            return traj
        time_scale = 1.0 / max(min(vel_scale, math.sqrt(acc_scale)), 0.01)
        for pt in traj.joint_trajectory.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            t *= time_scale
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            pt.velocities = [v / time_scale for v in pt.velocities]
            pt.accelerations = [a / (time_scale ** 2) for a in pt.accelerations]
        return traj

    # ──────────────────────────────────────────────────────────────────────
    # Gripper
    # ──────────────────────────────────────────────────────────────────────
    def _gripper_cmd(self, position: float) -> bool:
        if not self.has_gripper:
            self.get_logger().info(
                f'[sim] Gripper {"OUVERT" if position == GRIPPER_OPEN else "FERMÉ"}')
            return True
        goal = GripperCommand.Goal()
        goal.command.position   = float(position)
        goal.command.max_effort = GRIPPER_EFFORT
        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return True

    def open_gripper(self):  return self._gripper_cmd(GRIPPER_OPEN)
    def close_gripper(self): return self._gripper_cmd(GRIPPER_CLOSED)

    # ──────────────────────────────────────────────────────────────────────
    # Planning scene
    # ──────────────────────────────────────────────────────────────────────
    def _apply(self, scene: PlanningScene) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene = scene
        req.scene.is_diff = True
        fut = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)

    def add_box(self, box_id, position, size=(0.05, 0.05, 0.10)) -> bool:
        co = CollisionObject()
        co.id = box_id
        co.header.frame_id = 'world'
        co.header.stamp = self.get_clock().now().to_msg()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(p) for p in position)
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        ps = PlanningScene()
        ps.world.collision_objects.append(co)
        ok = self._apply(ps)
        self.get_logger().info(f'{"📦" if ok else "❌"} Box "{box_id}" @ {position}')
        return ok

    def remove_box(self, box_id) -> bool:
        co = CollisionObject()
        co.id = box_id
        co.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.world.collision_objects.append(co)
        return self._apply(ps)

    def attach_box(self, box_id) -> bool:
        aco = AttachedCollisionObject()
        aco.link_name = 'wrist_3_link'
        aco.object.id = box_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = [
            'wrist_3_link', 'wrist_2_link', 'wrist_1_link',
            'robotiq_85_base_link',
            'robotiq_85_left_knuckle_link',  'robotiq_85_right_knuckle_link',
            'robotiq_85_left_finger_link',   'robotiq_85_right_finger_link',
            'robotiq_85_left_inner_knuckle_link', 'robotiq_85_right_inner_knuckle_link',
            'robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link',
        ]
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects.append(aco)
        return self._apply(ps)

    def detach_box(self, box_id) -> bool:
        aco = AttachedCollisionObject()
        aco.link_name = 'wrist_3_link'
        aco.object.id = box_id
        aco.object.operation = CollisionObject.REMOVE
        co = CollisionObject()
        co.id = box_id
        co.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects.append(aco)
        ps.world.collision_objects.append(co)
        return self._apply(ps)

    def _cleanup_scene(self):
        self.remove_box('table')
        self.remove_box('target_box')

    # ──────────────────────────────────────────────────────────────────────
    # Séquence principale
    # ──────────────────────────────────────────────────────────────────────
    def run(self) -> bool:
        log = self.get_logger()
        log.info('=' * 60)
        log.info(' UR5 Pick & Place — Cartésien fluide')
        log.info('=' * 60)

        # ── Attendre /joint_states ────────────────────────────────────────
        t0 = time.time()
        while not self._js_received and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not self._js_received:
            log.warn('⚠️  /joint_states absent — fallback HOME')
            self._current_joints = dict(zip(ARM_JOINT_NAMES, HOME))

        # ── Attendre pose objet ───────────────────────────────────────────
        log.info('Attente /object_pose …')
        t0 = time.time()
        while rclpy.ok() and self.object_pose is None and time.time() - t0 < 10.0:
            time.sleep(0.1)
        if self.object_pose:
            ox = self.object_pose.pose.position.x
            oy = self.object_pose.pose.position.y
            oz = self.object_pose.pose.position.z
            log.info(f'📍 Objet : ({ox:.3f}, {oy:.3f}, {oz:.3f})')
        else:
            ox, oy, oz = 0.50, 0.10, 0.05
            log.warn(f'⚠️  Fallback objet ({ox}, {oy}, {oz})')

        px, py, pz = self.place_xyz

        # ── Setup scène ───────────────────────────────────────────────────
        log.info('\n[0/9] Setup scène')
        self.open_gripper()
        self.add_box('table',      (0.65, 0.0, -0.025), size=(0.8, 0.8, 0.05))
        # Taille alignée sur la géométrie Gazebo de pick_object : 0.05 × 0.05 × 0.10
        self.add_box('target_box', (ox,   oy,    oz),   size=(0.05, 0.05, 0.10))

        # ── IK : 4 poses cibles, warm-start chaîné ────────────────────────
        log.info('\n📐 Calcul IK (warm-start chaîné)')
        pre_pick_j = self.compute_ik(ox, oy, oz + self.pre_h, label='PRE-PICK')
        if pre_pick_j is None:
            self._cleanup_scene()
            return False
        # Seeds chainées : chaque IK utilise la précédente comme indice
        grasp_j     = self.compute_ik(ox, oy, oz + self.grasp_h,
                                      label='GRASP', extra_seeds=[pre_pick_j])
        pre_place_j = self.compute_ik(px, py, pz + self.pre_h,
                                      label='PRE-PLACE', extra_seeds=[pre_pick_j])
        place_j     = self.compute_ik(px, py, pz + self.grasp_h,
                                      label='PLACE',
                                      extra_seeds=[pre_place_j] if pre_place_j else None)
        if any(j is None for j in (grasp_j, pre_place_j, place_j)):
            self._cleanup_scene()
            return False

        # Poses cartésiennes pour les segments rectilignes
        grasp_pose     = self._pose(ox, oy, oz + self.grasp_h)
        pre_pick_pose  = self._pose(ox, oy, oz + self.pre_h)
        place_pose     = self._pose(px, py, pz + self.grasp_h)
        pre_place_pose = self._pose(px, py, pz + self.pre_h)

        # ── [1/9] HOME ────────────────────────────────────────────────────
        log.info('\n[1/9] HOME')
        if not self.move_joints(HOME, 'HOME'):
            self._cleanup_scene()
            return False

        # ── [2/9] PRE-PICK (joint-space, RRT) ─────────────────────────────
        log.info('\n[2/9] PRE-PICK')
        if not self.move_joints(pre_pick_j, 'PRE-PICK'):
            self._cleanup_scene()
            return False

        # ── [3/9] DESCENTE Cartésienne ────────────────────────────────────
        log.info('\n[3/9] GRASP (descente cartésienne)')
        # Retire l'objet de la scène — sinon les doigts entrent en collision.
        self.remove_box('target_box')
        if not self.cartesian_move(grasp_pose, 'GRASP'):
            self._cleanup_scene()
            return False

        # ── [4/9] Saisie ──────────────────────────────────────────────────
        log.info('\n[4/9] CLOSE + ATTACH')
        self.close_gripper()
        self.attach_box('target_box')

        # ── [5/9] LIFT cartésien ──────────────────────────────────────────
        log.info('\n[5/9] LIFT (cartésien)')
        if not self.cartesian_move(pre_pick_pose, 'LIFT'):
            self.open_gripper()
            self.detach_box('target_box')
            self._cleanup_scene()
            return False

        # ── [6/9] PRE-PLACE ───────────────────────────────────────────────
        log.info('\n[6/9] PRE-PLACE')
        if not self.move_joints(pre_place_j, 'PRE-PLACE'):
            self.open_gripper()
            self.detach_box('target_box')
            self._cleanup_scene()
            return False

        # ── [7/9] PLACE (descente cartésienne) ────────────────────────────
        log.info('\n[7/9] PLACE (descente cartésienne)')
        if not self.cartesian_move(place_pose, 'PLACE'):
            self.open_gripper()
            self.detach_box('target_box')
            self._cleanup_scene()
            return False

        # ── [8/9] Relâche + lift ──────────────────────────────────────────
        log.info('\n[8/9] RELEASE + LIFT')
        self.open_gripper()
        self.detach_box('target_box')
        if not self.cartesian_move(pre_place_pose, 'RETREAT'):
            log.warn('Lift cartésien échoué — fallback joint-space')
            self.move_joints(pre_place_j, 'RETREAT (joint)')

        # ── [9/9] HOME ────────────────────────────────────────────────────
        log.info('\n[9/9] HOME final')
        self.move_joints(HOME, 'HOME final')

        self._cleanup_scene()

        log.info('\n' + '=' * 60)
        log.info(' ✅  Pick & Place TERMINÉ')
        log.info('=' * 60)
        return True


def main(args=None):
    rclpy.init(args=args)

    # MultiThreadedExecutor obligatoire : run() bloque le thread principal
    # avec spin_until_future_complete() — l'executor doit traiter les
    # callbacks (subs, actions, services) dans des threads workers parallèles.
    executor = MultiThreadedExecutor(num_threads=4)
    node = PickPlaceNode()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if not node.run():
            node.get_logger().error('Séquence échouée.')
    except KeyboardInterrupt:
        node.get_logger().info("Interrompu par l'utilisateur.")
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
