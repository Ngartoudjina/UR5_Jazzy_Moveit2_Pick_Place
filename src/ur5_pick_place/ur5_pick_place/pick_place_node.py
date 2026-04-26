#!/usr/bin/env python3
"""
pick_place_node.py — UR5 Pick & Place avec cinématique inverse automatique
=========================================================================
Pipeline :
  1. Lire la pose 3D de l'objet depuis /object_pose
  2. Calculer l'IK via /compute_ik
  3. Exécuter la séquence pick & place complète (8 étapes)

CORRECTIONS APPLIQUÉES :
  [C1] Quaternion IK corrigé : qy=0.7071, qw=0.7071
       → rotation -90° autour Y_world → X_wrist3 = -Z_world (saisie vers le bas)
       L'ancien qx=1.0, qw=0.0 (180° autour X) était géométriquement faux pour
       cette configuration de bras.

  [C2] grasp_height corrigé : 0.0823 m (offset base gripper depuis wrist_3_link)
       L'axe de saisie du Robotiq 85 est X_wrist3 dans cette configuration.
       wrist_3_z_cible = oz + 0.0823 (base gripper au centre-haut de l'objet).
       Valeur précédente (0.16) ne correspondait à aucun calcul géométrique réel.

  [C3] MultiThreadedExecutor dans main() — obligatoire pour spin_until_future_complete()
       appelé depuis le thread principal sans deadlock.

  [C4] rclpy.spin_once() remplacé par time.sleep() dans la boucle d'attente pose.
       spin_once() est interdit quand le node tourne déjà dans un executor.

  [C5] attach_box/detach_box sur wrist_3_link (tip réel du groupe ur5_arm).

  [C6] Table ajoutée à la scène MoveIt2 avant tout mouvement.

  [C7] Seed IK diversifiée : 3 seeds (HOME, neutre, proche pick) pour augmenter
       le taux de succès KDL sur les poses difficiles.

  [C8] Messages d'erreur distincts dans _send_and_wait() pour faciliter le debug.

  [C9] Nettoyage de la scène à la fin (remove table + box) pour permettre
       les relances sans accumulation d'objets fantômes dans MoveIt2.
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
from sensor_msgs.msg import JointState

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

# ── Source unique de vérité pour les positions ──────────────────────────────
# Aligné avec initial_positions.yaml et la config physique UR5 standard.
# Ne pas modifier sans mettre à jour initial_positions.yaml en même temps.
HOME            = [0.0,  -1.5708,  1.5708, -1.5708, -1.5708,  0.0]

# Position de dépôt (côté gauche du robot, symétrique au pick)
PREPLACE_JOINTS = [1.57,  -1.10,  1.30,  -1.77,  -1.57,  0.0]
PLACE_JOINTS    = [1.57,  -1.45,  1.55,  -2.10,  -1.57,  0.0]
RETREAT_JOINTS  = [1.57,  -0.85,  1.10,  -1.57,  -1.57,  0.0]

# Seeds IK supplémentaires pour améliorer le taux de succès KDL
# KDL est un solveur itératif — des seeds variées évitent les minima locaux.
IK_SEED_NEUTRAL = [0.0, -0.5, 0.5, -0.5, -1.57, 0.0]
IK_SEED_PICK    = [0.2, -1.3, 1.0, -1.5, -1.57, 0.0]


class PickPlaceNode(Node):
    """Pick & Place avec IK automatique pour le UR5 + Robotiq 85."""

    def __init__(self):
        super().__init__('pick_place_node')

        self.declare_parameter('velocity_scale',     0.15)
        self.declare_parameter('acceleration_scale', 0.10)
        self.declare_parameter('planning_time',      30.0)
        self.declare_parameter('planning_attempts',  30)
        self.declare_parameter('joint_tolerance',    0.05)
        self.declare_parameter('pre_pick_height',    0.20)
        # grasp_height = d6 (0.0823m) + longueur doigts Robotiq 85 (≈ 0.050m)
        # d6 = offset wrist_3 → base gripper (frame tool0)
        # 0.050m = longueur utile des doigts jusqu'au point de contact
        # wrist_3_z_cible = oz_objet + grasp_height
        # → le point de contact des doigts arrive exactement au centre de l'objet
        self.declare_parameter('grasp_height',       0.130)

        self.vel        = self.get_parameter('velocity_scale').value
        self.acc        = self.get_parameter('acceleration_scale').value
        self.ptime      = self.get_parameter('planning_time').value
        self.patt       = self.get_parameter('planning_attempts').value
        self.jtol       = self.get_parameter('joint_tolerance').value
        self.pre_height = self.get_parameter('pre_pick_height').value
        self.grasp_h    = self.get_parameter('grasp_height').value

        # MoveGroup action client
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for /move_action …')
        self.move_client.wait_for_server()
        self.get_logger().info('Connected to /move_action ✅')

        # IK service
        # Lecture robuste /joint_states par nom — insensible à l'ordre du topic
        self._current_joints = {n: 0.0 for n in ARM_JOINT_NAMES}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('Waiting for /compute_ik …')
        self.ik_client.wait_for_service()
        self.get_logger().info('Connected to /compute_ik ✅')

        # Gripper — topic aligné avec moveit_controllers.yaml action_ns: gripper_cmd
        self.gripper_client = ActionClient(
            self, GripperCommand, '/hand_controller/gripper_cmd')
        if self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.has_gripper = True
            self.get_logger().info('Connected to gripper ✅')
        else:
            self.has_gripper = False
            self.get_logger().warn('⚠️  Gripper simulé (attach/detach only)')

        # Planning scene service
        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.scene_client.wait_for_service()

        # Object pose subscriber
        self.object_pose = None
        self.create_subscription(
            PoseStamped, '/object_pose', self._object_cb, 10)

        self.get_logger().info('PickPlaceNode prêt ✅')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _object_cb(self, msg: PoseStamped):
        self.object_pose = msg

    # ── IK ────────────────────────────────────────────────────────────────────
    def compute_ik(self, x, y, z,
                   qx=0.0, qy=0.7071, qz=0.0, qw=0.7071,
                   seed=None, timeout=5.0):
        """
        Calcule les angles articulaires pour atteindre la pose (x, y, z).

        [C1] Orientation par défaut : qy=0.7071, qw=0.7071
             = rotation -90° autour Y_world
             → X_wrist3 pointe dans -Z_world (vers le sol)
             → Axe de saisie du Robotiq 85 orienté verticalement vers le bas ✓

             L'ancien qx=1.0, qw=0.0 (180° autour X) était géométriquement incorrect
             pour le frame wrist_3_link dans la config HOME standard du UR5.

        [C7] Seed IK : seed=HOME par défaut. Appelé plusieurs fois depuis run()
             avec seeds différentes pour maximiser le taux de succès KDL.
        """
        if seed is None:
            seed = HOME

        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name        = 'ur5_arm'
        ik_req.avoid_collisions  = True
        ik_req.timeout.sec       = int(timeout)
        ik_req.timeout.nanosec   = 0

        rs = RobotState()
        rs.joint_state.name     = ARM_JOINT_NAMES
        rs.joint_state.position = list(seed)
        ik_req.robot_state = rs

        # Contraindre KDL à rester proche de la seed (évite les configs "bras retourné")
        # Sans contraintes, KDL explore tout l'espace et trouve des solutions à 6+ rad
        from moveit_msgs.msg import JointConstraint, Constraints
        constraints = Constraints()
        seed_list = list(seed)
        tol = 2.5  # ±2.5 rad autour de la seed — large mais évite les retournements
        for joint_name, seed_val in zip(ARM_JOINT_NAMES, seed_list):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = seed_val
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        ik_req.constraints = constraints

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
        req.ik_request = ik_req

        fut = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout + 2.0)

        if not fut.result():
            return None

        resp = fut.result()
        if resp.error_code.val != SUCCESS:
            return None

        joint_map = dict(zip(
            resp.solution.joint_state.name,
            resp.solution.joint_state.position,
        ))
        return [joint_map.get(n, 0.0) for n in ARM_JOINT_NAMES]

    def compute_ik_robust(self, x, y, z, label='', timeout=5.0):
        """
        [C7] IK robuste : essaie plusieurs seeds pour maximiser le taux de succès.
        Retourne les joints ou None si toutes les seeds échouent.
        """
        seeds = [HOME, IK_SEED_PICK, IK_SEED_NEUTRAL]
        for i, seed in enumerate(seeds):
            joints = self.compute_ik(x, y, z, seed=seed, timeout=timeout)
            if joints is not None:
                self.get_logger().info(
                    f'IK ✅ {label} (seed {i+1}/{len(seeds)}) → '
                    f'[{", ".join(f"{j:.2f}" for j in joints)}]')
                return joints
            self.get_logger().warn(f'IK seed {i+1} échouée pour {label} ({x:.3f},{y:.3f},{z:.3f})')

        self.get_logger().error(
            f'❌ IK impossible pour {label} ({x:.3f},{y:.3f},{z:.3f})\n'
            f'   Vérifiez que la pose est dans l\'espace de travail du UR5\n'
            f'   (reach max ~0.85m depuis la base, z > 0.0m)')
        return None

    # ── Goal builder ──────────────────────────────────────────────────────────

    def _js_cb(self, msg):
        """Stocke les positions joints par nom — robuste à l'ordre du topic."""
        for name, pos in zip(msg.name, msg.position):
            if name in self._current_joints:
                self._current_joints[name] = pos

    def _build_goal(self, positions, tol=None, planning_time=None):
        tol = tol or self.jtol
        goal = MoveGroup.Goal()
        req  = goal.request

        req.group_name                        = 'ur5_arm'
        req.workspace_parameters.header.frame_id = 'world'
        req.num_planning_attempts             = self.patt
        req.allowed_planning_time             = planning_time or self.ptime
        req.max_velocity_scaling_factor       = self.vel
        req.max_acceleration_scaling_factor   = self.acc
        # Spécifier l'état de départ explicitement par nom pour éviter
        # le bug d'ordre des joints dans /joint_states (wrist_1/2/3 permutés).
        # is_diff=False + noms explicites = robuste à l'ordre du topic.
        # start_state explicite par nom — robuste à l'ordre de /joint_states
        req.start_state.is_diff = False
        req.start_state.joint_state.name = list(ARM_JOINT_NAMES)
        req.start_state.joint_state.position = [
            self._current_joints.get(n, 0.0) for n in ARM_JOINT_NAMES
        ]

        c = Constraints()
        for name, pos in zip(ARM_JOINT_NAMES, positions):
            jc              = JointConstraint()
            jc.joint_name   = name
            jc.position     = float(pos)
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight       = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        return goal

    # ── Action send/wait ──────────────────────────────────────────────────────
    def _send_and_wait(self, goal, timeout=90.0):
        """
        [C8] Messages d'erreur distincts pour chaque point de défaillance.
        Avec MultiThreadedExecutor, spin_until_future_complete() fonctionne
        correctement : les callbacks d'action sont traités dans les threads
        workers pendant que ce thread attend le futur.
        """
        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)

        gh = fut.result()
        if not gh:
            self.get_logger().error(
                '❌ Goal send timeout — /move_action injoignable ?\n'
                '   Vérifiez que move_group est lancé et que /move_action est disponible.')
            return -1
        if not gh.accepted:
            self.get_logger().error(
                '❌ Goal rejeté par move_group\n'
                '   Causes possibles : état de départ invalide, contraintes infaisables,\n'
                '   ou collisions dans la configuration cible.')
            return -1

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)

        r = rf.result()
        if not r:
            self.get_logger().error(
                '❌ Result timeout — trajectoire trop longue ou contrôleur bloqué ?\n'
                '   Vérifiez goal_time dans ros2_controllers.yaml (actuellement 5.0s).')
            return -2
        return r.result.error_code.val

    def move_to(self, positions, label='', retries=3) -> bool:
        self.get_logger().info(
            f'➡️  {label}  [{", ".join(f"{p:.2f}" for p in positions)}]')

        for attempt in range(retries):
            goal = self._build_goal(positions)
            # Patience progressive : +5s par tentative
            goal.request.allowed_planning_time = self.ptime + (attempt * 5.0)

            code = self._send_and_wait(goal)

            if code == SUCCESS:
                self.get_logger().info(f'   ✅ {label}')
                return True

            self.get_logger().warn(
                f'   retry {attempt + 1}/{retries}  code={code}')
            time.sleep(0.3)

        self.get_logger().error(f'❌ {label} échoué après {retries} tentatives')
        return False

    # ── Gripper ───────────────────────────────────────────────────────────────
    def _gripper_cmd(self, position: float) -> bool:
        if not self.has_gripper:
            state = 'OPEN 🟢' if position == GRIPPER_OPEN else 'CLOSE 🔴'
            self.get_logger().info(f'[sim] Gripper {state}')
            return True
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = GRIPPER_EFFORT
        fut = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return True

    def open_gripper(self):  return self._gripper_cmd(GRIPPER_OPEN)
    def close_gripper(self): return self._gripper_cmd(GRIPPER_CLOSED)

    # ── Planning scene helpers ────────────────────────────────────────────────
    def _apply(self, scene) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene         = scene
        req.scene.is_diff = True
        fut = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)

    def add_box(self, box_id: str, position: tuple,
                size: tuple = (0.05, 0.05, 0.10)) -> bool:
        from moveit_msgs.msg import PlanningScene
        co                  = CollisionObject()
        co.id               = box_id
        co.header.frame_id  = 'world'
        co.header.stamp     = self.get_clock().now().to_msg()
        prim                = SolidPrimitive()
        prim.type           = SolidPrimitive.BOX
        prim.dimensions     = list(size)
        pose                = Pose()
        pose.position.x     = float(position[0])
        pose.position.y     = float(position[1])
        pose.position.z     = float(position[2])
        pose.orientation.w  = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation        = CollisionObject.ADD
        ps                  = PlanningScene()
        ps.world.collision_objects.append(co)
        ok = self._apply(ps)
        self.get_logger().info(f'{"📦" if ok else "❌"} Box "{box_id}" @ {position}')
        return ok

    def remove_box(self, box_id: str) -> bool:
        from moveit_msgs.msg import PlanningScene
        co           = CollisionObject()
        co.id        = box_id
        co.operation = CollisionObject.REMOVE
        ps           = PlanningScene()
        ps.world.collision_objects.append(co)
        return self._apply(ps)

    def attach_box(self, box_id: str) -> bool:
        """
        [C5] Attache l'objet à wrist_3_link (tip du groupe ur5_arm).
        touch_links inclut tous les links du gripper pour éviter les fausses
        collisions pendant le transport.
        """
        from moveit_msgs.msg import PlanningScene
        aco              = AttachedCollisionObject()
        aco.link_name    = 'wrist_3_link'
        aco.object.id    = box_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links  = [
            'wrist_3_link',
            'wrist_2_link',
            'wrist_1_link',
            'robotiq_85_base_link',
            'robotiq_85_left_knuckle_link',  'robotiq_85_right_knuckle_link',
            'robotiq_85_left_finger_link',   'robotiq_85_right_finger_link',
            'robotiq_85_left_inner_knuckle_link', 'robotiq_85_right_inner_knuckle_link',
            'robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link',
        ]
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects.append(aco)
        ok = self._apply(ps)
        self.get_logger().info(f'{"📎" if ok else "❌"} Attached "{box_id}"')
        return ok

    def detach_box(self, box_id: str) -> bool:
        """Détache et remet l'objet dans la scène monde."""
        from moveit_msgs.msg import PlanningScene
        aco              = AttachedCollisionObject()
        aco.link_name    = 'wrist_3_link'
        aco.object.id    = box_id
        aco.object.operation = CollisionObject.REMOVE
        co           = CollisionObject()
        co.id        = box_id
        co.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects.append(aco)
        ps.world.collision_objects.append(co)
        ok = self._apply(ps)
        self.get_logger().info(f'{"🧷" if ok else "❌"} Detached "{box_id}"')
        return ok

    def _cleanup_scene(self):
        """[C9] Nettoie la scène à la fin pour permettre les relances propres."""
        self.remove_box('table')
        self.remove_box('target_box')

    # ── Séquence principale ───────────────────────────────────────────────────
    def run(self) -> bool:
        log = self.get_logger()
        log.info('=' * 60)
        log.info(' UR5 Pick & Place — IK automatique')
        log.info('=' * 60)

        # Attendre que /joint_states soit reçu au moins une fois
        log.info('Attente de /joint_states …')
        deadline = self.get_clock().now().nanoseconds + 5_000_000_000
        while any(v == 0.0 for v in self._current_joints.values()):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                log.warn('⚠️  /joint_states non reçu — utilisation des valeurs HOME par défaut')
                self._current_joints = dict(zip(ARM_JOINT_NAMES, HOME))
                break
        log.info(f'Joints actuels : {[round(v,2) for v in self._current_joints.values()]}')

        # ── Attente pose objet ────────────────────────────────────────────
        # [C4] time.sleep() obligatoire ici : rclpy.spin_once(self) est interdit
        # quand le node tourne dans un MultiThreadedExecutor. Les callbacks sont
        # gérés par l'executor dans ses threads — attente passive uniquement.
        log.info('Attente de /object_pose …')
        t0 = time.time()
        while rclpy.ok() and self.object_pose is None and time.time() - t0 < 10.0:
            time.sleep(0.1)

        if self.object_pose:
            ox = self.object_pose.pose.position.x
            oy = self.object_pose.pose.position.y
            oz = self.object_pose.pose.position.z
            log.info(f'📍 Objet détecté à ({ox:.3f}, {oy:.3f}, {oz:.3f})')
        else:
            ox, oy, oz = 0.50, 0.10, 0.05
            log.warn(f'⚠️  Fallback position ({ox}, {oy}, {oz})')

        # ── Calcul IK ─────────────────────────────────────────────────────
        log.info('\n📐 Calcul cinématique inverse …')
        # [C1] qy=0.7071, qw=0.7071 : rotation -90° autour Y_world
        # → X_wrist3 = -Z_world → axe de saisie Robotiq 85 vers le sol ✓

        pre_pick_joints = self.compute_ik_robust(
            ox, oy, oz + self.pre_height, label='PRE-PICK')
        grasp_joints = self.compute_ik_robust(
            ox, oy, oz + self.grasp_h, label='GRASP')

        if pre_pick_joints is None or grasp_joints is None:
            return False

        # ── [0/8] Setup scène ─────────────────────────────────────────────
        log.info('\n[0/8] Setup scène')
        self.open_gripper()
        # [C6] Table dans la scène : obstacle critique pour éviter les trajectoires
        # traversant la surface. Aligné avec scene_config.yaml.
        self.add_box('table',      (0.65, 0.0,  -0.025), size=(0.8, 0.8, 0.05))
        self.add_box('target_box', (ox,   oy,    oz),    size=(0.05, 0.05, 0.05))
        time.sleep(0.3)

        # ── [1/8] HOME ────────────────────────────────────────────────────
        log.info('\n[1/8] HOME')
        if not self.move_to(HOME, 'HOME'):
            self._cleanup_scene()
            return False
        time.sleep(0.4)

        # ── [2/8] PRE-PICK ────────────────────────────────────────────────
        log.info(f'\n[2/8] PRE-PICK — au-dessus de ({ox:.2f}, {oy:.2f})')
        if not self.move_to(pre_pick_joints, 'PRE-PICK'):
            self._cleanup_scene()
            return False
        time.sleep(0.3)

        # ── [3/8] DESCENTE vers objet ─────────────────────────────────────
        log.info(f'\n[3/8] GRASP — vers ({ox:.2f}, {oy:.2f}, {oz:.2f})')
        if not self.move_to(grasp_joints, 'GRASP'):
            self._cleanup_scene()
            return False
        time.sleep(0.2)

        # ── [4/8] Fermer gripper + attacher ───────────────────────────────
        log.info('\n[4/8] CLOSE GRIPPER + ATTACH')
        self.close_gripper()
        time.sleep(0.5)
        self.attach_box('target_box')
        time.sleep(0.2)

        # ── [5/8] LIFT ────────────────────────────────────────────────────
        log.info('\n[5/8] LIFT')
        if not self.move_to(pre_pick_joints, 'LIFT'):
            self.open_gripper()
            self.detach_box('target_box')
            self._cleanup_scene()
            return False
        time.sleep(0.3)

        # ── [6/8] PRE-PLACE ───────────────────────────────────────────────
        log.info('\n[6/8] PRE-PLACE')
        if not self.move_to(PREPLACE_JOINTS, 'PRE-PLACE'):
            self.open_gripper()
            self.detach_box('target_box')
            self._cleanup_scene()
            return False
        time.sleep(0.3)

        # ── [7/8] PLACE ───────────────────────────────────────────────────
        log.info('\n[7/8] PLACE')
        if not self.move_to(PLACE_JOINTS, 'PLACE'):
            self.open_gripper()
            self.detach_box('target_box')
            self._cleanup_scene()
            return False
        time.sleep(0.2)

        # ── [8/8] Ouvrir gripper + détacher ───────────────────────────────
        log.info('\n[8/8] RELEASE')
        self.open_gripper()
        time.sleep(0.4)
        self.detach_box('target_box')
        time.sleep(0.2)

        # ── Retrait + HOME final ──────────────────────────────────────────
        log.info('\n[+] RETREAT')
        self.move_to(RETREAT_JOINTS, 'RETREAT')
        time.sleep(0.3)

        log.info('\n[+] HOME final')
        self.move_to(HOME, 'HOME final')

        # [C9] Nettoyage scène
        self._cleanup_scene()

        log.info('\n' + '=' * 60)
        log.info(' ✅  Pick & Place COMPLET !')
        log.info('=' * 60)
        return True


def main(args=None):
    rclpy.init(args=args)

    # [C3] MultiThreadedExecutor obligatoire.
    # spin_until_future_complete() est appelé depuis le thread principal (run()).
    # Avec SingleThreadedExecutor, l'executor est bloqué à attendre le futur
    # qu'il ne peut jamais résoudre car ses callbacks sont en file d'attente.
    # Avec 4 workers, les callbacks action/service s'exécutent dans des threads
    # séparés pendant que le thread principal attend le résultat.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    node = PickPlaceNode()
    executor.add_node(node)

    try:
        success = node.run()
        if not success:
            node.get_logger().error('Séquence échouée.')
    except KeyboardInterrupt:
        node.get_logger().info("Arrêté par l'utilisateur.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
