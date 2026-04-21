#!/usr/bin/env python3
"""
================================================================================
pick_place_hybrid.py  —  Pipeline 3 : MoveIt2 + ros2_control combinés
================================================================================

Séquence et répartition :
    HOME          → MoveIt2  (planification auto, évitement obstacles)
    Approche      → MoveIt2  (trajectoire sûre calculée)
    Descente PICK → MoveIt2  (précision cartésienne)
    Gripper fermé → ros2_control direct (réponse immédiate, pas de planning)
    Lift          → MoveIt2  (évitement avec objet en main)
    Approche PLACE→ MoveIt2
    Descente PLACE→ MoveIt2
    Gripper ouvert→ ros2_control direct
    Retrait       → MoveIt2
    HOME          → MoveIt2

Avantages de cette approche combinée :
    - MoveIt2 gère la cinématique inverse et l'évitement d'obstacles
    - ros2_control gère le gripper directement (plus rapide, plus fiable)
    - La scène de collision est mise à jour dynamiquement

Prérequis :
    ros2 launch ur5_moveit moveit.launch.py
================================================================================
"""

import os
import yaml
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    JointConstraint, CollisionObject, PlanningScene,
    AttachedCollisionObject
)
from moveit_msgs.srv import ApplyPlanningScene
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


# ==============================================================================
# CHARGEMENT DE LA CONFIG
# ==============================================================================

def load_config(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


# ==============================================================================
# SCENEMANAGER (avec support attach/detach objet)
# ==============================================================================

class SceneManager:

    def __init__(self, node: Node, cfg: dict):
        self.node = node
        self.cfg  = cfg['scene']

        self.scene_client = node.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        while not self.scene_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().info('Attente de /apply_planning_scene...')

    def _apply(self, scene: PlanningScene):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

    def setup(self):
        """Charge table + objet dans la scène."""
        self.node.get_logger().info('Chargement de la scène...')
        scene = PlanningScene()
        scene.is_diff = True

        for key in ('table', 'pick_object'):
            c = self.cfg[key]
            obj = CollisionObject()
            obj.id = c['id']
            obj.header.frame_id = c['frame_id']
            obj.operation = CollisionObject.ADD
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [c['size']['x'], c['size']['y'], c['size']['z']]
            pose = Pose()
            pose.position.x = float(c['position']['x'])
            pose.position.y = float(c['position']['y'])
            pose.position.z = float(c['position']['z'])
            pose.orientation.w = 1.0
            obj.primitives.append(box)
            obj.primitive_poses.append(pose)
            scene.world.collision_objects.append(obj)

        self._apply(scene)
        self.node.get_logger().info('Scène chargée ✅')

    def attach_object(self, obj_id: str, link: str = 'wrist_3_link'):
        """
        Attache l'objet au link du gripper après saisie.
        MoveIt2 prend en compte l'objet comme faisant partie du robot
        → évitement d'obstacles correct pendant le transport.
        """
        scene = PlanningScene()
        scene.is_diff = True

        attached = AttachedCollisionObject()
        attached.link_name = link
        attached.object.id = obj_id
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = [
            'robotiq_85_base_link',
            'robotiq_85_left_finger_link',
            'robotiq_85_left_finger_tip_link',
            'robotiq_85_right_finger_link',
            'robotiq_85_right_finger_tip_link',
        ]
        scene.robot_state.attached_collision_objects.append(attached)
        self._apply(scene)
        self.node.get_logger().info(f'Objet {obj_id} attaché à {link} ✅')

    def detach_object(self, obj_id: str):
        """Détache l'objet après dépôt."""
        scene = PlanningScene()
        scene.is_diff = True
        attached = AttachedCollisionObject()
        attached.object.id = obj_id
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(attached)
        self._apply(scene)
        self.node.get_logger().info(f'Objet {obj_id} détaché ✅')


# ==============================================================================
# NŒUD PRINCIPAL
# ==============================================================================

class PickPlaceHybrid(Node):

    def __init__(self, cfg: dict):
        super().__init__('pick_place_hybrid')
        self.cfg = cfg
        self.pp  = cfg['pick_place']

        # ── Clients d'action ──────────────────────────────────────────────────
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.grip_client = ActionClient(
            self, GripperCommand, '/hand_controller/gripper_cmd'
        )

        # Thread spin pour les callbacks asynchrones
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True
        )
        self._spin_thread.start()

        self.get_logger().info('Attente de MoveIt2...')
        self.move_client.wait_for_server()
        self.grip_client.wait_for_server()
        self.get_logger().info('MoveIt2 + Gripper prêts ✅')

        self.scene = SceneManager(self, cfg)

    # --------------------------------------------------------------------------
    # CONSTRUCTION DES CONTRAINTES MOVEIT2
    # --------------------------------------------------------------------------

    def _pose_goal(self, x, y, z) -> Constraints:
        ori = self.pp['gripper_orientation']
        c   = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = 'wrist_3_link'
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(ori['x'])
        pose.orientation.y = float(ori['y'])
        pose.orientation.z = float(ori['z'])
        pose.orientation.w = float(ori['w'])
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = 'world'
        oc.link_name = 'wrist_3_link'
        oc.orientation.x = float(ori['x'])
        oc.orientation.y = float(ori['y'])
        oc.orientation.z = float(ori['z'])
        oc.orientation.w = float(ori['w'])
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        return c

    def _joint_goal(self) -> Constraints:
        c = Constraints()
        for name, value in self.pp['home_joints'].items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    # --------------------------------------------------------------------------
    # ENVOI MOVEIT2
    # --------------------------------------------------------------------------

    def _move(self, constraints: Constraints, label: str) -> bool:
        self.get_logger().info(f'[MoveIt2] → {label}')

        goal = MoveGroup.Goal()
        goal.request.group_name = self.pp['planning_group']
        goal.request.num_planning_attempts = int(self.pp['planning_attempts'])
        goal.request.allowed_planning_time = float(self.pp['planning_time'])
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        future = self.move_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.02)

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f'{label} refusé ❌')
            return False

        result_future = handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        err = result_future.result().result.error_code.val
        if err == 1:
            self.get_logger().info(f'{label} ✅')
            return True
        else:
            self.get_logger().error(f'{label} échoué (code {err}) ❌')
            return False

    # --------------------------------------------------------------------------
    # GRIPPER VIA ros2_control direct
    # --------------------------------------------------------------------------

    def _gripper(self, open_gripper: bool):
        label    = 'Gripper OUVERT' if open_gripper else 'Gripper FERMÉ'
        position = float(self.pp['gripper_open'] if open_gripper
                         else self.pp['gripper_closed'])

        self.get_logger().info(f'[ros2_control] → {label}')

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 0.0

        future = self.grip_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.01)
        time.sleep(1.0)
        self.get_logger().info(f'{label} ✅')

    # --------------------------------------------------------------------------
    # SÉQUENCE PICK & PLACE HYBRIDE
    # --------------------------------------------------------------------------

    def run(self):
        self.get_logger().info('\n========== DÉMARRAGE PICK & PLACE (Hybride) ==========\n')

        obj    = self.cfg['scene']['pick_object']
        target = self.cfg['scene']['place_target']
        pp     = self.pp
        ox, oy, oz = float(obj['position']['x']), float(obj['position']['y']), float(obj['position']['z'])
        tx, ty, tz = float(target['position']['x']), float(target['position']['y']), float(target['position']['z'])
        ah  = float(pp['approach_height'])
        lh  = float(pp['lift_height'])
        obj_top = oz + float(obj['size']['z']) / 2.0

        # ── 1. Scène ──────────────────────────────────────────────────────────
        self.scene.setup()

        # ── 2. HOME ──────────────────────────────────────────────────────────
        if not self._move(self._joint_goal(), 'HOME'): return
        self._gripper(open_gripper=True)

        # ── 3. Approche PICK (MoveIt2) ────────────────────────────────────────
        if not self._move(self._pose_goal(ox, oy, oz + ah), 'Approche PICK'): return

        # ── 4. Descente PICK (MoveIt2) ────────────────────────────────────────
        if not self._move(self._pose_goal(ox, oy, obj_top + 0.01), 'Descente PICK'): return

        # ── 5. Saisie (ros2_control direct) ──────────────────────────────────
        self._gripper(open_gripper=False)
        self.scene.attach_object(obj['id'])   # objet attaché au robot
        time.sleep(0.5)

        # ── 6. Lift (MoveIt2 — avec objet attaché) ────────────────────────────
        if not self._move(self._pose_goal(ox, oy, oz + lh), 'Lift'): return

        # ── 7. Approche PLACE (MoveIt2) ───────────────────────────────────────
        if not self._move(self._pose_goal(tx, ty, tz + ah), 'Approche PLACE'): return

        # ── 8. Descente PLACE (MoveIt2) ───────────────────────────────────────
        if not self._move(self._pose_goal(tx, ty, tz + 0.01), 'Descente PLACE'): return

        # ── 9. Dépôt (ros2_control direct) ───────────────────────────────────
        self._gripper(open_gripper=True)
        self.scene.detach_object(obj['id'])   # objet détaché du robot
        time.sleep(0.5)

        # ── 10. Retrait (MoveIt2) ─────────────────────────────────────────────
        if not self._move(self._pose_goal(tx, ty, tz + ah), 'Retrait'): return

        # ── 11. HOME (MoveIt2) ────────────────────────────────────────────────
        self._move(self._joint_goal(), 'HOME final')

        self.get_logger().info('\n========== PICK & PLACE TERMINÉ ✅ ==========\n')


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'scene_config.yaml'
    )
    cfg = load_config(config_path)

    rclpy.init()
    node = PickPlaceHybrid(cfg)

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt demandé')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
