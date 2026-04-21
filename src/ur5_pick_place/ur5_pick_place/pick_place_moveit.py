#!/usr/bin/env python3
"""
================================================================================
pick_place_moveit.py  —  Pipeline 1 : MoveIt2 pur
================================================================================

Séquence :
    HOME → approche → pick (gripper ferme) → lift → place → HOME

Ce script utilise UNIQUEMENT MoveIt2 via /move_action pour planifier et
exécuter toutes les poses du bras. Le gripper est également commandé via
MoveIt2 (GripperCommand action via moveit_controllers.yaml).

Prérequis :
    ros2 launch ur5_moveit moveit.launch.py

Config :
    Toutes les positions viennent de scene_config.yaml
================================================================================
"""

import os
import yaml
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    CollisionObject, PlanningScene
)
from moveit_msgs.srv import ApplyPlanningScene
from control_msgs.action import GripperCommand

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
from builtin_interfaces.msg import Duration


# ==============================================================================
# CHARGEMENT DE LA CONFIG
# ==============================================================================

def load_config(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


# ==============================================================================
# CLASSE SCENEMANAGER
# ==============================================================================

class SceneManager:
    """
    Gère l'ajout des objets de collision dans la scène MoveIt2
    à partir de scene_config.yaml.
    """

    def __init__(self, node: Node, cfg: dict):
        self.node = node
        self.cfg  = cfg['scene']

        self.scene_client = node.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )
        while not self.scene_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().info('Attente de /apply_planning_scene...')

    def _make_box(self, obj_id, frame_id, size, position) -> CollisionObject:
        obj            = CollisionObject()
        obj.id         = obj_id
        obj.header.frame_id = frame_id
        obj.operation  = CollisionObject.ADD

        box            = SolidPrimitive()
        box.type       = SolidPrimitive.BOX
        box.dimensions = [size['x'], size['y'], size['z']]

        pose           = Pose()
        pose.position.x = float(position['x'])
        pose.position.y = float(position['y'])
        pose.position.z = float(position['z'])
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        return obj

    def setup(self):
        """Charge la table et l'objet dans la scène MoveIt2."""
        self.node.get_logger().info('Chargement de la scène...')

        scene       = PlanningScene()
        scene.is_diff = True

        table_cfg   = self.cfg['table']
        obj_cfg     = self.cfg['pick_object']

        scene.world.collision_objects.append(
            self._make_box(
                table_cfg['id'],
                table_cfg['frame_id'],
                table_cfg['size'],
                table_cfg['position']
            )
        )
        scene.world.collision_objects.append(
            self._make_box(
                obj_cfg['id'],
                obj_cfg['frame_id'],
                obj_cfg['size'],
                obj_cfg['position']
            )
        )

        req       = ApplyPlanningScene.Request()
        req.scene = scene
        future    = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info('Scène chargée ✅')

    def remove_object(self, obj_id: str):
        """Retire un objet de la scène (après saisie)."""
        scene = PlanningScene()
        scene.is_diff = True
        obj = CollisionObject()
        obj.id = obj_id
        obj.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)


# ==============================================================================
# CLASSE PRINCIPALE
# ==============================================================================

class PickPlaceMoveIt(Node):

    def __init__(self, cfg: dict):
        super().__init__('pick_place_moveit')
        self.cfg = cfg
        self.pp  = cfg['pick_place']

        # ── Clients d'action ──────────────────────────────────────────────────
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.grip_client = ActionClient(
            self, GripperCommand, '/hand_controller/gripper_cmd'
        )

        self.get_logger().info('Attente de MoveIt2 (/move_action)...')
        self.move_client.wait_for_server()
        self.get_logger().info('MoveIt2 prêt ✅')

        # ── SceneManager ──────────────────────────────────────────────────────
        self.scene = SceneManager(self, cfg)

    # --------------------------------------------------------------------------
    # HELPERS : construction des contraintes
    # --------------------------------------------------------------------------

    def _pose_goal(self, x, y, z) -> Constraints:
        """Contrainte de pose cartésienne pour le wrist_3_link."""
        ori = self.pp['gripper_orientation']
        c   = Constraints()

        # Position
        pc              = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name    = 'wrist_3_link'
        sphere          = SolidPrimitive()
        sphere.type     = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pose            = Pose()
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

        # Orientation
        oc              = OrientationConstraint()
        oc.header.frame_id = 'world'
        oc.link_name    = 'wrist_3_link'
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
        """Contrainte de joints pour la pose HOME."""
        from moveit_msgs.msg import JointConstraint
        c      = Constraints()
        joints = self.pp['home_joints']
        for name, value in joints.items():
            jc              = JointConstraint()
            jc.joint_name   = name
            jc.position     = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight       = 1.0
            c.joint_constraints.append(jc)
        return c

    # --------------------------------------------------------------------------
    # ENVOI D'UN GOAL MOVEIT2
    # --------------------------------------------------------------------------

    def _move(self, constraints: Constraints, label: str) -> bool:
        self.get_logger().info(f'→ {label}')

        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        goal = MoveGroup.Goal()
        goal.request.group_name            = self.pp['planning_group']
        goal.request.num_planning_attempts = int(self.pp['planning_attempts'])
        goal.request.allowed_planning_time = float(self.pp['planning_time'])
        goal.request.goal_constraints.append(constraints)
        goal.request.max_velocity_scaling_factor     = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.planning_options.plan_only              = False
        goal.planning_options.replan                 = True
        goal.planning_options.replan_attempts        = 3

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle or not handle.accepted:
            self.get_logger().error(f'Goal refusé : {label} ❌')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        err = result_future.result().result.error_code.val

        if err == 1:
            self.get_logger().info(f'{label} ✅')
            return True
        else:
            self.get_logger().error(f'{label} échoué (code {err}) ❌')
            return False

    # --------------------------------------------------------------------------
    # COMMANDE GRIPPER
    # --------------------------------------------------------------------------

    def _gripper(self, open_gripper: bool):
        label    = 'Gripper OUVERT' if open_gripper else 'Gripper FERMÉ'
        position = float(self.pp['gripper_open'] if open_gripper
                         else self.pp['gripper_closed'])

        self.get_logger().info(f'→ {label}')
        self.grip_client.wait_for_server()

        goal                  = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 0.0

        future = self.grip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)
        self.get_logger().info(f'{label} ✅')

    # --------------------------------------------------------------------------
    # SÉQUENCE PICK & PLACE
    # --------------------------------------------------------------------------

    def run(self):
        self.get_logger().info('\n========== DÉMARRAGE PICK & PLACE (MoveIt2) ==========\n')

        obj     = self.cfg['scene']['pick_object']
        target  = self.cfg['scene']['place_target']
        pp      = self.pp
        ox, oy, oz = float(obj['position']['x']), float(obj['position']['y']), float(obj['position']['z'])
        tx, ty, tz = float(target['position']['x']), float(target['position']['y']), float(target['position']['z'])
        ah      = float(pp['approach_height'])
        lh      = float(pp['lift_height'])

        # ── 1. Charger la scène ───────────────────────────────────────────────
        self.scene.setup()
        time.sleep(2.0)   # laisser MoveIt2 mettre à jour son planning scene

        # ── 2. HOME ───────────────────────────────────────────────────────────
        if not self._move(self._joint_goal(), 'HOME initial'): return
        self._gripper(open_gripper=True)

        # ── 3. Approche (au-dessus de l'objet) ────────────────────────────────
        if not self._move(self._pose_goal(ox, oy, oz + ah), 'Approche PICK'): return

        # ── 4. Descente vers l'objet ──────────────────────────────────────────
        obj_top = oz + float(obj['size']['z']) / 2.0
        if not self._move(self._pose_goal(ox, oy, obj_top + 0.01), 'Descente PICK'): return

        # ── 5. Fermer le gripper ──────────────────────────────────────────────
        self._gripper(open_gripper=False)
        self.scene.remove_object(obj['id'])  # l'objet est maintenant "dans la main"
        time.sleep(0.5)

        # ── 6. Lift ───────────────────────────────────────────────────────────
        if not self._move(self._pose_goal(ox, oy, oz + lh), 'Lift'): return

        # ── 7. Approche zone de dépôt ─────────────────────────────────────────
        if not self._move(self._pose_goal(tx, ty, tz + ah), 'Approche PLACE'): return

        # ── 8. Descente vers zone de dépôt ────────────────────────────────────
        if not self._move(self._pose_goal(tx, ty, tz + 0.01), 'Descente PLACE'): return

        # ── 9. Ouvrir le gripper ──────────────────────────────────────────────
        self._gripper(open_gripper=True)
        time.sleep(0.5)

        # ── 10. Retrait ───────────────────────────────────────────────────────
        if not self._move(self._pose_goal(tx, ty, tz + ah), 'Retrait PLACE'): return

        # ── 11. HOME ──────────────────────────────────────────────────────────
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
    node = PickPlaceMoveIt(cfg)

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt demandé')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
