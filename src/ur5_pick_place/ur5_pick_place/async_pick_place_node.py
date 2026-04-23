# async_pick_place_node.py — Remplacement de pick_place_node.py
#!/usr/bin/env python3
"""
Pick & Place entièrement asynchrone avec machine d'état explicite.
Remplace les time.sleep() par des callbacks ROS2.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from enum import Enum, auto
import asyncio

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes

SUCCESS = MoveItErrorCodes.SUCCESS


class PickPlaceState(Enum):
    IDLE         = auto()
    MOVING_HOME  = auto()
    MOVING_PRE_PICK = auto()
    MOVING_GRASP = auto()
    GRASPING     = auto()
    LIFTING      = auto()
    MOVING_PRE_PLACE = auto()
    PLACING      = auto()
    RELEASING    = auto()
    RETREATING   = auto()
    SUCCESS      = auto()
    FAILED       = auto()


class AsyncPickPlaceNode(Node):
    """
    Architecture event-driven avec machine d'état.
    Plus de time.sleep() — transitions déclenchées par callbacks.
    """
    
    def __init__(self):
        super().__init__('async_pick_place_node')
        
        # CallbackGroup réentrant pour permettre les appels simultanés
        self.cb_group = ReentrantCallbackGroup()
        
        self.state = PickPlaceState.IDLE
        self.state_history = []
        
        self.move_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.cb_group)
        
        # Timer de watchdog: détecte les blocages (timeout 60s par étape)
        self.step_start_time = None
        self.watchdog_timer = self.create_timer(
            1.0, self._watchdog_cb, callback_group=self.cb_group)
        
        self.get_logger().info('AsyncPickPlaceNode prêt')

    def _transition_to(self, new_state: PickPlaceState):
        """Transition d'état avec logging et historique."""
        self.get_logger().info(
            f'État: {self.state.name} → {new_state.name}')
        self.state_history.append(self.state)
        self.state = new_state
        self.step_start_time = self.get_clock().now()

    def _watchdog_cb(self):
        """Détecte si une étape dépasse 60 secondes."""
        if self.step_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds / 1e9
        if elapsed > 60.0 and self.state not in (
                PickPlaceState.IDLE, PickPlaceState.SUCCESS, PickPlaceState.FAILED):
            self.get_logger().error(
                f'WATCHDOG: étape {self.state.name} bloquée depuis {elapsed:.1f}s')
            self._transition_to(PickPlaceState.FAILED)

    async def move_to_async(self, positions, label='', 
                             planning_time=10.0, retries=3) -> bool:
        """
        Mouvement entièrement asynchrone — ne bloque PAS le spin.
        Utilise await pour céder le contrôle à l'event loop.
        """
        ARM_JOINT_NAMES = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        
        for attempt in range(retries):
            goal = MoveGroup.Goal()
            goal.request.group_name = 'ur5_arm'
            goal.request.allowed_planning_time = planning_time + attempt * 5.0
            goal.request.max_velocity_scaling_factor = 0.15
            goal.request.max_acceleration_scaling_factor = 0.10
            goal.request.start_state.is_diff = True
            goal.request.num_planning_attempts = 10
            
            from moveit_msgs.msg import Constraints, JointConstraint
            c = Constraints()
            for name, pos in zip(ARM_JOINT_NAMES, positions):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = float(pos)
                jc.tolerance_above = 0.05
                jc.tolerance_below = 0.05
                jc.weight = 1.0
                c.joint_constraints.append(jc)
            goal.request.goal_constraints = [c]
            
            # Attendre le serveur sans bloquer
            if not await self._wait_for_server_async(timeout=10.0):
                self.get_logger().error('Serveur MoveGroup indisponible')
                return False
            
            # Envoyer le goal de façon async
            goal_handle = await self.move_client.send_goal_async(goal)
            
            if not goal_handle.accepted:
                self.get_logger().warn(f'{label}: goal refusé (tentative {attempt+1})')
                continue
            
            # Attendre le résultat
            result = await goal_handle.get_result_async()
            code = result.result.error_code.val
            
            if code == SUCCESS:
                self.get_logger().info(f'✅ {label}')
                return True
            
            self.get_logger().warn(
                f'{label}: échec code={code} (tentative {attempt+1}/{retries})')
            
            # Pause non-bloquante entre les tentatives
            await asyncio.sleep(0.5)
        
        return False

    async def _wait_for_server_async(self, timeout=10.0) -> bool:
        """Attend le serveur action sans bloquer."""
        import asyncio
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout
        while self.get_clock().now().nanoseconds / 1e9 < deadline:
            if self.move_client.server_is_ready():
                return True
            await asyncio.sleep(0.1)
        return False

    async def run_pick_place_async(self, 
                                    grasp_joints, pre_pick_joints,
                                    place_joints, preplace_joints,
                                    home_joints) -> bool:
        """Séquence complète asynchrone avec gestion d'erreur à chaque étape."""
        
        steps = [
            (PickPlaceState.MOVING_HOME,     home_joints,      'HOME',      1),
            (PickPlaceState.MOVING_PRE_PICK, pre_pick_joints,  'PRE-PICK',  2),
            (PickPlaceState.MOVING_GRASP,    grasp_joints,     'GRASP',     3),
        ]
        
        for state, joints, label, step_num in steps:
            self._transition_to(state)
            success = await self.move_to_async(joints, label)
            if not success:
                self.get_logger().error(f'Échec à l\'étape {step_num}: {label}')
                self._transition_to(PickPlaceState.FAILED)
                return False
            
            # Pause non-bloquante — LE BON WAY
            await asyncio.sleep(0.3)
        
        # Saisie
        self._transition_to(PickPlaceState.GRASPING)
        await self._close_gripper_async()
        
        # Lift + Place + Release
        remaining_steps = [
            (PickPlaceState.LIFTING,          pre_pick_joints,  'LIFT'),
            (PickPlaceState.MOVING_PRE_PLACE, preplace_joints,  'PRE-PLACE'),
            (PickPlaceState.PLACING,          place_joints,     'PLACE'),
        ]
        
        for state, joints, label in remaining_steps:
            self._transition_to(state)
            success = await self.move_to_async(joints, label)
            if not success:
                # Récupération d'erreur: ouvrir le gripper avant de partir
                await self._open_gripper_async()
                self._transition_to(PickPlaceState.FAILED)
                return False
            await asyncio.sleep(0.3)
        
        self._transition_to(PickPlaceState.RELEASING)
        await self._open_gripper_async()
        
        # Retour HOME
        self._transition_to(PickPlaceState.RETREATING)
        await self.move_to_async(home_joints, 'HOME final')
        
        self._transition_to(PickPlaceState.SUCCESS)
        return True

    async def _close_gripper_async(self):
        # Remplacer par l'appel action réel du Robotiq
        self.get_logger().info('Gripper CLOSE')
        await asyncio.sleep(1.5)  # Durée réelle d'actionnement

    async def _open_gripper_async(self):
        self.get_logger().info('Gripper OPEN')
        await asyncio.sleep(1.5)
