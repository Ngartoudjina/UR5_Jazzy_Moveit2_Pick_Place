#!/usr/bin/env python3
"""
base_pick_place.py — Classe de base pour le pick & place UR5.

CORRECTIONS APPLIQUÉES :
  - Utilisation d'un ActionClient FollowJointTrajectory (au lieu d'un publisher topic)
  - Attente réelle de la fin de trajectoire via get_result_async()
  - Durée de trajectoire calculée dynamiquement selon l'amplitude angulaire
  - Nom de contrôleur aligné avec ros2_controllers.yaml : /ur5_arm_controller
"""

import math
import time
from typing import Union, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration as RosDuration

from ur5_pick_place.config_loader import ConfigLoader, JointPosition
from ur5_pick_place.validators import JointValidator, ValidationResult

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]

# Vitesse angulaire maximale de sécurité (rad/s) — 50% de la limite UR5
MAX_JOINT_VELOCITY = 1.0  # rad/s


def compute_trajectory_duration(current_angles, target_angles, max_vel=MAX_JOINT_VELOCITY) -> float:
    """Calcule la durée minimale de trajectoire selon l'amplitude max."""
    if not current_angles:
        return 3.0
    max_delta = max(abs(t - c) for t, c in zip(target_angles, current_angles))
    duration = max_delta / max_vel
    return max(duration, 1.5)  # minimum 1.5 secondes


class BasePickPlace(Node):
    """
    Classe de base pour orchestrer les stages du pick & place.

    Utilise un ActionClient FollowJointTrajectory pour envoyer les trajectoires
    avec confirmation de fin d'exécution (pas de publisher topic aveugle).
    """

    # Nom du contrôleur tel que défini dans ros2_controllers.yaml
    ARM_CONTROLLER_ACTION = '/ur5_arm_controller/follow_joint_trajectory'

    def __init__(self, node_name: str = 'base_pick_place'):
        super().__init__(node_name)

        # --- ActionClient bras (corrigé : plus de publisher topic) ---
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.ARM_CONTROLLER_ACTION
        )

        # État courant des joints (mis à jour via /joint_states si besoin)
        self._current_joints = [0.0] * 6

        # Chargement configuration
        try:
            ConfigLoader.load_from_file('config/ur5_pick_place_config.yaml')
            self.config = ConfigLoader()
        except Exception as e:
            self.get_logger().warn(f"Config non chargée: {e} — utilisation des valeurs par défaut")
            self.config = None

        self.get_logger().info(f"[BasePickPlace] Initialisé — ActionClient: {self.ARM_CONTROLLER_ACTION}")

    # ──────────────────────────────────────────────────────────────────────────
    #  Interface publique
    # ──────────────────────────────────────────────────────────────────────────

    def move_to_position(
        self,
        position: Union[JointPosition, Dict, list],
        retries: int = 3,
        timeout_per_try: float = 30.0
    ) -> bool:
        """
        Déplace le bras vers une position articulaire.

        Attend la confirmation de fin de trajectoire avant de retourner.

        Args:
            position: JointPosition, liste de 6 angles (rad), ou dict cartésien
            retries: Nombre de tentatives en cas d'échec
            timeout_per_try: Timeout (s) par tentative

        Returns:
            True si le mouvement s'est terminé avec succès
        """
        # Résolution de la position en liste d'angles
        angles = self._resolve_angles(position)
        if angles is None:
            self.get_logger().error("Impossible de résoudre les angles pour la position donnée")
            return False

        # Validation des angles
        validation: ValidationResult = JointValidator.validate_angles(angles)
        if not validation.valid:
            self.get_logger().error(f"Angles invalides : {validation.message}")
            return False

        # Calcul de la durée dynamique
        duration_sec = compute_trajectory_duration(self._current_joints, angles)

        pos_name = getattr(position, 'name', 'position') if not isinstance(position, (list, dict)) else 'position'

        for attempt in range(1, retries + 1):
            self.get_logger().info(
                f"[{pos_name}] Tentative {attempt}/{retries} — durée prévue: {duration_sec:.1f}s"
            )

            success = self._send_trajectory_and_wait(angles, duration_sec, timeout_per_try)
            if success:
                self._current_joints = angles.copy()
                self.get_logger().info(f"✅ [{pos_name}] Mouvement terminé avec succès")
                return True
            else:
                self.get_logger().warn(f"⚠️  [{pos_name}] Tentative {attempt} échouée")
                time.sleep(0.5)

        self.get_logger().error(f"❌ [{pos_name}] Échec après {retries} tentatives")
        return False

    def wait_for_arm_controller(self, timeout_sec: float = 10.0) -> bool:
        """Attend que le serveur d'action du bras soit disponible."""
        self.get_logger().info("En attente du contrôleur bras...")
        ready = self._arm_client.wait_for_server(timeout_sec=timeout_sec)
        if ready:
            self.get_logger().info("✅ Contrôleur bras disponible")
        else:
            self.get_logger().error(f"❌ Contrôleur bras non disponible après {timeout_sec}s")
        return ready

    # ──────────────────────────────────────────────────────────────────────────
    #  Méthodes internes
    # ──────────────────────────────────────────────────────────────────────────

    def _send_trajectory_and_wait(
        self,
        angles: list,
        duration_sec: float,
        timeout_sec: float
    ) -> bool:
        """
        Envoie une trajectoire via ActionClient et ATTEND la fin réelle.

        Contrairement à l'ancienne implémentation (publisher topic + spin_once),
        cette méthode utilise le mécanisme action ROS2 pour obtenir un résultat.
        """
        # Construction du message de trajectoire
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in angles]
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6

        secs = int(duration_sec)
        nanosecs = int((duration_sec - secs) * 1e9)
        point.time_from_start = RosDuration(sec=secs, nanosec=nanosecs)
        trajectory.points = [point]

        # Construction du goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Envoi du goal
        if not self._arm_client.server_is_ready():
            self.get_logger().error("ActionServer non disponible !")
            return False

        send_future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error("Timeout lors de l'envoi du goal")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejeté par le contrôleur !")
            return False

        # Attente du résultat avec timeout dynamique
        result_future = goal_handle.get_result_async()
        actual_timeout = duration_sec + 5.0  # marge de 5s
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=actual_timeout)

        if not result_future.done():
            self.get_logger().error(f"Timeout d'exécution après {actual_timeout:.1f}s")
            return False

        result = result_future.result()
        if result is None:
            self.get_logger().error("Résultat nul reçu du contrôleur")
            return False

        error_code = result.result.error_code
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return True
        else:
            self.get_logger().error(f"Erreur contrôleur : error_code={error_code}")
            return False

    def _resolve_angles(self, position) -> Optional[list]:
        """Convertit position (JointPosition, list, dict) en liste de 6 angles."""
        if isinstance(position, list):
            if len(position) == 6:
                return [float(a) for a in position]
            self.get_logger().error(f"Liste doit avoir 6 éléments, reçu {len(position)}")
            return None

        if isinstance(position, JointPosition):
            return [float(a) for a in position.angles]

        if isinstance(position, dict) and 'joints' in position:
            return [float(a) for a in position['joints']]

        if isinstance(position, dict) and 'x' in position:
            # Cartésien → IK (à implémenter dans les sous-classes)
            self.get_logger().warn("IK cartésien non implémenté dans BasePickPlace")
            return None

        self.get_logger().error(f"Type de position non supporté : {type(position)}")
        return None
