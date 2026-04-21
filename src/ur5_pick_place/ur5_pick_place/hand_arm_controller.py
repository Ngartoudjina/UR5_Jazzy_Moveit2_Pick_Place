#!/usr/bin/env python3

"""
================================================================================
hand_arm_controller.py
================================================================================

But du script
--------------
Contrôler le bras UR5 et le gripper Robotiq 2F-85 par les gestes de la main,
en utilisant MediaPipe pour la détection et ros2_control pour l'exécution.

Correspondance gestes → actions
---------------------------------
  ✊  Poing    (0 doigt)  → STOP  (annule toute trajectoire en cours)
  ☝   Index    (1 doigt)  → Pose  HOME  (position neutre sûre)
  ✌   2 doigts            → Pose  PICK  (descente vers objet)
  3 doigts                → Pose  PLACE (dépôt de l'objet)
  4 doigts                → Gripper FERMÉ (saisir)
  🖐   5 doigts            → Gripper OUVERT (relâcher)

Architecture
--------------
  Caméra → MediaPipe → count_fingers() → fingers_to_command()
         → HandArmController (Node ROS2)
              ├── ActionClient  →  /ur5_arm_controller/follow_joint_trajectory
              └── ActionClient  →  /robotiq_gripper_controller/follow_joint_trajectory

IMPORTANT : ce script envoie des trajectoires DIRECTEMENT via ros2_control.
            Il n'utilise PAS MoveIt2.

Dépendances Python
-------------------
    pip install mediapipe opencv-python

Dépendances ROS2 (workspace colcon)
-------------------------------------
    control_msgs
    trajectory_msgs
    builtin_interfaces

================================================================================
IMPORTS
================================================================================
"""

import math
import time
import threading

import cv2
import mediapipe as mp  # noqa

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image


# ==============================================================================
# CONFIGURATION DES SERVEURS D'ACTION
# ==============================================================================

ARM_ACTION_SERVER     = '/ur5_arm_controller/follow_joint_trajectory'

# Le gripper utilise GripperActionController → action type différent
# Son topic est /hand_controller/gripper_cmd
GRIPPER_ACTION_SERVER = '/hand_controller/gripper_cmd'

# Nom du joint du gripper Robotiq 2F-85 (confirmé dans l'URDF)
GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'

# Position ouverte / fermée du gripper (en radians)
# Limite URDF : upper="0.80285"
GRIPPER_OPEN   = 0.0     # 0 rad  = ouvert
GRIPPER_CLOSED = 0.8     # 0.8 rad ≈ fermé


# ==============================================================================
# POSES PRÉDÉFINIES DU BRAS UR5
# ==============================================================================
# Toutes les valeurs sont en RADIANS.
# Ordre des joints (identique au fichier joint_wave_demo.py) :
#
#   [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]

ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

POSES = {
    # Position neutre, bras levé, sûre pour tout démarrage
    'HOME': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],

    # Descente vers l'objet à saisir
    # → bras penché vers l'avant, poignet orienté vers le bas
    'PICK': [0.0, -1.0, 1.4, -1.97, -1.57, 0.0],

    # Position de dépôt de l'objet (décalée latéralement)
    # → bras pivoté de ~90°, puis abaissé
    'PLACE': [1.57, -1.0, 1.4, -1.97, -1.57, 0.0],
}

# Durée en secondes pour atteindre une pose
POSE_DURATION = 3.0


# ==============================================================================
# CONFIGURATION MEDIAPIPE (compatible 0.10.x)
# ==============================================================================

# Dans mediapipe 0.10.x, mp.solutions existe encore via un import explicite
from mediapipe.python.solutions import hands      as _mp_hands_mod
from mediapipe.python.solutions import drawing_utils  as mp_draw
from mediapipe.python.solutions import drawing_styles as mp_styles

mp_hands = _mp_hands_mod

hands = mp_hands.Hands(
    static_image_mode        = False,
    max_num_hands            = 1,
    min_detection_confidence = 0.7,
    min_tracking_confidence  = 0.6,
)

FINGER_NAMES = ["Pouce", "Index", "Majeur", "Annulaire", "Auriculaire"]


# ==============================================================================
# NŒUD ROS2 PRINCIPAL
# ==============================================================================

class HandArmController(Node):
    """
    Nœud ROS2 qui :
      1. Démarre deux ActionClients (bras + gripper)
      2. Expose send_arm_pose()   → envoie le bras vers une pose
      3. Expose send_gripper()    → ouvre ou ferme le gripper
      4. Expose stop()            → annule la trajectoire en cours

    L'envoi des goals est NON-BLOQUANT grâce à send_goal_async() :
    pendant que le bras bouge, la caméra continue à tourner.
    """

    def __init__(self):
        super().__init__('hand_arm_controller')

        # ------------------------------------------------------------------
        # Client d'action : bras UR5
        # ------------------------------------------------------------------
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            ARM_ACTION_SERVER
        )

        # ------------------------------------------------------------------
        # Client d'action : gripper Robotiq 2F-85
        # GripperActionController utilise GripperCommand (pas FollowJointTrajectory)
        # ------------------------------------------------------------------
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            GRIPPER_ACTION_SERVER
        )

        # Handle du dernier goal envoyé (pour pouvoir l'annuler)
        self._current_arm_goal_handle    = None
        self._current_gripper_goal_handle = None

        # Verrou pour éviter les envois simultanés
        self._arm_lock    = threading.Lock()
        self._gripper_lock = threading.Lock()

        self.get_logger().info('Attente des serveurs ros2_control...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Bras ✅  Gripper ✅  — Prêt !')

        # Publisher pour le flux caméra → affiché dans RViz
        self.image_publisher = self.create_publisher(
            Image,
            '/hand_camera/image_raw',
            10
        )

    # --------------------------------------------------------------------------
    # CONSTRUCTION D'UNE TRAJECTOIRE VERS UNE POSE
    # --------------------------------------------------------------------------

    def _build_arm_trajectory(self, target_positions: list, duration: float) -> JointTrajectory:
        """
        Crée une trajectoire simple : un seul point cible.

        Le contrôleur interpole lui-même depuis la position courante
        jusqu'à target_positions.
        """
        traj             = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES

        point           = JointTrajectoryPoint()
        point.positions  = [float(p) for p in target_positions]
        point.velocities = [0.0] * 6   # vitesse nulle à l'arrivée (arrêt propre)

        sec  = int(duration)
        nsec = int((duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points.append(point)
        return traj

    def _build_gripper_trajectory(self, position: float, duration: float) -> JointTrajectory:
        """
        Crée une trajectoire pour le gripper Robotiq 2F-85.

        position : valeur en radians
                   0.0  = ouvert
                   0.8  = fermé (ajuster selon ton modèle)
        """
        traj             = JointTrajectory()
        traj.joint_names = [GRIPPER_JOINT]

        point            = JointTrajectoryPoint()
        point.positions  = [float(position)]
        point.velocities = [0.0]

        sec  = int(duration)
        nsec = int((duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points.append(point)
        return traj

    # --------------------------------------------------------------------------
    # ENVOI D'UNE POSE VERS LE BRAS
    # --------------------------------------------------------------------------

    def send_arm_pose(self, pose_name: str):
        """
        Envoie le bras vers une pose prédéfinie (HOME, PICK, PLACE).

        L'appel est asynchrone : on lance le goal et on retourne
        immédiatement sans bloquer la boucle caméra.
        """
        if pose_name not in POSES:
            self.get_logger().warn(f'Pose inconnue : {pose_name}')
            return

        with self._arm_lock:

            goal           = FollowJointTrajectory.Goal()
            goal.trajectory = self._build_arm_trajectory(
                POSES[pose_name], POSE_DURATION
            )

            self.get_logger().info(f'→ Bras : pose {pose_name}')

            future = self.arm_client.send_goal_async(goal)
            future.add_done_callback(self._arm_goal_response_callback)

    def _arm_goal_response_callback(self, future):
        handle = future.result()
        if handle and handle.accepted:
            self._current_arm_goal_handle = handle
            self.get_logger().info('Trajectoire bras acceptée ✅')
        else:
            self.get_logger().error('Trajectoire bras refusée ❌')

    # --------------------------------------------------------------------------
    # ENVOI D'UNE COMMANDE AU GRIPPER
    # --------------------------------------------------------------------------

    def send_gripper(self, open_gripper: bool):
        """
        Ouvre (open_gripper=True) ou ferme (open_gripper=False) le gripper.

        GripperActionController attend un GripperCommand.Goal avec :
            command.position  = position cible en radians
            command.max_effort = effort max (0.0 = pas de limite)
        """
        position = GRIPPER_OPEN if open_gripper else GRIPPER_CLOSED
        label    = 'OUVERT' if open_gripper else 'FERMÉ'

        with self._gripper_lock:

            goal                    = GripperCommand.Goal()
            goal.command.position   = float(position)
            goal.command.max_effort = 0.0   # laisser le contrôleur gérer

            self.get_logger().info(f'→ Gripper : {label} ({position} rad)')

            future = self.gripper_client.send_goal_async(goal)
            future.add_done_callback(self._gripper_goal_response_callback)

    def _gripper_goal_response_callback(self, future):
        handle = future.result()
        if handle and handle.accepted:
            self._current_gripper_goal_handle = handle
            self.get_logger().info('Commande gripper acceptée ✅')
        else:
            self.get_logger().error('Commande gripper refusée ❌')

    # --------------------------------------------------------------------------
    # STOP : annulation de tout mouvement en cours
    # --------------------------------------------------------------------------

    def stop(self):
        """
        Annule les trajectoires en cours (bras et gripper).
        """
        self.get_logger().info('→ STOP demandé')

        if self._current_arm_goal_handle:
            self._current_arm_goal_handle.cancel_goal_async()
            self._current_arm_goal_handle = None

        if self._current_gripper_goal_handle:
            self._current_gripper_goal_handle.cancel_goal_async()
            self._current_gripper_goal_handle = None


    # --------------------------------------------------------------------------
    # PUBLICATION DU FLUX CAMÉRA VERS RVIZ
    # --------------------------------------------------------------------------

    def publish_image(self, frame):
        """
        Convertit une frame OpenCV (BGR) en sensor_msgs/Image
        et la publie sur /hand_camera/image_raw pour RViz.
        """
        msg                 = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height          = frame.shape[0]
        msg.width           = frame.shape[1]
        msg.encoding        = 'bgr8'
        msg.is_bigendian    = False
        msg.step            = frame.shape[1] * 3
        msg.data            = frame.tobytes()
        self.image_publisher.publish(msg)


# ==============================================================================
# DÉTECTION MEDIAPIPE
# ==============================================================================

def count_fingers(hand_landmarks, handedness: str) -> list:
    """
    Retourne [True/False] × 5 doigts (levé = True).
    Ordre : [Pouce, Index, Majeur, Annulaire, Auriculaire]
    """
    lm       = hand_landmarks.landmark
    tips_ids = [4, 8, 12, 16, 20]
    pip_ids  = [3, 6, 10, 14, 18]
    fingers  = []

    # Pouce : comparaison horizontale (inversée selon la main)
    is_right = (handedness == 'Right')
    if is_right:
        fingers.append(lm[tips_ids[0]].x < lm[pip_ids[0]].x)
    else:
        fingers.append(lm[tips_ids[0]].x > lm[pip_ids[0]].x)

    # Quatre autres doigts : comparaison verticale
    for i in range(1, 5):
        fingers.append(lm[tips_ids[i]].y < lm[pip_ids[i]].y)

    return fingers


# ==============================================================================
# LOGIQUE GESTES → COMMANDES
# ==============================================================================

# Anti-rebond : durée minimale entre deux commandes identiques (secondes)
DEBOUNCE_SEC = 1.5


def fingers_to_command(fingers: list) -> tuple:
    """
    Traduit l'état des 5 doigts en commande pour le robot.

    Retourne : (action_type, action_value, description)

        action_type  = 'arm'      → action_value = nom de pose ('HOME', 'PICK', 'PLACE')
                     = 'gripper'  → action_value = True (ouvert) / False (fermé)
                     = 'stop'     → action_value = None
                     = None       → geste non reconnu, aucune action

    Correspondance :
        0 doigt   → STOP
        1 doigt   → Pose HOME
        2 doigts  → Pose PICK
        3 doigts  → Pose PLACE
        4 doigts  → Gripper FERMÉ
        5 doigts  → Gripper OUVERT
    """
    count = sum(fingers)
    pouce, index, majeur, annulaire, auriculaire = fingers

    if count == 0:
        return ('stop', None, '⛔ STOP')

    elif count == 1 and index:
        return ('arm', 'HOME', '🏠 Pose HOME')

    elif count == 2 and index and majeur:
        return ('arm', 'PICK', '⬇  Pose PICK')

    elif count == 3 and index and majeur and annulaire:
        return ('arm', 'PLACE', '📦 Pose PLACE')

    elif count == 4:
        return ('gripper', False, '✊ Gripper FERMÉ')

    elif count == 5:
        return ('gripper', True, '🖐 Gripper OUVERT')

    else:
        return (None, None, '❓ Geste inconnu')


# ==============================================================================
# AFFICHAGE OPENCV
# ==============================================================================

def draw_ui(frame, fingers: list, command_desc: str, total: int):
    """
    Affiche l'interface sur l'image de la caméra.
    """
    h, w = frame.shape[:2]

    # Fond semi-transparent en haut
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 100), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    # Commande en cours
    cv2.putText(frame, command_desc, (15, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 180), 2)

    # Nombre de doigts
    cv2.putText(frame, f'Doigts : {total}', (15, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 100), 2)

    # État visuel de chaque doigt
    if fingers:
        for j, (name, up) in enumerate(zip(FINGER_NAMES, fingers)):
            color = (0, 255, 120) if up else (80, 80, 80)
            bx, by = 20, 120 + j * 25
            cv2.rectangle(frame, (bx, by - 10), (bx + 12, by + 4),
                          color, -1 if up else 1)
            cv2.putText(frame, name, (bx + 18, by + 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

    # Guide des gestes (coin bas gauche)
    guide = [
        '✊ 0 → STOP',
        '☝  1 → HOME',
        '✌  2 → PICK',
        '3  → PLACE',
        '4  → Grip OFF',
        '🖐 5  → Grip ON',
    ]
    for k, line in enumerate(guide):
        cv2.putText(frame, line, (15, h - 130 + k * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return frame


# ==============================================================================
# PROGRAMME PRINCIPAL
# ==============================================================================

def main():

    # ── Initialisation ROS2 ────────────────────────────────────────────────
    rclpy.init()
    node = HandArmController()

    """
    Le nœud ROS2 tourne dans un thread séparé.

    Pourquoi ?
    Parce que rclpy.spin() est bloquant.
    On veut que la boucle principale reste libre pour lire la caméra
    et afficher l'image à 30 fps.

    Le thread appelle rclpy.spin(node) en continu.
    Cela permet au nœud de traiter les callbacks (réponses des ActionClients)
    sans bloquer la boucle caméra.
    """
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()


    # ── Ouverture de la caméra ─────────────────────────────────────────────
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        node.get_logger().error('Impossible d\'ouvrir la caméra')
        rclpy.shutdown()
        return

    print('\n🤖 Contrôle du bras UR5 par gestes — actif !')
    print('──────────────────────────────────────────────')
    print('  ✊  Poing    → STOP')
    print('  ☝   Index    → Pose HOME')
    print('  ✌   2 doigts → Pose PICK')
    print('      3 doigts → Pose PLACE')
    print('      4 doigts → Gripper FERMÉ')
    print('  🖐   5 doigts → Gripper OUVERT')
    print('  Q / Ctrl+C  → Quitter')
    print('──────────────────────────────────────────────\n')


    # ── Variables d'anti-rebond ────────────────────────────────────────────
    last_command_desc = ''
    last_command_time = 0.0


    # ── Boucle principale ──────────────────────────────────────────────────
    while True:

        ret, frame = cap.read()
        if not ret:
            break

        # Miroir horizontal (plus naturel pour l'utilisateur)
        frame = cv2.flip(frame, 1)

        # Conversion BGR → RGB pour MediaPipe
        rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        fingers      = [False] * 5
        command_desc = '— Aucune main détectée'
        action_type  = None
        action_value = None

        if results.multi_hand_landmarks:

            hand_lms  = results.multi_hand_landmarks[0]
            hand_info = results.multi_handedness[0]
            label     = hand_info.classification[0].label

            # Détection des doigts levés
            fingers = count_fingers(hand_lms, label)

            # Traduction geste → commande
            action_type, action_value, command_desc = fingers_to_command(fingers)

            # Dessin du squelette de la main sur l'image
            mp_draw.draw_landmarks(
                frame, hand_lms, mp_hands.HAND_CONNECTIONS,
                mp_styles.get_default_hand_landmarks_style(),
                mp_styles.get_default_hand_connections_style()
            )

            # ── Anti-rebond ──────────────────────────────────────────────
            # On n'exécute une commande que si elle est différente de la
            # précédente OU si assez de temps s'est écoulé.
            now      = time.time()
            is_new   = (command_desc != last_command_desc)
            is_ready = (now - last_command_time >= DEBOUNCE_SEC)

            if action_type is not None and (is_new or is_ready):

                last_command_desc = command_desc
                last_command_time = now

                if action_type == 'stop':
                    node.stop()

                elif action_type == 'arm':
                    node.send_arm_pose(action_value)

                elif action_type == 'gripper':
                    node.send_gripper(action_value)

        # ── Affichage local (fenêtre OpenCV) ─────────────────────────────
        frame = draw_ui(frame, fingers, command_desc, sum(fingers))
        cv2.imshow('Hand → UR5 Controller', frame)

        # ── Publication vers RViz ─────────────────────────────────────────
        node.publish_image(frame)

        # Quitter avec la touche Q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # ── Nettoyage propre ──────────────────────────────────────────────────
    node.get_logger().info('Arrêt : envoi de STOP au robot...')
    node.stop()

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

    print('\n✅ Programme arrêté proprement.')


# ── Point d'entrée ─────────────────────────────────────────────────────────
if __name__ == '__main__':
    main()
