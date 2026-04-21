#!/usr/bin/env python3
"""
================================================================================
pick_place_ros2control.py  —  Pipeline 2 : ros2_control direct
================================================================================

Séquence :
    HOME → approche → pick (gripper ferme) → lift → place → HOME

Ce script envoie les trajectoires DIRECTEMENT via ros2_control,
sans passer par MoveIt2. Les poses sont définies en angles de joints
(pas en coordonnées cartésiennes).

Prérequis :
    ros2 launch ur5_controller ur5_rviz_control.launch.py

Config :
    Toutes les poses viennent de scene_config.yaml (section pick_place.home_joints)
    Les poses PICK/PLACE sont hardcodées ici en angles de joints.

Corrections apportées :
    - max_effort corrigé pour le gripper fermé (était 0.0 → le gripper ne serrait rien)
    - return False sur erreur dans _move() (était True → séquence continuait après échec)
    - Pose HOME corrigée (pose "ready" du SRDF, sans collision)
    - Résultat gripper attendu via get_result_async() (était juste time.sleep(1.0))
    - import concurrent.futures inutilisé supprimé
    - Timeout ajouté sur toutes les attentes de futures
    - Boucles while not future.done() remplacées par threading.Event
    - Sleeps liés à move_duration pour cohérence
================================================================================
"""

import os
import sys
import yaml
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ==============================================================================
# UTILITAIRE: Chargement de configuration
# ==============================================================================

def load_config(path: str) -> dict:
    """Load YAML configuration file with error handling."""
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config file not found: {path}")
    
    try:
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        if not config:
            raise ValueError("Empty configuration file")
        return config
    except yaml.YAMLError as e:
        raise RuntimeError(f"YAML parsing error: {e}")
    except Exception as e:
        raise RuntimeError(f"Failed to load config: {e}")


# ==============================================================================
# CONSTANTES ET VALEURS PAR DÉFAUT
# ==============================================================================

FUTURE_TIMEOUT_SEC   = 15.0   # timeout global pour les futures
GRIPPER_OPEN_EFFORT  = 0.0    # pas d'effort pour ouvrir
GRIPPER_CLOSE_EFFORT = 50.0   # ✅ effort non nul pour saisir l'objet
SERVER_WAIT_TIMEOUT  = 10.0   # timeout pour wait_for_server()

# Valeurs par défaut si config est incomplete
DEFAULT_ARM_ACTION     = '/ur5_arm_controller/follow_joint_trajectory'
DEFAULT_GRIPPER_ACTION = '/hand_controller/gripper_cmd'
DEFAULT_MOVE_DURATION  = 3.0

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# POSES par défaut (fallback si pas dans YAML)
# ✅ OPTIMISÉ POUR DESCENDRE VRAIMENT SUR L'OBJET À (0.5, 0.1, 0.05)
DEFAULT_POSES = {
    'HOME':           [0.0,     -0.1041,  1.4232,  0.243,   4.6863,  1.6315],
    'APPROCHE_PICK':  [0.2,     -1.3,     1.0,    -1.5,    -1.57,    0.0],      # Au-dessus, mais plus bas
    'PICK':           [0.2,     -1.45,    1.15,   -1.75,   -1.57,    0.0],      # DESCEND SUR L'OBJET
    'LIFT':           [0.2,     -1.2,     1.0,    -1.5,    -1.57,    0.0],      # Soulève
    'APPROCHE_PLACE': [-0.6,    -1.3,     1.0,    -1.5,    -1.57,    0.0],      # Vers zone place
    'PLACE':          [-0.6,    -1.45,    1.15,   -1.75,   -1.57,    0.0],      # Pose l'objet
    'RETRAIT':        [-0.6,    -1.2,     1.0,    -1.5,    -1.57,    0.0],      # Dégagement
}

# ==============================================================================
# HELPER : attente d'une future avec timeout via threading.Event
# ==============================================================================

def wait_for_future(future, timeout_sec: float = FUTURE_TIMEOUT_SEC) -> bool:
    """
    Attend qu'une future soit complète sans boucle active (CPU-friendly).
    Retourne True si terminée dans le délai, False si timeout.

    ✅ CORRECTION : remplace les boucles 'while not future.done(): time.sleep(0.01)'
    qui consomment du CPU inutilement et ne gèrent pas les timeouts.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())

    # Si la future est déjà terminée avant l'ajout du callback
    if future.done():
        return True

    completed = event.wait(timeout=timeout_sec)
    return completed


# ==============================================================================
# NŒUD PRINCIPAL
# ==============================================================================

class PickPlaceRos2Control(Node):

    def __init__(self, cfg: dict):
        super().__init__('pick_place_ros2control')
        self.cfg      = cfg
        
        # Load pick_place config with error handling
        if 'pick_place' not in cfg:
            raise ValueError("Missing 'pick_place' section in config")
        
        self.pp = cfg['pick_place']
        
        # Load move_duration with fallback
        try:
            self.duration = float(self.pp.get('move_duration', DEFAULT_MOVE_DURATION))
        except (ValueError, TypeError) as e:
            self.get_logger().warning(f"Invalid move_duration: {e}, using default {DEFAULT_MOVE_DURATION}")
            self.duration = DEFAULT_MOVE_DURATION
        
        # Load action server names with fallbacks
        self.arm_action = self.pp.get('arm_action', DEFAULT_ARM_ACTION)
        self.gripper_action = self.pp.get('gripper_action', DEFAULT_GRIPPER_ACTION)
        
        # Load POSES from config, with fallback to defaults
        try:
            self.poses = {}
            # Try to load all required poses from YAML
            required_poses = ['HOME', 'APPROCHE_PICK', 'PICK', 'LIFT', 'APPROCHE_PLACE', 'PLACE', 'RETRAIT']
            
            poses_in_config = self.pp.get('poses', {})
            if poses_in_config:
                # Load from YAML
                for pose_name in required_poses:
                    if pose_name in poses_in_config:
                        self.poses[pose_name] = poses_in_config[pose_name]
                    else:
                        # Use default for this pose
                        self.poses[pose_name] = DEFAULT_POSES[pose_name]
                self.get_logger().info(f"Loaded poses from config (mixed with defaults)")
            else:
                # Use all defaults
                self.poses = DEFAULT_POSES.copy()
                self.get_logger().info(f"Using {len(DEFAULT_POSES)} default poses")
        except Exception as e:
            self.get_logger().warning(f"Error loading poses: {e}, using all defaults")
            self.poses = DEFAULT_POSES.copy()
        
        # Validate poses
        self._validate_poses()
        
        # Create action clients
        self.arm_client  = ActionClient(self, FollowJointTrajectory, self.arm_action)
        self.grip_client = ActionClient(self, GripperCommand, self.gripper_action)

        # Thread dédié au spin ROS2 (permet les callbacks non bloquants)
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True
        )
        self._spin_thread.start()

        # Wait for controller servers with timeout
        self.get_logger().info(f'Attente des contrôleurs ros2_control (timeout={SERVER_WAIT_TIMEOUT}s)...')
        try:
            if not self.arm_client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT):
                raise RuntimeError(f"Arm controller not available at {self.arm_action}")
            if not self.grip_client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT):
                raise RuntimeError(f"Gripper controller not available at {self.gripper_action}")
        except Exception as e:
            self.get_logger().error(f"Controller initialization failed: {e}")
            raise
        
        self.get_logger().info('Contrôleurs prêts ✅')

    def _validate_poses(self):
        """Validate that all required poses exist and have correct structure."""
        required_poses = ['HOME', 'APPROCHE_PICK', 'PICK', 'LIFT', 'APPROCHE_PLACE', 'PLACE', 'RETRAIT']
        
        for pose_name in required_poses:
            if pose_name not in self.poses:
                raise ValueError(f"Missing required pose: {pose_name}")
            
            pose_angles = self.poses[pose_name]
            if not isinstance(pose_angles, (list, tuple)) or len(pose_angles) != 6:
                raise ValueError(f"Invalid pose {pose_name}: expected 6 angles, got {len(pose_angles) if isinstance(pose_angles, (list, tuple)) else '?'}")
        
        self.get_logger().info(f"✓ All {len(required_poses)} required poses validated")

    # --------------------------------------------------------------------------
    # CONSTRUCTION D'UNE TRAJECTOIRE
    # --------------------------------------------------------------------------

    def _build_traj(self, positions: list, duration: float) -> JointTrajectory:
        traj             = JointTrajectory()
        traj.joint_names = ARM_JOINTS

        pt             = JointTrajectoryPoint()
        pt.positions   = [float(p) for p in positions]
        pt.velocities  = [0.0] * len(ARM_JOINTS)
        sec            = int(duration)
        nsec           = int((duration - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points.append(pt)
        return traj

    # --------------------------------------------------------------------------
    # ENVOI D'UNE POSE
    # --------------------------------------------------------------------------

    def _move(self, pose_name: str) -> bool:
        self.get_logger().info(f'→ {pose_name}')
        
        # Validate pose exists
        if pose_name not in self.poses:
            self.get_logger().error(f"Unknown pose: {pose_name}")
            return False

        goal            = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_traj(self.poses[pose_name], self.duration)

        future = self.arm_client.send_goal_async(goal)

        # ✅ Attente de l'acceptation avec timeout
        if not wait_for_future(future):
            self.get_logger().error(f'{pose_name} : timeout en attente d\'acceptation ❌')
            return False

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f'{pose_name} refusé ❌')
            return False

        result_future = handle.get_result_async()

        # ✅ Timeout = durée du mouvement + marge de sécurité
        timeout = self.duration + 5.0
        if not wait_for_future(result_future, timeout_sec=timeout):
            self.get_logger().error(f'{pose_name} : timeout en attente du résultat ❌')
            return False

        code = result_future.result().result.error_code
        if code == 0:
            self.get_logger().info(f'{pose_name} ✅')
            return True
        else:
            # ✅ CORRECTION : return False sur erreur (était True → séquence continuait)
            self.get_logger().error(f'{pose_name} échoué (code={code}) ❌')
            return False

    # --------------------------------------------------------------------------
    # COMMANDE GRIPPER
    # --------------------------------------------------------------------------

    def _gripper(self, open_gripper: bool) -> bool:
        """
        Commande le gripper et attend la fin de l'exécution.

        ✅ max_effort non nul à la fermeture (sinon aucune saisie).
        ✅ résultat attendu via get_result_async() (pas time.sleep).
        ✅ Error handling robuste avec defaults.
        """
        label    = 'Gripper OUVERT' if open_gripper else 'Gripper FERMÉ'
        
        # Load gripper positions with error handling
        try:
            if open_gripper:
                position = float(self.pp.get('gripper_open', 0.0))
            else:
                position = float(self.pp.get('gripper_closed', 0.7))
        except (ValueError, TypeError) as e:
            self.get_logger().warning(f"Invalid gripper position in config: {e}, using default")
            position = 0.0 if open_gripper else 0.7
        
        effort   = GRIPPER_OPEN_EFFORT if open_gripper else GRIPPER_CLOSE_EFFORT

        self.get_logger().info(f'→ {label}')

        goal                    = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = effort   # ✅

        future = self.grip_client.send_goal_async(goal)

        if not wait_for_future(future):
            self.get_logger().error(f'{label} : timeout acceptation ❌')
            return False

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f'{label} refusé ❌')
            return False

        # ✅ CORRECTION : attente du résultat réel (était juste time.sleep(1.0))
        result_future = handle.get_result_async()
        if not wait_for_future(result_future, timeout_sec=5.0):
            self.get_logger().error(f'{label} : timeout résultat ❌')
            return False

        self.get_logger().info(f'{label} ✅')
        return True

    # --------------------------------------------------------------------------
    # PAUSE COHÉRENTE AVEC move_duration
    # --------------------------------------------------------------------------

    def _pause(self, factor: float = 0.15):
        """
        ✅ CORRECTION : pause proportionnelle à move_duration au lieu de
        valeurs hardcodées (time.sleep(0.5), time.sleep(0.3), etc.).
        Permet de changer move_duration dans le YAML sans risque de chevauchement.
        """
        time.sleep(self.duration * factor)

    # --------------------------------------------------------------------------
    # SÉQUENCE PICK & PLACE
    # --------------------------------------------------------------------------

    def run(self):
        self.get_logger().info(
            '\n========== DÉMARRAGE PICK & PLACE (ros2_control) ==========\n'
        )

        # 1. HOME + gripper ouvert
        if not self._move('HOME'):          return
        self._gripper(open_gripper=True)
        self._pause(0.2)

        # 2. Approche au-dessus de l'objet
        if not self._move('APPROCHE_PICK'): return
        self._pause(0.1)

        # 3. Descente vers l'objet
        if not self._move('PICK'):          return
        self._pause(0.2)

        # 4. Fermer le gripper
        if not self._gripper(open_gripper=False): return
        self._pause(0.2)

        # 5. Lift
        if not self._move('LIFT'):          return
        self._pause(0.1)

        # 6. Approche zone de dépôt
        if not self._move('APPROCHE_PLACE'): return
        self._pause(0.1)

        # 7. Descente vers zone de dépôt
        if not self._move('PLACE'):         return
        self._pause(0.2)

        # 8. Ouvrir le gripper
        if not self._gripper(open_gripper=True): return
        self._pause(0.2)

        # 9. Retrait
        if not self._move('RETRAIT'):       return
        self._pause(0.1)

        # 10. HOME final
        self._move('HOME')

        self.get_logger().info(
            '\n========== PICK & PLACE TERMINÉ ✅ ==========\n'
        )


# ==============================================================================
# MAIN
# ==============================================================================

def main(args=None):
    # Try multiple locations for scene_config.yaml
    possible_paths = [
        # 1. Same directory as this script (development mode)
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scene_config.yaml'),
        
        # 2. Installed via colcon (pip install location)
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'share', 
                    'ur5_pick_place', 'scene_config.yaml'),
        
        # 3. ROS2 install directory
        os.path.expandvars('$ROS_INSTALL_DIR/share/ur5_pick_place/scene_config.yaml'),
        os.path.expandvars('$COLCON_PREFIX_PATH/share/ur5_pick_place/scene_config.yaml'),
        
        # 4. Workspace paths
        os.path.expandvars('~/ros2_humble_ws/src/ur5_pick_place/ur5_pick_place/scene_config.yaml'),
        os.path.expandvars('~/ros2_humble_ws/install/ur5_pick_place/share/ur5_pick_place/scene_config.yaml'),
        
        # 5. Current working directory
        'scene_config.yaml',
        os.path.join(os.getcwd(), 'ur5_pick_place', 'scene_config.yaml'),
        os.path.join(os.getcwd(), 'src', 'ur5_pick_place', 'ur5_pick_place', 'scene_config.yaml'),
        
        # 6. Docker environment
        '/ros2_ws/src/ur5_pick_place/ur5_pick_place/scene_config.yaml',
        '/ros2_ws/install/ur5_pick_place/share/ur5_pick_place/scene_config.yaml',
    ]
    
    config_path = None
    for path in possible_paths:
        expanded_path = os.path.expanduser(path)
        if os.path.exists(expanded_path):
            config_path = expanded_path
            print(f"✓ Configuration file found at: {config_path}")
            break
    
    if config_path is None:
        print(f"✗ FATAL: Failed to find scene_config.yaml")
        print(f"\nSearched in these locations:")
        for path in possible_paths:
            print(f"  ✗ {os.path.expanduser(path)}")
        print(f"\nHints:")
        print(f"  1. Make sure scene_config.yaml exists in ur5_pick_place/ directory")
        print(f"  2. Run 'colcon build' and 'source install/setup.bash'")
        print(f"  3. Check your ROS2 installation with 'ros2 pkg list | grep ur5'")
        return 1
    
    # Load and validate configuration
    try:
        cfg = load_config(config_path)
        print(f"✓ Configuration loaded successfully")
    except Exception as e:
        print(f"✗ FATAL: Failed to load config from {config_path}: {e}")
        return 1

    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        node = PickPlaceRos2Control(cfg)
        print("✓ Pick Place Node initialized")
    except Exception as e:
        print(f"✗ FATAL: Failed to initialize node: {e}")
        rclpy.shutdown()
        return 1

    try:
        node.run()
        return_code = 0
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt demandé par utilisateur')
        return_code = 0
    except Exception as e:
        node.get_logger().error(f'Erreur non gérée: {e}')
        return_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return return_code


if __name__ == '__main__':
    main()
