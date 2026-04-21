#!/usr/bin/env python3
"""UR5 Pick and Place Node v2.1 - Integrated with Analytical IK Solver."""

import sys
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur5_pick_place.base_pick_place import PickPlaceBase
from ur5_pick_place.config_loader import ConfigLoader
from ur5_pick_place.ik_solver import JOINT_NAMES

class PickPlaceNode(PickPlaceBase):
    """Main pick and place execution node using Analytical IK."""

    def __init__(self):
        super().__init__("pick_place_node")
        # Publisher pour envoyer les commandes au contrôleur de trajectoire
        self.joint_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        self.logger.info("PickPlaceNode initialized with Analytical IK Solver")

    def move_to_position(self, pos_config) -> bool:
        """Helper to solve IK and publish trajectory."""
        # Extraction des coordonnées du config_loader
        x = pos_config['x']
        y = pos_config['y']
        z = pos_config['z']
        # Utilisation de l'orientation par défaut (pince vers le bas) 
        # ou celle du YAML si elle existe
        qx = pos_config.get('qx', 0.0)
        qy = pos_config.get('qy', 0.707)
        qz = pos_config.get('qz', 0.0)
        qw = pos_config.get('qw', 0.707)

        # Calcul IK
        ik_res = self.solver.solve(x, y, z, qx, qy, qz, qw)
        
        if not ik_res:
            self.logger.error(f"IK Solver failed for position: {x, y, z}")
            return False

        # Création du message de trajectoire
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = ik_res['joints']
        point.time_from_start.sec = 2  # Temps de déplacement (ajustable)
        
        msg.points.append(point)
        self.joint_pub.publish(msg)
        
        # Petit délai pour laisser le robot bouger (ou attendre le feedback des joint_states)
        self.logger.info(f"Moving to: x={x}, y={y}, z={z}")
        rclpy.spin_once(self, timeout_sec=2.5) 
        return True

    def run_sequence(self) -> bool:
        """Execute 8-stage pick and place sequence."""
        # Le reste de ta logique run_sequence demeure IDENTIQUE 
        # car elle appelle self.move_to_position
        try:
            # Stage 0: Setup
            if not self.execute_stage("[0/8] SETUP", self._stage_setup).success: return False

            # Stage 1: Home
            if not self.execute_stage("[1/8] HOME", self._stage_move_home).success: return False

            # Stage 2: Pre-pick
            if not self.execute_stage("[2/8] PRE_PICK", self._stage_move_pre_pick).success: return False

            # Stage 3: Grasp
            if not self.execute_stage("[3/8] GRASP", self._stage_move_grasp).success: return False

            # Stage 4: Close
            if not self.execute_stage("[4/8] CLOSE", self._stage_close_gripper).success: return False

            # Stage 5: Lift
            if not self.execute_stage("[5/8] LIFT", self._stage_move_lift).success: return False

            # Stage 6: Pre-place
            if not self.execute_stage("[6/8] PRE_PLACE", self._stage_move_pre_place).success: return False

            # Stage 7: Place
            if not self.execute_stage("[7/8] PLACE", self._stage_move_place).success: return False

            # Stage 8: Release
            return self.execute_stage("[8/8] RELEASE", self._stage_release_object).success

        except Exception as e:
            self.logger.error(f"Exception in run_sequence: {e}")
            return False

    # --- Les méthodes de stages ( _stage_... ) restent les mêmes ---
    def _stage_setup(self): return self.gripper.async_open()
    
    def _stage_move_home(self):
        pos = ConfigLoader.get_position("home")
        return self.move_to_position(pos) if pos else False

    def _stage_move_pre_pick(self):
        pos = ConfigLoader.get_position("pre_pick")
        return self.move_to_position(pos) if pos else False

    def _stage_move_grasp(self):
        pos = ConfigLoader.get_position("grasp")
        return self.move_to_position(pos) if pos else False

    def _stage_close_gripper(self):
        profile = ConfigLoader.get_gripper_profile("medium_grasp")
        return self.gripper.async_grasp(profile) if profile else False

    def _stage_move_lift(self):
        pos = ConfigLoader.get_position("lift")
        return self.move_to_position(pos) if pos else False

    def _stage_move_pre_place(self):
        pos = ConfigLoader.get_position("pre_place")
        return self.move_to_position(pos) if pos else False

    def _stage_move_place(self):
        pos = ConfigLoader.get_position("place")
        return self.move_to_position(pos) if pos else False

    def _stage_release_object(self):
        if not self.gripper.async_open(): return False
        pos = ConfigLoader.get_position("retreat")
        return self.move_to_position(pos) if pos else True

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        # Note: Dans ton code, tu lances node.run(). 
        # Assure-toi que PickPlaceBase.run() appelle run_sequence()
        success = node.run_sequence()
        exit_code = 0 if success else 1
    except KeyboardInterrupt:
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
