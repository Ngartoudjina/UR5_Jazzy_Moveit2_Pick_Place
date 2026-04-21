#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from ur5_pick_place.ik_solver import UR5IKSolver

class TestIK(Node):
    def __init__(self):
        super().__init__("test_ik_node")
        self.solver = UR5IKSolver(self)
        self.get_logger().info("Initialisation du test IK terminée.")

    def run(self):
        # On attend un peu pour recevoir les /joint_states initiaux
        self.get_logger().info("En attente des données du robot...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Liste de tests : [X, Y, Z, Description]
        # On garde une orientation fixe : pince pointant vers le bas (Quaternions standard)
        qx, qy, qz, qw = 0.0, 0.707, 0.0, 0.707
        
        test_points = [
            (0.3, 0.0, 0.2, "Point bas devant"),
            (0.4, 0.1, 0.3, "Point milieu décalé"),
            (0.3, -0.1, 0.4, "Point haut décalé"),
            (0.5, 0.0, 0.1, "Limite d'extension")
        ]

        self.get_logger().info("--- DÉBUT DE LA SÉQUENCE DE TEST ---")
        
        for x, y, z, desc in test_points:
            self.get_logger().info(f"Cible : {desc} [x={x}, y={y}, z={z}]")
            
            # Appel du solveur
            res = self.solver.solve(x, y, z, qx, qy, qz, qw)
            
            if res:
                joints = res['joints']
                # Conversion en degrés pour une lecture humaine facile
                deg_joints = [round(math.degrees(q), 1) for q in joints]
                
                self.get_logger().info(f"  ✅ SUCCÈS")
                self.get_logger().info(f"  Configuration : {deg_joints}")
                self.get_logger().info(f"  Base/Épaule/Coude : {deg_joints[0]}°, {deg_joints[1]}°, {deg_joints[2]}°")
                self.get_logger().info(f"  Poignets (Ori.)  : {deg_joints[3]}°, {deg_joints[4]}°, {deg_joints[5]}°")
            else:
                self.get_logger().error(f"  ❌ ÉCHEC : Point hors d'atteinte ou singularité.")
            
            print("-" * 40)

def main():
    rclpy.init()
    node = TestIK()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
