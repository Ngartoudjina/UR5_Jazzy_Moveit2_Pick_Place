# ik_solver_robust.py — Extension de ton ik_solver.py existant
#!/usr/bin/env python3
"""
Solveur IK robuste avec :
- Seed aléatoire pour éviter les minima locaux
- Détection de singularité (manipulabilité de Yoshikawa)
- Sélection de la meilleure solution parmi plusieurs
- Contrainte de continuité (minimise le mouvement articulaire)
"""

import math
import numpy as np
from typing import Optional, List
from ur5_pick_place.ik_solver import UR5IKSolver, JOINT_NAMES


class RobustUR5IKSolver(UR5IKSolver):
    
    # Limites articulaires réelles UR5 (hardware)
    JOINT_LIMITS = [
        (-math.pi, math.pi),       # shoulder_pan
        (-math.pi, math.pi),       # shoulder_lift  
        (-2*math.pi, 2*math.pi),   # elbow
        (-math.pi, math.pi),       # wrist_1
        (-math.pi, math.pi),       # wrist_2
        (-2*math.pi, 2*math.pi),   # wrist_3
    ]
    
    # Seuil de manipulabilité en dessous duquel on considère une singularité
    SINGULARITY_THRESHOLD = 0.02
    
    def solve_robust(self, x, y, z, qx, qy, qz, qw,
                     num_seeds: int = 8) -> Optional[List[float]]:
        """
        IK robuste avec plusieurs seeds et filtrage de singularité.
        
        Args:
            num_seeds: Nombre de seeds aléatoires à essayer
        Returns:
            Meilleure solution joints ou None si impossible
        """
        all_solutions = []
        
        # Seed 1: position actuelle (continuité)
        result = self.solve(x, y, z, qx, qy, qz, qw)
        if result:
            all_solutions.append(result['joints'])
        
        # Seeds aléatoires dans les limites articulaires
        for _ in range(num_seeds - 1):
            seed = [
                np.random.uniform(lo, hi)
                for lo, hi in self.JOINT_LIMITS
            ]
            self.current_joints = seed  # Modifier la seed pour le solveur parent
            result = self.solve(x, y, z, qx, qy, qz, qw)
            if result:
                all_solutions.append(result['joints'])
        
        # Remettre la seed à la position actuelle
        self.current_joints = self.current_joints
        
        if not all_solutions:
            return None
        
        # Filtrer les solutions singulières
        valid_solutions = [
            sol for sol in all_solutions
            if self._manipulability(sol) > self.SINGULARITY_THRESHOLD
        ]
        
        if not valid_solutions:
            # Assouplir si aucune solution non-singulière
            valid_solutions = all_solutions
        
        # Choisir la solution qui minimise le déplacement depuis la position actuelle
        best = min(
            valid_solutions,
            key=lambda sol: sum(
                (a - b)**2 for a, b in zip(sol, self.current_joints)
            )
        )
        
        manip = self._manipulability(best)
        if manip < self.SINGULARITY_THRESHOLD:
            import rclpy
            self.node.get_logger().warn(
                f'Solution proche d\'une singularité: manipulabilité={manip:.4f}')
        
        return best

    def _manipulability(self, joints: List[float]) -> float:
        """
        Mesure de manipulabilité de Yoshikawa: det(J @ J^T).
        Proche de 0 = proche d'une singularité.
        """
        J = self._compute_jacobian(joints)
        # Manipulabilité = sqrt(det(J @ J^T))
        JJt = J @ J.T
        det = max(0.0, np.linalg.det(JJt))
        return math.sqrt(det)

    def _compute_jacobian(self, joints: List[float]) -> np.ndarray:
        """
        Jacobien géométrique 6×6 pour le UR5 via DH.
        Retourne seulement les 3 premières lignes (position) pour simplifier.
        """
        # Calcul des matrices de transformation successives
        from ur5_pick_place.ik_solver import d1, a2, a3, d4, d5, d6
        
        t1, t2, t3, t4, t5, t6 = joints
        
        # Positions des origines de chaque repère
        T = np.eye(4)
        origins = [np.array([0.0, 0.0, 0.0])]
        
        dh_params = [
            (0,  d1, math.pi/2, t1),
            (a2, 0,  0,          t2),
            (a3, 0,  0,          t3),
            (0,  d4, math.pi/2,  t4),
            (0,  d5, -math.pi/2, t5),
            (0,  d6, 0,          t6),
        ]
        
        transforms = []
        for a, d, alpha, theta in dh_params:
            Ti = self._dh(a, d, alpha, theta)
            T = T @ Ti
            transforms.append(T.copy())
            origins.append(T[:3, 3].copy())
        
        # Jacobien (3 lignes position seulement — suffisant pour manipulabilité)
        pe = origins[-1]  # position end-effector
        J = np.zeros((3, 6))
        
        T_current = np.eye(4)
        for i, dh in enumerate(dh_params):
            a, d, alpha, theta = dh
            Ti = self._dh(a, d, alpha, theta)
            z_i = T_current[:3, 2]  # axe z du repère i
            o_i = T_current[:3, 3]  # origine du repère i
            J[:, i] = np.cross(z_i, pe - o_i)
            T_current = T_current @ Ti
        
        return J
