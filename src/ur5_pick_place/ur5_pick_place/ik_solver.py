#!/usr/bin/env python3
"""
ik_solver.py — Solveur IK analytique pour le UR5
=================================================
Solveur géométrique basé sur les paramètres DH officiels Universal Robots.
Fournit la classe de base UR5IKSolver utilisée par tout le pipeline.
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# --- Paramètres DH officiels UR5 (mètres) ---
d1 = 0.089159
a2 = -0.425
a3 = -0.39225
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class UR5IKSolver:
    """Solveur IK analytique géométrique pour le UR5 + Robotiq 85."""

    def __init__(self, node: Node):
        self.node = node
        self.current_joints = [0.0] * 6
        self.node.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(name in m for name in JOINT_NAMES):
            self.current_joints = [m[n] for n in JOINT_NAMES]

    def solve(self, x, y, z, qx, qy, qz, qw):
        """
        Calcule les angles articulaires pour atteindre la pose (x,y,z) + quaternion.
        Retourne {"joints": [t1..t6]} ou None si aucune solution.
        """
        target_pos = np.array([x, y, z])
        target_rot = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

        # 1. Wrist Center — recule de d6 le long de l'axe Z de l'outil
        wc = target_pos - d6 * target_rot[:, 2]
        xc, yc, zc = wc[0], wc[1], wc[2]

        # 2. Theta1 — deux configurations (gauche / droite)
        r = math.sqrt(xc**2 + yc**2)
        if r < abs(d4):
            return None
        alpha = math.atan2(yc, xc)
        phi = math.acos(d4 / r)
        t1_opts = [alpha + phi + math.pi / 2,
                   alpha - phi + math.pi / 2]

        solutions = []
        for t1 in t1_opts:
            c1, s1 = math.cos(t1), math.sin(t1)
            x_arm = c1 * xc + s1 * yc
            y_arm = zc - d1
            dist_sq = x_arm**2 + y_arm**2

            cos_t3 = (dist_sq - a2**2 - a3**2) / (2 * a2 * a3)
            if abs(cos_t3) > 1.0:
                continue

            # 3. Theta3 — deux configurations (coude haut / bas)
            for t3_sign in [1, -1]:
                t3 = t3_sign * math.acos(cos_t3)
                t2 = (math.atan2(y_arm, x_arm)
                      - math.atan2(a3 * math.sin(t3),
                                   a2 + a3 * math.cos(t3)))

                # 4. R0_3 — rotation de la base jusqu'au joint 3
                r01 = self._dh(0,  d1, math.pi / 2, t1)
                r12 = self._dh(a2, 0,  0,            t2)
                r23 = self._dh(a3, 0,  0,            t3)
                R0_3 = (r01 @ r12 @ r23)[:3, :3]

                # 5. Rotation résiduelle du poignet
                R3_6 = R0_3.T @ target_rot

                # 6. Extraction ZYZ correcte pour le poignet sphérique UR5
                #    R3_6[2,2] = cos(t5)
                t5 = math.acos(np.clip(R3_6[2, 2], -1.0, 1.0))

                if abs(math.sin(t5)) < 1e-6:
                    # Singularité de poignet : t4 et t6 couplés, on fixe t4=0
                    t4 = 0.0
                    t6 = math.atan2(-R3_6[0, 1], R3_6[0, 0])
                else:
                    # t4 : atan2(R3_6[1,2], R3_6[0,2])  = atan2(s4*s5, c4*s5)
                    t4 = math.atan2(R3_6[1, 2],  R3_6[0, 2])
                    # t6 : atan2(R3_6[2,1], -R3_6[2,0]) = atan2(s5*s6, -(-s5*c6))
                    t6 = math.atan2(R3_6[2, 1], -R3_6[2, 0])

                solutions.append([t1, t2, t3, t4, t5, t6])

        if not solutions:
            return None

        # Sélection de la solution la plus proche de la position courante
        best = min(
            solutions,
            key=lambda sol: sum((a - b) ** 2
                                for a, b in zip(sol, self.current_joints))
        )
        return {"joints": best}

    def _dh(self, a, d, alpha, theta):
        """Matrice de transformation homogène DH standard."""
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d     ],
            [0,   0,        0,       1     ],
        ])


# ---------------------------------------------------------------------------
# Extension robuste (multi-seed + détection singularité)
# ---------------------------------------------------------------------------

from typing import Optional, List


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
        # d1, a2, a3, d4, d5, d6 sont définis au niveau module dans ce fichier
        
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
