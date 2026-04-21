#!/usr/bin/env python3
import math
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# --- Paramètres URDF/DH fixes ---
d1 = 0.089159
a2 = -0.425
a3 = -0.39225
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823 

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

class UR5IKSolver:
    def __init__(self, node: Node):
        self.node = node
        self.current_joints = [0.0]*6
        self.node.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(name in m for name in JOINT_NAMES):
            self.current_joints = [m[n] for n in JOINT_NAMES]

    def solve(self, x, y, z, qx, qy, qz, qw):
        target_pos = np.array([x, y, z])
        target_rot = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

        # 1. Wrist Center (Calculé sur l'axe d'approche Z)
        wc = target_pos - d6 * target_rot[:, 2]
        xc, yc, zc = wc[0], wc[1], wc[2]

        # 2. Theta 1
        r = math.sqrt(xc**2 + yc**2)
        if r < abs(d4): return None
        alpha = math.atan2(yc, xc)
        phi = math.acos(d4 / r)
        t1_opts = [alpha + phi + math.pi/2, alpha - phi + math.pi/2]

        solutions = []
        for t1 in t1_opts:
            c1, s1 = math.cos(t1), math.sin(t1)
            x_arm = c1 * xc + s1 * yc
            y_arm = zc - d1
            dist_sq = x_arm**2 + y_arm**2
            
            cos_t3 = (dist_sq - a2**2 - a3**2) / (2 * a2 * a3)
            if abs(cos_t3) > 1.0: continue
            
            for t3_sign in [1, -1]:
                t3 = t3_sign * math.acos(cos_t3)
                t2 = math.atan2(y_arm, x_arm) - math.atan2(a3 * math.sin(t3), a2 + a3 * math.cos(t3))

                # --- Calcul des poignets (Orientation) ---
                # Rotation de la base jusqu'au bras (R0_3)
                r01 = self._dh(0, d1, math.pi/2, t1)
                r12 = self._dh(a2, 0, 0, t2)
                r23 = self._dh(a3, 0, 0, t3)
                R0_3 = (r01 @ r12 @ r23)[:3, :3]
                
                # Rotation résiduelle pour les 3 derniers joints
                R3_6 = R0_3.T @ target_rot
                
                # Extraction angles d'Euler pour t4, t5, t6
                try:
                    t5 = math.acos(np.clip(R3_6[1, 2], -1, 1))
                    if abs(math.sin(t5)) < 1e-6:
                        t4 = 0.0
                        t6 = math.atan2(R3_6[0, 1], R3_6[0, 0])
                    else:
                        t4 = math.atan2(R3_6[2, 2], -R3_6[0, 2])
                        t6 = math.atan2(-R3_6[1, 1], R3_6[1, 0])
                    
                    solutions.append([t1, t2, t3, t4, t5, t6])
                except: continue

        if not solutions: return None
        
        # Sélection de la solution la plus fluide
        best_sol = min(solutions, key=lambda sol: sum((np.array(sol) - self.current_joints)**2))
        return {"joints": best_sol}

    def _dh(self, a, d, alpha, theta):
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array([[ct, -st*ca, st*sa, a*ct],
                         [st, ct*ca, -ct*sa, a*st],
                         [0, sa, ca, d],
                         [0, 0, 0, 1]])
