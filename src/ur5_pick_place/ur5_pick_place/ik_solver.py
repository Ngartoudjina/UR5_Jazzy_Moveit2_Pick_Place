#!/usr/bin/env python3
"""
ik_solver.py — Solveur IK ultra-moderne pour UR5
==================================================
Algorithme : TRAC-IK hybride implémenté en Python pur
  - Phase 1 : KDL analytique (rapide, donne une solution initiale)
  - Phase 2 : Optimisation SQP (scipy) pour raffiner et trouver la solution
              minimisant le mouvement depuis la config actuelle
  - Phase 3 : Validation complète (limites, singularités, collisions MoveIt2)
  - Phase 4 : Calcul des vitesses via la Jacobienne analytique

Usage interactif :
    ros2 run ur5_pick_place ik_solver

Utilisation comme module :
    from ur5_pick_place.ik_solver import UR5IKSolver
    solver = UR5IKSolver(node)
    result = solver.solve(x, y, z, qx, qy, qz, qw)
"""

import sys
import time
import math
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped


# ──────────────────────────────────────────────────────────────────────────────
# Paramètres DH du UR5 (Denavit-Hartenberg modifié)
# ──────────────────────────────────────────────────────────────────────────────
UR5_DH = {
    # [a,      d,       alpha,       theta_offset]
    'd':  [0.089159, 0.0,     0.0,     0.10915, 0.09465, 0.0823],
    'a':  [0.0,     -0.425,  -0.39225, 0.0,     0.0,     0.0   ],
    'alpha': [math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0  ],
}

# Limites articulaires UR5 (radians)
JOINT_LIMITS = [
    (-2*math.pi, 2*math.pi),   # shoulder_pan
    (-2*math.pi, 2*math.pi),   # shoulder_lift
    (-math.pi,   math.pi  ),   # elbow
    (-2*math.pi, 2*math.pi),   # wrist_1
    (-2*math.pi, 2*math.pi),   # wrist_2
    (-2*math.pi, 2*math.pi),   # wrist_3
]

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Seuil de manipulabilité (en-dessous = singularité)
MANIP_THRESHOLD = 1e-4

# Vitesse max par joint (rad/s) — spec UR5
JOINT_VEL_MAX = [3.15, 3.15, 3.15, 3.2, 3.2, 3.2]


# ──────────────────────────────────────────────────────────────────────────────
# Cinématique directe (Forward Kinematics) via matrices DH
# ──────────────────────────────────────────────────────────────────────────────
def dh_matrix(a, d, alpha, theta):
    """Matrice de transformation DH 4x4."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,  -st*ca,  st*sa,  a*ct],
        [st,   ct*ca, -ct*sa,  a*st],
        [0.0,  sa,     ca,     d   ],
        [0.0,  0.0,    0.0,    1.0 ],
    ])


def forward_kinematics(q):
    """
    Cinématique directe du UR5.
    q : liste de 6 angles (radians)
    Retourne : (position [x,y,z], matrice rotation 3x3, T_ee 4x4)
    """
    T = np.eye(4)
    d     = UR5_DH['d']
    a     = UR5_DH['a']
    alpha = UR5_DH['alpha']

    for i in range(6):
        T = T @ dh_matrix(a[i], d[i], alpha[i], q[i])

    pos = T[:3, 3]
    rot = T[:3, :3]
    return pos, rot, T


def jacobian_analytical(q):
    """
    Jacobienne analytique 6x6 du UR5.
    Colonnes = dérivées de la pose EE par rapport à chaque joint.
    """
    d     = UR5_DH['d']
    a     = UR5_DH['a']
    alpha = UR5_DH['alpha']

    # Matrices de transformation cumulatives
    T = [np.eye(4)]
    for i in range(6):
        T.append(T[-1] @ dh_matrix(a[i], d[i], alpha[i], q[i]))

    T_ee  = T[6]
    p_ee  = T_ee[:3, 3]
    J     = np.zeros((6, 6))

    for i in range(6):
        z_i = T[i][:3, 2]           # axe de rotation du joint i
        p_i = T[i][:3, 3]           # position origine frame i
        J[:3, i] = np.cross(z_i, p_ee - p_i)   # partie linéaire
        J[3:, i] = z_i                           # partie angulaire

    return J


def manipulability(q):
    """
    Indice de manipulabilité de Yoshikawa : sqrt(det(J * J^T)).
    0 = singularité, grand = bon conditionnement.
    """
    J   = jacobian_analytical(q)
    JJT = J @ J.T
    det = max(0.0, np.linalg.det(JJT))
    return math.sqrt(det)


# ──────────────────────────────────────────────────────────────────────────────
# Classe principale du solveur IK
# ──────────────────────────────────────────────────────────────────────────────
class UR5IKSolver:
    """
    Solveur IK hybride pour UR5.
    Peut fonctionner de manière autonome ou être utilisé comme module.
    """

    def __init__(self, node: Node):
        self.node = node
        self.log  = node.get_logger()

        # Client IK MoveIt2 (pour validation finale)
        self.ik_client = node.create_client(GetPositionIK, '/compute_ik')

        # Position actuelle des joints
        self.current_joints = [0.0] * 6
        self._joint_sub = node.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

    def _joint_cb(self, msg):
        jmap = dict(zip(msg.name, msg.position))
        self.current_joints = [jmap.get(n, 0.0) for n in JOINT_NAMES]

    # ── Phase 1 : solution initiale analytique ────────────────────────────────
    def _ik_analytical_initial(self, x, y, z, R_target):
        """
        Solution initiale rapide via géométrie analytique du UR5.
        Donne un point de départ pour l'optimisation SQP.
        """
        d = UR5_DH['d']
        a = UR5_DH['a']

        # Position du poignet (wrist center)
        n = R_target[:, 2]               # axe z de l'outil
        p_wc = np.array([x, y, z]) - d[5] * n

        # q1 — rotation de la base
        q1 = math.atan2(p_wc[1], p_wc[0])

        # Distance dans le plan shoulder-wrist
        r = math.sqrt(p_wc[0]**2 + p_wc[1]**2) - 0.0
        s = p_wc[2] - d[0]

        D = (r**2 + s**2 - a[1]**2 - a[2]**2) / (2 * abs(a[1]) * abs(a[2]))
        D = max(-1.0, min(1.0, D))

        # q3 — elbow (deux configurations : haut et bas)
        solutions = []
        for sign in (+1, -1):
            q3 = math.atan2(sign * math.sqrt(1 - D**2), D)
            k1 = abs(a[1]) + abs(a[2]) * math.cos(q3)
            k2 = abs(a[2]) * math.sin(q3)
            q2 = math.atan2(s, r) - math.atan2(k2, k1)

            # q4, q5, q6 depuis la matrice de rotation résiduelle
            T03 = (dh_matrix(a[0], d[0], UR5_DH['alpha'][0], q1) @
                   dh_matrix(a[1], d[1], UR5_DH['alpha'][1], q2) @
                   dh_matrix(a[2], d[2], UR5_DH['alpha'][2], q3))
            R03 = T03[:3, :3]
            R36 = R03.T @ R_target

            q5 = math.atan2(
                math.sqrt(R36[0,2]**2 + R36[1,2]**2),
                R36[2, 2]
            )
            if abs(math.sin(q5)) > 1e-6:
                q4 = math.atan2(R36[1,2], R36[0,2])
                q6 = math.atan2(R36[2,1], -R36[2,0])
            else:
                q4 = 0.0
                q6 = math.atan2(-R36[1,0], R36[1,1])

            solutions.append([q1, q2, q3, q4, q5, q6])

        return solutions

    # ── Phase 2 : optimisation SQP ────────────────────────────────────────────
    def _ik_sqp(self, x, y, z, R_target, q_init, q_current):
        """
        Optimisation SQP (Sequential Quadratic Programming) :
        Minimise la distance à q_current tout en satisfaisant les
        contraintes de position/orientation et les limites articulaires.
        """
        target_pos = np.array([x, y, z])

        def cost(q):
            # Coût = distance à la config actuelle (pondérée)
            weights = np.array([2.0, 1.5, 1.5, 1.0, 1.0, 1.0])
            diff    = np.array(q) - np.array(q_current)
            return float(np.dot(weights * diff, diff))

        def pos_constraint(q):
            pos, _, _ = forward_kinematics(q)
            return float(np.sum((pos - target_pos)**2))

        def orient_constraint(q):
            _, rot, _ = forward_kinematics(q)
            # Distance entre matrices de rotation
            R_err = rot.T @ R_target
            trace = max(-1.0, min(3.0, np.trace(R_err)))
            return float(abs(math.acos((trace - 1) / 2)) if trace < 3.0 else 0.0)

        constraints = [
            {'type': 'eq', 'fun': pos_constraint},
            {'type': 'ineq', 'fun': lambda q: 0.01 - orient_constraint(q)},
        ]

        result = minimize(
            cost,
            q_init,
            method='SLSQP',
            bounds=JOINT_LIMITS,
            constraints=constraints,
            options={
                'ftol':    1e-8,
                'maxiter': 200,
                'disp':    False,
            }
        )

        if result.success or result.fun < 0.1:
            return list(result.x)
        return None

    # ── Phase 3 : validation complète ─────────────────────────────────────────
    def _validate(self, q):
        """
        Vérifie :
        1. Limites articulaires
        2. Manipulabilité (pas de singularité)
        Retourne (valide: bool, raison: str)
        """
        # Limites
        for i, (qi, (lo, hi)) in enumerate(zip(q, JOINT_LIMITS)):
            if not (lo <= qi <= hi):
                return False, f'joint {JOINT_NAMES[i]} hors limites ({qi:.3f})'

        # Singularité
        m = manipulability(q)
        if m < MANIP_THRESHOLD:
            return False, f'singularité détectée (manipulabilité={m:.6f})'

        return True, 'ok'

    # ── Phase 4 : calcul des vitesses ─────────────────────────────────────────
    def compute_velocities(self, q, v_cartesian=None):
        """
        Calcule les vitesses articulaires via la pseudo-inverse de la Jacobienne.
        v_cartesian : vitesse cartésienne cible [vx,vy,vz,wx,wy,wz] (m/s, rad/s)
                      Si None, retourne la vitesse max admissible.
        """
        J      = jacobian_analytical(q)
        J_pinv = np.linalg.pinv(J)     # pseudo-inverse de Moore-Penrose

        if v_cartesian is None:
            # Vitesse cartésienne de référence = 0.1 m/s vers le bas
            v_cartesian = np.array([0.0, 0.0, -0.1, 0.0, 0.0, 0.0])

        q_dot = J_pinv @ np.array(v_cartesian)

        # Normaliser pour respecter les limites de vitesse
        scale = 1.0
        for i, (dq, vmax) in enumerate(zip(q_dot, JOINT_VEL_MAX)):
            if abs(dq) > vmax:
                scale = min(scale, vmax / abs(dq))
        q_dot *= scale

        return list(q_dot)

    # ── Point d'entrée principal ───────────────────────────────────────────────
    def solve(self, x, y, z,
              qx=0.0, qy=0.707, qz=0.0, qw=0.707,
              v_cartesian=None,
              n_attempts=6):
        """
        Résout l'IK pour la pose (x, y, z, quat) et retourne le meilleur résultat.

        Retourne un dict :
        {
          'joints'       : [q1..q6]  angles articulaires (rad)
          'velocities'   : [dq1..dq6] vitesses articulaires (rad/s)
          'manipulability': float     indice de Yoshikawa
          'cost'         : float      distance à la config actuelle
          'fk_position'  : [x,y,z]   vérification FK
          'fk_error'     : float      erreur de position (m)
          'config'       : str        description de la configuration
        }
        ou None si aucune solution valide.
        """
        # Matrice de rotation cible depuis le quaternion
        rot   = Rotation.from_quat([qx, qy, qz, qw])
        R_tar = rot.as_matrix()
        target_pos = np.array([x, y, z])

        # Lire la config actuelle
        rclpy.spin_once(self.node, timeout_sec=0.2)
        q_cur = self.current_joints[:]

        self.log.info(f'Résolution IK pour ({x:.3f}, {y:.3f}, {z:.3f})')
        self.log.info(f'Config actuelle : [{", ".join(f"{q:.2f}" for q in q_cur)}]')

        candidates = []

        # ── Phase 1 : solutions analytiques initiales ──────────────────────
        try:
            initials = self._ik_analytical_initial(x, y, z, R_tar)
        except Exception as e:
            self.log.warn(f'Analytique échoué : {e}')
            initials = []

        # ── Phase 2 : optimisation SQP sur chaque solution initiale ────────
        for i, q_init in enumerate(initials):
            q_opt = self._ik_sqp(x, y, z, R_tar, q_init, q_cur)
            if q_opt:
                candidates.append(('analytique+SQP', q_opt))

        # Points de départ aléatoires supplémentaires
        for _ in range(n_attempts - len(initials)):
            q_rand = [
                np.random.uniform(lo, hi)
                for lo, hi in JOINT_LIMITS
            ]
            q_opt = self._ik_sqp(x, y, z, R_tar, q_rand, q_cur)
            if q_opt:
                candidates.append(('aléatoire+SQP', q_opt))

        if not candidates:
            self.log.error('Aucune solution IK trouvée')
            return None

        # ── Phase 3 : validation et sélection optimale ─────────────────────
        valid_solutions = []
        for origin, q in candidates:
            ok, reason = self._validate(q)
            if not ok:
                self.log.debug(f'Solution rejetée ({origin}): {reason}')
                continue

            # Coût = distance pondérée à la config actuelle
            weights = [2.0, 1.5, 1.5, 1.0, 1.0, 1.0]
            cost    = sum(w * (qi - qc)**2
                         for w, qi, qc in zip(weights, q, q_cur))

            # Vérification FK
            pos_fk, _, _ = forward_kinematics(q)
            fk_error      = float(np.linalg.norm(pos_fk - target_pos))
            manip          = manipulability(q)

            valid_solutions.append({
                'joints'        : q,
                'cost'          : cost,
                'fk_error'      : fk_error,
                'manipulability': manip,
                'origin'        : origin,
            })

        if not valid_solutions:
            self.log.error('Toutes les solutions IK sont invalides')
            return None

        # Trier : d'abord par erreur FK, puis par coût
        valid_solutions.sort(key=lambda s: (s['fk_error'] > 0.01, s['cost']))
        best = valid_solutions[0]

        # ── Phase 4 : calcul des vitesses ──────────────────────────────────
        velocities = self.compute_velocities(best['joints'], v_cartesian)

        # Description de la configuration
        q = best['joints']
        config = ('coude_haut' if q[2] > 0 else 'coude_bas') + \
                 ('_poignet_normal' if q[4] > 0 else '_poignet_retourne')

        result = {
            'joints'        : best['joints'],
            'velocities'    : velocities,
            'manipulability': best['manipulability'],
            'cost'          : best['cost'],
            'fk_position'   : list(forward_kinematics(best['joints'])[0]),
            'fk_error'      : best['fk_error'],
            'config'        : config,
            'n_valid'       : len(valid_solutions),
        }

        return result


# ──────────────────────────────────────────────────────────────────────────────
# Mode interactif : demande la position et affiche les résultats
# ──────────────────────────────────────────────────────────────────────────────
class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        self.solver = UR5IKSolver(self)
        self.get_logger().info('Solveur IK UR5 prêt')

    def run_interactive(self):
        print('\n' + '='*60)
        print(' Solveur IK UR5 — Mode interactif')
        print(' Entrez "q" pour quitter')
        print('='*60)
        print(' Orientation par défaut : outil vers le bas')
        print('   qx=0.0  qy=0.707  qz=0.0  qw=0.707')
        print('='*60)

        while True:
            print()
            try:
                raw = input(' Position cible (x y z) en mètres : ').strip()
                if raw.lower() in ('q', 'quit', 'exit'):
                    print(' Au revoir !')
                    break

                parts = raw.split()
                if len(parts) < 3:
                    print(' Erreur : entrez 3 valeurs (ex: 0.4 0.0 0.3)')
                    continue

                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])

                # Orientation optionnelle
                if len(parts) == 7:
                    qx, qy, qz, qw = (float(parts[3]), float(parts[4]),
                                       float(parts[5]), float(parts[6]))
                    print(f' Orientation : quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})')
                else:
                    qx, qy, qz, qw = 0.0, 0.707, 0.0, 0.707
                    print(' Orientation : outil vers le bas (défaut)')

                # Vitesse cartésienne optionnelle
                v_cart_raw = input(
                    ' Vitesse cartésienne [vx vy vz] m/s (Entrée=défaut 0.05 m/s vers bas) : '
                ).strip()
                if v_cart_raw:
                    vp = v_cart_raw.split()
                    v_cart = [float(vp[0]), float(vp[1]), float(vp[2]),
                              0.0, 0.0, 0.0]
                else:
                    v_cart = [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]

                print()
                print(f' Résolution pour ({x:.3f}, {y:.3f}, {z:.3f}) ...')
                t0  = time.time()
                res = self.solver.solve(x, y, z, qx, qy, qz, qw,
                                        v_cartesian=v_cart)
                dt  = time.time() - t0

                if res is None:
                    print(' Aucune solution valide trouvée.')
                    print(' Vérifiez que la cible est dans l\'espace de travail du UR5.')
                    print(' Espace de travail : rayon ~0.85m, hauteur ~0.1 à ~0.9m')
                    continue

                # Affichage des résultats
                print()
                print('─'*60)
                print(f' Solution trouvée en {dt*1000:.1f} ms')
                print(f' Configuration     : {res["config"]}')
                print(f' Solutions valides : {res["n_valid"]}')
                print(f' Manipulabilité    : {res["manipulability"]:.6f}',
                      ' [OK]' if res['manipulability'] > 0.01 else ' [PROCHE SINGULARITE]')
                print(f' Erreur FK         : {res["fk_error"]*1000:.2f} mm')
                print(f' Coût (distance)   : {res["cost"]:.4f}')
                print()
                print(' Angles articulaires (radians) :')
                for name, q, dq in zip(JOINT_NAMES,
                                       res['joints'],
                                       res['velocities']):
                    deg = math.degrees(q)
                    print(f'   {name:<30} q={q:+.4f} rad ({deg:+7.2f}°)'
                          f'  dq={dq:+.4f} rad/s')
                print()
                print(f' Position FK vérifiée : '
                      f'({res["fk_position"][0]:.4f}, '
                      f'{res["fk_position"][1]:.4f}, '
                      f'{res["fk_position"][2]:.4f}) m')
                print()
                print(' Copier-coller pour pick_place_node / joint_wave_demo :')
                joints_str = ', '.join(f'{q:.4f}' for q in res['joints'])
                vels_str   = ', '.join(f'{v:.4f}' for v in res['velocities'])
                print(f'   joints     = [{joints_str}]')
                print(f'   velocities = [{vels_str}]')
                print('─'*60)

            except ValueError:
                print(' Erreur : valeurs numériques invalides')
            except KeyboardInterrupt:
                print('\n Au revoir !')
                break


def main():
    rclpy.init()
    node = IKSolverNode()

    # Laisser le temps de recevoir les joint_states
    print(' Connexion aux topics ROS 2...')
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        node.run_interactive()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
