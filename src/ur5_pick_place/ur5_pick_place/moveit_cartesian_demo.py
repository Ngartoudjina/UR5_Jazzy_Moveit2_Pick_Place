#!/usr/bin/env python3
"""
================================================================================
moveit_cartesian_demo.py
================================================================================

Ce script permet à l'utilisateur d'entrer des coordonnées cartésiennes (x, y, z)
pour le bras UR5. Il calcule automatiquement la cinématique inverse (IK) pour
trouver les angles de joints correspondants, puis demande à MoveIt2 de planifier
et exécuter la trajectoire.

Exemple d'utilisation :

    Point 1 : x=0.4  y=0.0  z=0.3   → IK calculé → MoveIt2 exécute
    Point 2 : x=0.3  y=0.2  z=0.4   → IK calculé → MoveIt2 exécute

IMPORTANT :
-----------
- Les coordonnées sont en mètres, dans le repère world du robot
- L'espace de travail du UR5 : rayon ~0.85m, hauteur 0.1m à 0.9m
- MoveIt2 doit être lancé avant ce script :
    ros2 launch ur5_moveit moveit.launch.py
================================================================================
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState

from ur5_pick_place.ik_solver import UR5IKSolver, JOINT_NAMES


GROUP_NAME  = 'ur5_arm'
MOVE_ACTION = '/move_action'


class MoveitCartesianDemo(Node):

    def __init__(self):
        super().__init__('moveit_cartesian_demo')

        # Client MoveIt2
        self.client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.get_logger().info(f'Attente de {MOVE_ACTION} ...')
        self.client.wait_for_server()
        self.get_logger().info('MoveIt2 connecté ✅')

        # Solveur IK
        self.ik_solver = UR5IKSolver(self)

        # Suivi de la position actuelle
        self.current_joints = [0.0] * 6
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        # Laisser le temps de recevoir les joint_states
        self.get_logger().info('Lecture de la position actuelle...')
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f'Position actuelle : '
            f'[{", ".join(f"{q:.2f}" for q in self.current_joints)}]'
        )

    def _joint_cb(self, msg):
        jmap = dict(zip(msg.name, msg.position))
        self.current_joints = [jmap.get(n, 0.0) for n in JOINT_NAMES]

    # ── Envoi d'une commande MoveIt2 ──────────────────────────────────────────
    def move_to_joints(self, joints, velocities, label, point_number,
                       tolerance=0.02, vel_scale=0.3, acc_scale=0.3):
        """
        Envoie une commande de mouvement à MoveIt2 via les angles articulaires.

        joints      : liste de 6 angles (rad)
        velocities  : liste de 6 vitesses (rad/s) — affichage uniquement
        label       : description du point (ex: "au-dessus objet")
        tolerance   : tolérance en radians (défaut ±0.02 rad ≈ ±1°)
        vel_scale   : facteur de vitesse MoveIt2 (0.0 à 1.0)
        acc_scale   : facteur d'accélération MoveIt2 (0.0 à 1.0)
        """
        self.get_logger().info(
            f'\n{"="*54}\n'
            f' POINT {point_number} — {label}\n'
            f'{"="*54}'
        )

        # Afficher les joints et vitesses calculés
        self.get_logger().info('Angles articulaires (IK) :')
        for name, q, dq in zip(JOINT_NAMES, joints, velocities):
            deg = math.degrees(q)
            self.get_logger().info(
                f'  {name:<30} q={q:+.4f} rad ({deg:+7.2f}°)'
                f'  dq={dq:+.4f} rad/s'
            )

        # Construction du goal MoveIt2
        goal = MoveGroup.Goal()
        req  = goal.request

        req.group_name                      = GROUP_NAME
        req.allowed_planning_time           = 20.0
        req.num_planning_attempts           = 20
        req.max_velocity_scaling_factor     = vel_scale
        req.max_acceleration_scaling_factor = acc_scale
        req.pipeline_id                     = 'ompl'
        req.start_state.is_diff             = True

        # Contraintes articulaires
        constraints = Constraints()
        for name, q in zip(JOINT_NAMES, joints):
            jc                 = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(q)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints = [constraints]

        # Envoi et attente
        self.get_logger().info('Envoi à MoveIt2...')
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()

        if not gh or not gh.accepted:
            self.get_logger().error('MoveIt2 a refusé ce point ❌')
            return False

        self.get_logger().info('Planification et exécution en cours...')
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        result = rf.result()

        if not result:
            self.get_logger().error('Aucun résultat reçu ❌')
            return False

        code = result.result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Mouvement réussi ✅')
            # Mettre à jour la position courante
            rclpy.spin_once(self, timeout_sec=0.5)
            return True

        self.get_logger().error(f'Échec du mouvement (code={code}) ❌')
        return False

    # ── Résolution IK + mouvement ─────────────────────────────────────────────
    def move_to_cartesian(self, x, y, z,
                          qx=0.0, qy=0.707, qz=0.0, qw=0.707,
                          label='', point_number=1,
                          vel_scale=0.3, acc_scale=0.3):
        """
        Calcule l'IK pour (x, y, z) puis commande MoveIt2.
        Retourne True si succès, False sinon.
        """
        self.get_logger().info(
            f'Calcul IK pour ({x:.3f}, {y:.3f}, {z:.3f}) ...'
        )

        t0  = time.time()
        res = self.ik_solver.solve(
            x, y, z, qx, qy, qz, qw,
            v_cartesian=[0.0, 0.0, -0.05, 0.0, 0.0, 0.0],
            n_attempts=8
        )
        dt = time.time() - t0

        if res is None:
            self.get_logger().error(
                f'IK impossible pour ({x:.3f}, {y:.3f}, {z:.3f})\n'
                f'Vérifiez que la cible est dans l\'espace de travail :\n'
                f'  rayon max ~0.85m depuis la base\n'
                f'  hauteur   0.1m à 0.9m'
            )
            return False

        self.get_logger().info(
            f'IK résolue en {dt*1000:.1f} ms | '
            f'config={res["config"]} | '
            f'manipulabilité={res["manipulability"]:.4f} | '
            f'erreur FK={res["fk_error"]*1000:.2f} mm'
        )

        return self.move_to_joints(
            res['joints'],
            res['velocities'],
            label or f'({x:.3f}, {y:.3f}, {z:.3f})',
            point_number,
            vel_scale=vel_scale,
            acc_scale=acc_scale,
        )


# ── Interface utilisateur ─────────────────────────────────────────────────────
def ask_user_for_points():
    """
    Demande à l'utilisateur les points cartésiens à atteindre.
    Retourne une liste de dicts avec les paramètres de chaque point.
    """
    print()
    print('╔' + '═'*56 + '╗')
    print('║   Démo cartésienne UR5 — Saisie des points cibles       ║')
    print('╚' + '═'*56 + '╝')
    print()
    print('  Espace de travail du UR5 :')
    print('    x : 0.1  à  0.85 m   (devant le robot)')
    print('    y : -0.5 à  0.5  m   (gauche/droite)')
    print('    z : 0.0  à  0.9  m   (hauteur)')
    print()
    print('  Orientation par défaut : outil pointant vers le bas')
    print('  (qx=0.0  qy=0.707  qz=0.0  qw=0.707)')
    print()

    # Paramètres globaux
    print('─'*58)
    while True:
        try:
            raw = input('  Facteur de vitesse MoveIt2 [0.1-1.0] (défaut 0.3) : ').strip()
            vel_scale = float(raw) if raw else 0.3
            vel_scale = max(0.05, min(1.0, vel_scale))
            break
        except ValueError:
            print('  Valeur invalide.')

    while True:
        try:
            raw = input('  Facteur d\'accélération    [0.1-1.0] (défaut 0.3) : ').strip()
            acc_scale = float(raw) if raw else 0.3
            acc_scale = max(0.05, min(1.0, acc_scale))
            break
        except ValueError:
            print('  Valeur invalide.')

    print()
    print('─'*58)

    # Nombre de points
    while True:
        try:
            n = int(input('  Combien de points veux-tu tester ? : '))
            if n > 0:
                break
            print('  Entre un nombre positif.')
        except ValueError:
            print('  Valeur invalide.')

    points = []

    for i in range(n):
        print()
        print(f'  ── Point {i+1}/{n} ──────────────────────────────────')
        print('  Entre les coordonnées : x y z')
        print('  Exemple : 0.4 0.0 0.3')
        print('  (ou "x y z qx qy qz qw" pour une orientation custom)')

        while True:
            try:
                raw = input(f'  Point {i+1} > ').strip()
                if not raw:
                    continue
                parts = [float(v) for v in raw.split()]

                if len(parts) == 3:
                    x, y, z = parts
                    qx, qy, qz, qw = 0.0, 0.707, 0.0, 0.707
                elif len(parts) == 7:
                    x, y, z, qx, qy, qz, qw = parts
                else:
                    print('  Erreur : entre 3 valeurs (x y z) ou 7 (x y z qx qy qz qw)')
                    continue

                # Vérification grossière de l'espace de travail
                dist = (x**2 + y**2 + z**2)**0.5
                if dist > 0.9:
                    print(f'  Attention : distance {dist:.2f}m > 0.9m, '
                          f'peut être hors portée.')

                label = input(
                    f'  Label pour ce point (Entrée = "point {i+1}") : '
                ).strip() or f'point {i+1}'

                points.append({
                    'x': x, 'y': y, 'z': z,
                    'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                    'label': label,
                    'vel_scale': vel_scale,
                    'acc_scale': acc_scale,
                })
                break

            except ValueError:
                print('  Valeur invalide.')

    return points


# ── Programme principal ───────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = MoveitCartesianDemo()

    try:
        points = ask_user_for_points()

        node.get_logger().info(
            f'\n{len(points)} point(s) reçu(s). Début des mouvements...'
        )

        success_count = 0

        for i, pt in enumerate(points, start=1):
            ok = node.move_to_cartesian(
                x=pt['x'], y=pt['y'], z=pt['z'],
                qx=pt['qx'], qy=pt['qy'], qz=pt['qz'], qw=pt['qw'],
                label=pt['label'],
                point_number=i,
                vel_scale=pt['vel_scale'],
                acc_scale=pt['acc_scale'],
            )
            if ok:
                success_count += 1
            else:
                node.get_logger().error(
                    f'Point {i} échoué — arrêt du programme.'
                )
                break

            # Pause entre les mouvements
            if i < len(points):
                time.sleep(0.5)

        node.get_logger().info(
            f'\n{"="*54}\n'
            f' Résultat : {success_count}/{len(points)} points réussis\n'
            f'{"="*54}'
        )

    except KeyboardInterrupt:
        node.get_logger().info('\nArrêt par Ctrl+C')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
