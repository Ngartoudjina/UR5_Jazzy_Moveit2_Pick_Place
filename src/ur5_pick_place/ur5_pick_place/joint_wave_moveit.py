#!/usr/bin/env python3

"""
================================================================================
moveit_joint_points_demo.py
================================================================================

Ce script permet à l'utilisateur d'entrer manuellement plusieurs positions de
joints pour le bras UR5.

Exemple d'utilisation :

L'utilisateur peut entrer :

    Point 1 : 0.0 -1.57 1.57 -1.57 -1.57 0.0
    Point 2 : 0.5 -1.20 1.20 -1.50 -1.57 0.3
    Point 3 : 0.2 -1.80 1.60 -1.20 -1.30 -0.5

Puis le script demandera à MoveIt2 de :

1. planifier une trajectoire vers le premier point
2. exécuter cette trajectoire
3. planifier ensuite vers le second point
4. etc.

Ainsi, tu peux tester facilement différents mouvements du bras dans RViz et
mieux comprendre comment MoveIt2 travaille.

IMPORTANT :
------------
Les valeurs doivent être données en radians.

Rappel rapide :
- 0 rad      = 0°
- 1.57 rad   ≈ 90°
- -1.57 rad  ≈ -90°
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Action MoveIt2 utilisée pour demander un mouvement
from moveit_msgs.action import MoveGroup

# Messages permettant de définir les contraintes sur les joints
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


# =============================================================================
# CONSTANTES
# =============================================================================

# Nom du groupe MoveIt2 correspondant au bras.
# Selon ta configuration UR5, cela peut être :
# - "arm"
# - "ur_manipulator"
# Vérifie dans ton fichier SRDF ou dans RViz.
GROUP_NAME = 'arm'

# Nom de l'action serveur MoveIt2.
MOVE_ACTION = '/move_action'

# Ordre exact des joints du robot.
# Cet ordre doit TOUJOURS être respecté.
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


# =============================================================================
# CLASSE PRINCIPALE
# =============================================================================

class MoveItJointPointDemo(Node):

    def __init__(self):

        # Création du noeud ROS2
        super().__init__('moveit_joint_points_demo')

        # Création du client d'action.
        # Ce client va envoyer des demandes de mouvement à MoveIt2.
        self.client = ActionClient(
            self,
            MoveGroup,
            MOVE_ACTION
        )

        self.get_logger().info(
            f'Attente du serveur MoveIt2 : {MOVE_ACTION} ...'
        )

        # On attend que MoveIt2 soit complètement lancé.
        self.client.wait_for_server()

        self.get_logger().info('MoveIt2 connecté ✅')


    # =========================================================================
    # FONCTION QUI ENVOIE UNE POSITION AU ROBOT
    # =========================================================================

    def move_to_joint_positions(self, target_positions, point_number):

        """
        target_positions est une liste de 6 nombres.

        Exemple :

        [0.5, -1.57, 1.2, -1.4, -1.57, 0.0]

        Chaque valeur correspond à un joint dans l'ordre suivant :

        shoulder_pan_joint
        shoulder_lift_joint
        elbow_joint
        wrist_1_joint
        wrist_2_joint
        wrist_3_joint
        """

        self.get_logger().info(
            f'\n================ POINT {point_number} ================'
        )

        self.get_logger().info('Position demandée :')

        # Affiche chaque joint et sa valeur pour aider au débogage.
        for name, value in zip(JOINT_NAMES, target_positions):
            self.get_logger().info(
                f'  {name:<22} : {value:+.3f} rad'
            )

        # ---------------------------------------------------------------------
        # CRÉATION DE L'OBJECTIF MOVEIT2
        # ---------------------------------------------------------------------

        # Objet représentant la demande envoyée à MoveIt2.
        goal = MoveGroup.Goal()

        # request contient tous les paramètres de planification.
        req = goal.request

        # Groupe MoveIt2 à déplacer.
        req.group_name = GROUP_NAME

        # Temps maximum accordé à MoveIt2 pour trouver une trajectoire.
        req.allowed_planning_time = 10.0

        # Nombre d'essais.
        # Si le premier calcul échoue, MoveIt2 réessaiera.
        req.num_planning_attempts = 10

        # On réduit la vitesse et l'accélération pour éviter les mouvements
        # trop brusques pendant les tests.
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        # Choix du planificateur.
        # OMPL est le plus adapté pour commencer.
        req.pipeline_id = 'ompl'

        # Indique à MoveIt2 de partir de la position actuelle réelle du robot.
        req.start_state.is_diff = True

        # ---------------------------------------------------------------------
        # CONSTRUCTION DES CONTRAINTES SUR LES JOINTS
        # ---------------------------------------------------------------------

        # Constraints = ensemble de contraintes.
        constraints = Constraints()

        # Pour chaque joint du robot...
        for joint_name, target in zip(JOINT_NAMES, target_positions):

            # JointConstraint = contrainte individuelle sur UN seul joint.
            jc = JointConstraint()

            # Nom exact du joint concerné.
            jc.joint_name = joint_name

            # Position voulue pour ce joint.
            jc.position = float(target)

            # Tolérance autorisée autour de cette position.
            # Ici : ±0.02 rad ≈ ±1°
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02

            # Importance de cette contrainte.
            # 1.0 signifie : cette contrainte doit absolument être respectée.
            jc.weight = 1.0

            # Ajoute cette contrainte à la liste.
            constraints.joint_constraints.append(jc)

        # On donne toutes les contraintes à MoveIt2.
        req.goal_constraints = [constraints]

        # ---------------------------------------------------------------------
        # ENVOI DE LA DEMANDE À MOVEIT2
        # ---------------------------------------------------------------------

        self.get_logger().info('Envoi de la demande à MoveIt2...')

        # send_goal_async() envoie la demande sans bloquer immédiatement.
        future = self.client.send_goal_async(goal)

        # On attend que MoveIt2 réponde.
        rclpy.spin_until_future_complete(self, future)

        # goal_handle représente la réponse de MoveIt2.
        goal_handle = future.result()

        # Si MoveIt2 refuse l'objectif.
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                'MoveIt2 a refusé ce point ❌'
            )
            return False

        self.get_logger().info(
            'MoveIt2 a accepté le point ✅'
        )

        self.get_logger().info(
            'Planification et exécution de la trajectoire en cours...'
        )

        # Maintenant MoveIt2 calcule la trajectoire puis l'exécute.
        result_future = goal_handle.get_result_async()

        # On attend la fin réelle du mouvement.
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        # Si aucun résultat n'est revenu.
        if not result:
            self.get_logger().error('Aucun résultat reçu ❌')
            return False

        # Code retourné par MoveIt2.
        code = result.result.error_code.val

        # SUCCESS vaut généralement 1.
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                'Trajectoire exécutée avec succès ✅'
            )
            return True

        self.get_logger().error(
            f'Échec du mouvement, code retourné = {code} ❌'
        )
        return False


# =============================================================================
# FONCTION QUI DEMANDE LES POINTS À L'UTILISATEUR
# =============================================================================

def ask_user_for_points():

    """
    Cette fonction demande à l'utilisateur combien de points il souhaite entrer,
    puis demande les 6 valeurs de joints pour chaque point.
    """

    print('\nCombien de points veux-tu tester ?')

    while True:
        try:
            n_points = int(input('Nombre de points : '))

            if n_points <= 0:
                print('Entre un nombre strictement positif.')
                continue

            break

        except ValueError:
            print('Entrée invalide. Entre un entier.')

    all_points = []

    # Pour chaque point demandé...
    for i in range(n_points):

        print(f'\nPoint {i + 1}/{n_points}')
        print('Entre les 6 joints séparés par des espaces :')
        print('shoulder_pan shoulder_lift elbow wrist1 wrist2 wrist3')
        print('Exemple : 0.5 -1.57 1.57 -1.57 -1.57 0.0')

        while True:
            raw = input('> ')

            try:
                values = [float(x) for x in raw.split()]

                # Il faut exactement 6 valeurs.
                if len(values) != 6:
                    print('Erreur : tu dois entrer exactement 6 nombres.')
                    continue

                all_points.append(values)
                break

            except ValueError:
                print('Erreur : certaines valeurs ne sont pas des nombres.')

    return all_points


# =============================================================================
# PROGRAMME PRINCIPAL
# =============================================================================

def main():

    # Initialisation de ROS2
    rclpy.init()

    # Création du noeud MoveIt2
    node = MoveItJointPointDemo()

    try:

        # Demande à l'utilisateur la liste des points à tester.
        points = ask_user_for_points()

        node.get_logger().info(
            f'\n{len(points)} point(s) reçu(s). Début des mouvements...'
        )

        # Exécute les points un par un.
        for i, point in enumerate(points, start=1):

            success = node.move_to_joint_positions(point, i)

            # Si un mouvement échoue, on arrête la suite.
            if not success:
                node.get_logger().error(
                    'Arrêt du programme à cause d\'une erreur.'
                )
                break

        node.get_logger().info('\nFin du programme.')

    except KeyboardInterrupt:
        node.get_logger().info('\nArrêt demandé par Ctrl+C')

    finally:
        # Nettoyage propre du noeud ROS2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
