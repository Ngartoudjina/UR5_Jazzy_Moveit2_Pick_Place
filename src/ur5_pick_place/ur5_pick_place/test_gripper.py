#!/usr/bin/env python3
"""
===============================================================================
TEST INFINI DE LA PINCE ROBOTIQ 85 AVEC MOVEIT2
===============================================================================

But :
-----
Ce script permet de tester la pince Robotiq 85 dans RViz / MoveIt2.

Il effectue en boucle :

    1. ouverture de la pince
    2. pause
    3. fermeture de la pince
    4. pause
    5. recommence indéfiniment

Le programme s'arrête avec Ctrl+C.

Pourquoi ce script est utile ?
------------------------------
Avant d'utiliser la pince dans un vrai pick-and-place, il faut vérifier que :

- MoveIt2 communique bien avec le groupe "hand"
- la pince bouge réellement dans RViz
- les limites d'ouverture / fermeture sont correctes
- le nom du joint utilisé est le bon

Commande d'exécution :
----------------------
ros2 run ur5_pick_place gripper_test

===============================================================================
"""

# -----------------------------------------------------------------------------
# Bibliothèques standards Python
# -----------------------------------------------------------------------------

import time
# Sert uniquement à faire une pause entre ouverture et fermeture

# -----------------------------------------------------------------------------
# Bibliothèques ROS2
# -----------------------------------------------------------------------------

import rclpy
# rclpy = bibliothèque ROS2 Python

from rclpy.node import Node
# Classe de base de tous les noeuds ROS2

from rclpy.action import ActionClient
# Permet d'envoyer une Action ROS2
# Ici nous allons envoyer une action MoveIt2 de type MoveGroup

# -----------------------------------------------------------------------------
# Messages / Actions MoveIt2
# -----------------------------------------------------------------------------

from moveit_msgs.action import MoveGroup
# Action MoveIt2 utilisée pour demander un mouvement planifié

from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes
)

# Constraints      : ensemble de contraintes imposées au mouvement
# JointConstraint  : contrainte sur une articulation précise
# MoveItErrorCodes : codes de retour (succès, échec, collision, etc.)

# -----------------------------------------------------------------------------
# Constantes globales
# -----------------------------------------------------------------------------

# Code correspondant à "succès" dans MoveIt2
SUCCESS = MoveItErrorCodes.SUCCESS

# Position de pince ouverte
#
# Pour la Robotiq 85, 0.0 signifie généralement complètement ouverte.
GRIPPER_OPEN = 0.0

# Position de pince fermée
#
# 0.72 rad est souvent une bonne valeur pour la fermeture complète.
# Si la pince ne ferme pas totalement, augmenter légèrement.
# Si elle force ou dépasse, diminuer.
GRIPPER_CLOSED = 0.72

# Temps d'attente entre deux mouvements
PAUSE = 1.5


# =============================================================================
# CLASSE PRINCIPALE
# =============================================================================

class GripperTest(Node):
    """
    Noeud ROS2 chargé de piloter la pince.

    Ce noeud :
    ----------
    - se connecte à MoveIt2
    - envoie des ordres d'ouverture / fermeture
    - affiche les résultats dans le terminal
    """

    def __init__(self):
        """
        Constructeur du noeud.

        Il est exécuté automatiquement quand on crée :

            node = GripperTest()
        """

        # Initialise le noeud ROS2 avec le nom "gripper_test"
        super().__init__('gripper_test')

        # ---------------------------------------------------------------------
        # Création du client d'action MoveIt2
        # ---------------------------------------------------------------------
        #
        # /move_action est l'action serveur fournie par MoveIt2.
        #
        # On va lui envoyer des objectifs ("goals") de mouvement.
        #
        self.client = ActionClient(
            self,               # noeud ROS2 courant
            MoveGroup,          # type d'action
            '/move_action'      # nom du serveur MoveIt2
        )

        # Affiche un message dans le terminal
        self.get_logger().info('Attente du serveur MoveIt2 /move_action ...')

        # Attend que MoveIt2 soit prêt
        self.client.wait_for_server()

        # Confirmation
        self.get_logger().info('Connexion à MoveIt2 réussie ✅')

        # Compteur du nombre de cycles ouverture/fermeture
        self.cycle = 0

    # =========================================================================
    # FONCTION QUI ENVOIE UN ORDRE A LA PINCE
    # =========================================================================

    def move_gripper(self, position: float, label: str) -> bool:
        """
        Demande à MoveIt2 de déplacer la pince.

        Paramètres :
        ------------
        position : float
            Valeur cible du joint de pince.

            Exemple :
                0.0  -> pince ouverte
                0.72 -> pince fermée

        label : str
            Texte affiché dans le terminal.
            Exemple : "OPEN" ou "CLOSE"

        Retour :
        --------
        True  -> mouvement réussi
        False -> échec
        """

        # ---------------------------------------------------------------------
        # Création d'un objectif MoveIt2
        # ---------------------------------------------------------------------

        goal = MoveGroup.Goal()

        # goal.request contient tous les paramètres du mouvement
        req = goal.request

        # ---------------------------------------------------------------------
        # Groupe MoveIt à utiliser
        # ---------------------------------------------------------------------
        #
        # Dans ton SRDF, la pince appartient probablement au groupe "hand".
        #
        # Si le nom dans ton projet est différent, remplace "hand".
        #
        req.group_name = 'hand'

        # ---------------------------------------------------------------------
        # Paramètres de planification
        # ---------------------------------------------------------------------

        # Temps maximal autorisé à MoveIt2 pour trouver une trajectoire
        req.allowed_planning_time = 10.0

        # Nombre maximal d'essais de planification
        req.num_planning_attempts = 10

        # Facteur de vitesse
        #
        # 1.0 = vitesse maximale
        # 0.5 = moitié de la vitesse
        #
        req.max_velocity_scaling_factor = 0.5

        # Facteur d'accélération
        req.max_acceleration_scaling_factor = 0.5

        # Choix du pipeline de planification
        #
        # OMPL = bibliothèque de planification la plus classique dans MoveIt2
        req.pipeline_id = 'ompl'

        # Utiliser l'état courant du robot comme point de départ
        req.start_state.is_diff = True

        # ---------------------------------------------------------------------
        # Création de la contrainte sur le joint de la pince
        # ---------------------------------------------------------------------

        jc = JointConstraint()

        # Nom exact du joint de la pince
        #
        # IMPORTANT :
        # Ce nom doit correspondre exactement à celui dans ton URDF.
        #
        # Vérifie avec :
        #     ros2 topic echo /joint_states
        #
        jc.joint_name = 'robotiq_85_left_knuckle_joint'

        # Position cible demandée
        jc.position = float(position)

        # Tolérance supérieure
        #
        # La pince peut finir légèrement au-dessus de la valeur demandée.
        jc.tolerance_above = 0.05

        # Tolérance inférieure
        #
        # La pince peut finir légèrement en-dessous.
        jc.tolerance_below = 0.05

        # Importance de cette contrainte
        #
        # 1.0 = contrainte très importante
        jc.weight = 1.0

        # ---------------------------------------------------------------------
        # Encapsulation de la contrainte dans un objet Constraints
        # ---------------------------------------------------------------------

        constraints = Constraints()

        # On ajoute la contrainte de joint à la liste
        constraints.joint_constraints.append(jc)

        # MoveIt attend une liste de contraintes
        req.goal_constraints = [constraints]

        # ---------------------------------------------------------------------
        # Envoi de l'objectif au serveur MoveIt2
        # ---------------------------------------------------------------------

        self.get_logger().info(
            f'Envoi de la commande {label} '
            f'(position = {position:.2f})'
        )

        # Envoi asynchrone
        future = self.client.send_goal_async(goal)

        # Attend que MoveIt2 réponde
        rclpy.spin_until_future_complete(self, future)

        # Récupère la réponse
        goal_handle = future.result()

        # ---------------------------------------------------------------------
        # Vérifie si MoveIt2 accepte la demande
        # ---------------------------------------------------------------------

        if goal_handle is None:
            self.get_logger().error(
                'MoveIt2 n’a pas répondu à la demande.'
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                'MoveIt2 a rejeté le mouvement.'
            )
            return False

        # ---------------------------------------------------------------------
        # Attend la fin réelle du mouvement
        # ---------------------------------------------------------------------

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        # Code de retour MoveIt2
        code = result.result.error_code.val

        # True si le mouvement a réussi
        ok = (code == SUCCESS)

        # ---------------------------------------------------------------------
        # Affichage du résultat dans le terminal
        # ---------------------------------------------------------------------

        if ok:
            self.get_logger().info(
                f'✅ Cycle {self.cycle} : {label} réussi'
            )
        else:
            self.get_logger().warning(
                f'⚠️ Cycle {self.cycle} : {label} échoué '
                f'(code MoveIt={code})'
            )

        return ok


# =============================================================================
# FONCTION PRINCIPALE
# =============================================================================

def main():
    """
    Point d'entrée du programme.
    """

    # Initialise ROS2
    rclpy.init()

    # Création du noeud
    node = GripperTest()

    # Message de démarrage
    node.get_logger().info(
        '==================================================='
    )
    node.get_logger().info(
        'Test infini de la pince Robotiq 85'
    )
    node.get_logger().info(
        'Appuie sur Ctrl+C pour arrêter'
    )
    node.get_logger().info(
        '==================================================='
    )

    try:
        # Boucle infinie tant que ROS2 fonctionne
        while rclpy.ok():

            # Incrémente le numéro du cycle
            node.cycle += 1

            node.get_logger().info(
                f'\n================ CYCLE {node.cycle} ================'
            )

            # ================================================================
            # ETAPE 1 : OUVERTURE
            # ================================================================

            node.get_logger().info('🟢 Ouverture de la pince...')
            node.move_gripper(GRIPPER_OPEN, 'OPEN')

            # Pause pour laisser le temps de voir le mouvement dans RViz
            time.sleep(PAUSE)

            # ================================================================
            # ETAPE 2 : FERMETURE
            # ================================================================

            node.get_logger().info('🔴 Fermeture de la pince...')
            node.move_gripper(GRIPPER_CLOSED, 'CLOSE')

            # Nouvelle pause
            time.sleep(PAUSE)

    # Gestion de Ctrl+C
    except KeyboardInterrupt:
        node.get_logger().info(
            f'\nArrêt demandé par l’utilisateur après {node.cycle} cycles.'
        )

    # Toujours exécuté avant la fermeture
    finally:
        node.get_logger().info('Fermeture propre du noeud ROS2...')

        node.destroy_node()
        rclpy.shutdown()


# =============================================================================
# LANCEMENT DU SCRIPT
# =============================================================================

# Cette partie permet d'exécuter le fichier directement :
#
#     python3 gripper_test.py
#
# ou via :
#
#     ros2 run ur5_pick_place gripper_test
#
if __name__ == '__main__':
    main()