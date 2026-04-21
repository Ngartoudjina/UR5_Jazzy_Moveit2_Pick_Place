#!/usr/bin/env python3

"""
================================================================================
joint_wave_demo.py
================================================================================

But du script
--------------
Ce script permet de faire bouger automatiquement tous les joints du bras UR5
selon un mouvement sinusoïdal.

L'objectif n'est PAS de déplacer le robot vers une position précise comme avec
MoveIt2, mais simplement de vérifier que :

- chaque joint répond correctement
- le contrôleur ros2_control fonctionne
- les limites des joints semblent correctes
- le robot peut exécuter une trajectoire complète

Le mouvement ressemble à une "vague" continue : chaque articulation bouge
doucement avec une fréquence différente.

IMPORTANT :
------------
Ce script n'utilise PAS MoveIt2.

Il envoie directement une trajectoire au contrôleur ROS2 du robot :

    /ur5_arm_controller/follow_joint_trajectory

Le robot suit alors la trajectoire exactement telle qu'on la lui donne.

================================================================================
IMPORTS
================================================================================
"""

import math
# Sert à utiliser sin(), cos() et pi pour créer les mouvements sinusoïdaux

import rclpy
# Bibliothèque ROS2 Python principale

from rclpy.node import Node
# Permet de créer un noeud ROS2

from rclpy.action import ActionClient
# Permet de communiquer avec un "serveur d'action"

from control_msgs.action import FollowJointTrajectory
# Type d'action ROS2 utilisé par ros2_control pour recevoir une trajectoire

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# JointTrajectory      = toute la trajectoire
# JointTrajectoryPoint = un point individuel de la trajectoire

from builtin_interfaces.msg import Duration
# Sert à exprimer le temps ROS2 : 2 secondes, 50 ms, etc.


"""
================================================================================
NOM DU SERVEUR D'ACTION
================================================================================

Le contrôleur ros2_control expose une action ROS2.

Une action ROS2 est un service spécial qui :
- reçoit une demande
- travaille pendant plusieurs secondes
- peut envoyer une progression
- renvoie un résultat à la fin

Ici le serveur est :

    /ur5_arm_controller/follow_joint_trajectory

Cela signifie :

    "Je suis un contrôleur de bras.
     Envoie-moi une trajectoire et je la suivrai."

"""
ACTION_SERVER = '/ur5_arm_controller/follow_joint_trajectory'


"""
================================================================================
LISTE DES JOINTS DU BRAS
================================================================================

L'ordre des joints est TRÈS IMPORTANT.

Les positions envoyées devront toujours être dans exactement cet ordre :

position[0] -> shoulder_pan_joint
position[1] -> shoulder_lift_joint
position[2] -> elbow_joint
position[3] -> wrist_1_joint
position[4] -> wrist_2_joint
position[5] -> wrist_3_joint

Si l'ordre est mauvais, le mauvais joint bougera.
"""
JOINT_NAMES = [
    'shoulder_pan_joint',   # rotation de la base
    'shoulder_lift_joint',  # épaule
    'elbow_joint',          # coude
    'wrist_1_joint',        # premier poignet
    'wrist_2_joint',        # deuxième poignet
    'wrist_3_joint',        # dernier poignet
]


"""
================================================================================
POSITION DE DÉPART
================================================================================

Cette position correspond à une posture "sûre" du bras.

Les valeurs sont en radians.

Exemple :
- 0.0 rad      = 0°
- 1.57 rad     ≈ 90°
- -1.57 rad    ≈ -90°

Cette position est utilisée comme centre du mouvement sinusoïdal.

Le robot ne partira donc jamais de zéro absolu ; il oscillera autour de cette
posture.

Exemple :
si shoulder_pan_joint vaut 0.5 rad et qu'on ajoute une sinusoïde de ±0.4 rad,
alors ce joint bougera entre :

    0.5 - 0.4 = 0.1 rad
et
    0.5 + 0.4 = 0.9 rad
"""
START_POS = [0.5, -1.57, 1.57, -1.57, -1.57, 0.0]


# ==============================================================================
# CLASSE PRINCIPALE
# ==============================================================================

class JointWaveDemo(Node):

    def __init__(self):

        # Création du noeud ROS2 appelé "joint_wave_demo"
        super().__init__('joint_wave_demo')

        """
        Création du client d'action.

        Ici notre noeud devient un "client" qui parle au contrôleur du robot.

        Client  ---->  /ur5_arm_controller/follow_joint_trajectory
        """

        self.client = ActionClient(
            self,                        # le noeud ROS2 courant
            FollowJointTrajectory,       # type de l'action
            ACTION_SERVER                # nom du serveur
        )

        # Message affiché dans le terminal
        self.get_logger().info(
            f'Attente du serveur {ACTION_SERVER} ...'
        )

        """
        wait_for_server()

        Le programme reste bloqué ici tant que le contrôleur n'est pas prêt.

        Si le robot ou ros2_control n'est pas lancé, le script attendra.
        """
        self.client.wait_for_server()

        self.get_logger().info(
            'Contrôleur ros2_control trouvé et prêt ✅'
        )


    # ==========================================================================
    # CONSTRUCTION DE LA TRAJECTOIRE
    # ==========================================================================

    def build_trajectory(self, n_points=200, duration=40.0):

        """
        Cette fonction construit toute la trajectoire du robot.

        Paramètres :
        ------------
        n_points : nombre de points dans la trajectoire
        duration : durée totale en secondes

        Exemple :
        si n_points = 200 et duration = 40 s

        alors le robot reçoit 200 points répartis sur 40 secondes.

        Donc environ un nouveau point toutes les 0.2 secondes.
        """

        # Objet qui contiendra toute la trajectoire
        traj = JointTrajectory()

        # On indique quels joints sont concernés
        traj.joint_names = JOINT_NAMES


        """
        AMPLITUDES
        ----------

        Valeur maximale du mouvement de chaque joint autour de START_POS.

        Exemple :
        amplitude = 0.4 rad

        Le joint pourra bouger de :
            +0.4 rad
        ou
            -0.4 rad

        autour de sa position de départ.
        """
        amplitudes = [0.4, 0.3, 0.4, 0.3, 0.3, 0.4]


        """
        FRÉQUENCES RELATIVES
        --------------------

        Chaque joint bouge à une vitesse différente.

        Plus la fréquence est grande, plus le joint oscillera rapidement.

        Ici :
        - wrist_3_joint bouge le plus vite
        - shoulder_lift_joint bouge plus lentement
        """
        freqs = [1.0, 0.8, 1.2, 1.5, 2.0, 2.5]


        # ----------------------------------------------------------------------
        # Création de tous les points de la trajectoire
        # ----------------------------------------------------------------------
        for i in range(n_points):

            """
            t représente l'angle de progression dans la sinusoïde.

            Quand i va de 0 à 199, t va de 0 à 2*pi.

            Donc on parcourt une période complète de sin().
            """
            t = i * (2 * math.pi / n_points)


            """
            POSITION DE CHAQUE JOINT
            ------------------------

            Formule :

                position = position_depart + amplitude * sin(...)

            Cela produit un mouvement fluide.

            Si sin(...) = 0  -> on est à la position de départ
            Si sin(...) = 1  -> on est au maximum
            Si sin(...) = -1 -> on est au minimum
            """
            positions = [
                START_POS[j] + amplitudes[j] * math.sin(t * freqs[j])
                for j in range(6)
            ]


            """
            VITESSE DE CHAQUE JOINT
            -----------------------

            Le contrôleur aime aussi connaître la vitesse prévue.

            La dérivée de sin() est cos()

            Donc :

                vitesse = amplitude * fréquence * cos(...)

            Puis on multiplie encore par (2*pi/duration)
            pour adapter la vitesse à la durée totale de la trajectoire.
            """
            velocities = [
                amplitudes[j] * freqs[j] * math.cos(t * freqs[j])
                * (2 * math.pi / duration)
                for j in range(6)
            ]


            """
            TEMPS DU POINT COURANT
            ----------------------

            Chaque point doit dire à quel instant il doit être atteint.

            Exemple :
            point 0   -> 0 s
            point 50  -> 10 s
            point 100 -> 20 s
            ...
            """
            time_sec = duration * i / n_points

            sec = int(time_sec)
            nsec = int((time_sec - sec) * 1e9)


            """
            IMPORTANT :
            Le premier point ne doit pas être exactement à 0.0 seconde.

            Certains contrôleurs ROS2 refusent une trajectoire dont le premier
            point est à t=0.

            On met donc 10 ms.
            """
            if i == 0:
                sec = 0
                nsec = 10_000_000   # 10 ms


            # Création d'un point individuel
            point = JointTrajectoryPoint()

            # Position des 6 joints à cet instant
            point.positions = positions

            # Vitesse des 6 joints à cet instant
            point.velocities = velocities

            # Temps auquel ce point doit être atteint
            point.time_from_start = Duration(
                sec=sec,
                nanosec=nsec
            )

            # On ajoute ce point à la trajectoire
            traj.points.append(point)


        """
        AJOUT DU DERNIER POINT
        ----------------------

        On ajoute un dernier point identique au premier.

        Pourquoi ?

        Parce que la sinusoïde doit revenir proprement à sa position initiale.

        Sinon le robot finirait brusquement dans une autre posture.
        """
        last = JointTrajectoryPoint()

        last.positions = [
            START_POS[j] + amplitudes[j] * math.sin(0.0)
            for j in range(6)
        ]

        # On veut que le robot soit immobile à la fin
        last.velocities = [0.0] * 6

        # Ce dernier point est placé à exactement "duration" secondes
        last.time_from_start = Duration(
            sec=int(duration),
            nanosec=0
        )

        traj.points.append(last)

        # On renvoie la trajectoire complète
        return traj


    # ==========================================================================
    # ENVOI EN BOUCLE DE LA TRAJECTOIRE
    # ==========================================================================

    def run_forever(self):

        cycle = 0

        self.get_logger().info(
            'Démarrage de la démonstration — Ctrl+C pour arrêter'
        )

        while rclpy.ok():

            cycle += 1

            self.get_logger().info(
                f'\n=== Cycle {cycle} : envoi de la trajectoire ==='
            )

            """
            Création d'un objectif d'action.

            FollowJointTrajectory.Goal() est le message envoyé au contrôleur.
            """
            goal = FollowJointTrajectory.Goal()

            # On met dedans notre trajectoire
            goal.trajectory = self.build_trajectory(
                n_points=200,
                duration=40.0
            )


            """
            Envoi asynchrone de la trajectoire.

            send_goal_async() signifie :

                "voici la trajectoire, traite-la en arrière-plan"
            """
            future = self.client.send_goal_async(goal)

            """
            On attend que le contrôleur réponde :

            - accepté
            - refusé
            """
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()


            # Si refus
            if not goal_handle or not goal_handle.accepted:

                self.get_logger().error('Trajectoire refusée ❌')

                self.get_logger().error(
                    'Vérifie que le contrôleur est actif avec :\n'
                    'ros2 control list_controllers'
                )

                return


            self.get_logger().info(
                'Trajectoire acceptée ✅'
            )

            self.get_logger().info(
                'Le robot exécute maintenant les mouvements...'
            )


            """
            Une fois acceptée, on attend la fin de l'exécution réelle.
            """
            result_future = goal_handle.get_result_async()

            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()


            if result:

                code = result.result.error_code

                """
                Code 0 = succès
                """
                if code == 0:
                    self.get_logger().info(
                        f'Cycle {cycle} terminé avec succès ✅'
                    )
                else:
                    self.get_logger().warn(
                        f'Cycle {cycle} terminé avec erreur code={code}'
                    )

            else:
                self.get_logger().warn(
                    'Aucun résultat renvoyé par le contrôleur'
                )


# ==============================================================================
# FONCTION PRINCIPALE
# ==============================================================================

def main():

    # Démarrage de ROS2
    rclpy.init()

    # Création du noeud
    node = JointWaveDemo()

    try:
        # Lancement du mouvement infini
        node.run_forever()

    except KeyboardInterrupt:
        # Appui sur Ctrl+C
        node.get_logger().info('Arrêt demandé par Ctrl+C')

    finally:
        # Nettoyage propre
        node.destroy_node()
        rclpy.shutdown()


# Lance main() si on exécute directement le fichier
if __name__ == '__main__':
    main()