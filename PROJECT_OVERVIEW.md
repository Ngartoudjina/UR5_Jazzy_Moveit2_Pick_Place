# UR5 Jazzy MoveIt2 Pick & Place — Vue d'ensemble du projet

## Contexte général

Ce projet implémente une séquence de **pick & place** (saisie et dépôt d'objet) pour un bras robotique **UR5 6-DOF** équipé d'un **gripper Robotiq 85**, dans un environnement de simulation **Gazebo + ROS2**. Le code est écrit en **Python**, tourne sous **ROS2 Humble**, et utilise **MoveIt2** pour la planification de trajectoire.

---

## Structure des packages

```
UR5_Jazzy_Moveit2_Pick_Place/
├── src/
│   ├── ur5_pick_place/          ← Package principal (pick & place logic)
│   │   ├── ur5_pick_place/      ← Modules Python
│   │   ├── launch/              ← Fichiers de lancement
│   │   ├── config/              ← Configuration YAML
│   │   └── test/
│   └── ur5_ws/
│       └── src/
│           ├── ur5_description/ ← URDF, monde Gazebo, launch simulation
│           ├── ur5_controller/  ← Configuration ros2_control
│           └── ur5_moveit/      ← Configuration MoveIt2
├── config/
│   └── ur5_pick_place_config.yaml
├── scripts/
├── quick_validation.py
└── VALIDATION_REPORT.md
```

---

## Architecture logicielle

### Couches principales

```
┌──────────────────────────────────────────────────────┐
│              pick_place_node.py                       │
│  (orchestrateur principal, séquence 8 étapes)        │
│  Utilise MoveGroup action + IK service               │
├──────────────────────────────────────────────────────┤
│           base_pick_place.py                         │
│  (classe de base avec ActionClient                   │
│   FollowJointTrajectory + validation)                │
├────────────────────┬─────────────────────────────────┤
│  config_loader.py  │  validators.py                  │
│  (YAML singleton)  │  (JointValidator, Workspace)    │
├────────────────────┴─────────────────────────────────┤
│  gripper_controller.py   ik_solver.py               │
│  (abstraction gripper)   (cinématique inverse KDL)  │
└──────────────────────────────────────────────────────┘
```

### Modules clés

| Fichier | Rôle |
|---|---|
| `pick_place_node.py` | Nœud principal : attend `/object_pose`, calcule l'IK, exécute la séquence complète via MoveGroup |
| `base_pick_place.py` | Classe de base : envoie des trajectoires via `FollowJointTrajectory` action, attend la confirmation de fin |
| `async_pick_place_node.py` | Variante event-driven avec machine d'état explicite (sans `time.sleep()`) |
| `config_loader.py` | Singleton YAML pour charger les positions nommées et profils de gripper |
| `validators.py` | Vérifie les limites articulaires (±2π pour tous les joints) et l'espace de travail |
| `gripper_controller.py` | Abstraction du gripper avec mode simulé (timer one-shot, non-bloquant) |
| `ik_solver.py` | Solveur cinématique inverse basé sur KDL |
| `fake_object_pose.py` | Publie une pose simulée sur `/object_pose` pour les tests sans caméra |
| `add_object.py` | Ajoute des objets de collision dans la planning scene MoveIt2 |
| `object_detector_node.py` | Détecteur d'objet (pour utilisation avec caméra réelle) |
| `pick_place_state_machine.py` | Machine d'état pour la séquence pick & place |
| `moveit_cartesian_demo.py` | Démo de planification cartésienne |
| `moveit_system_diagnostics.py` | Diagnostic de l'état du système MoveIt2 |
| `hand_arm_controller.py` | Contrôle gestuel du bras et de la main |

---

## Séquence pick & place (8 étapes)

```
[0/8] Setup scène     → Ouvrir gripper, ajouter table + objet dans MoveIt2
[1/8] HOME            → Position de repos sécurisée
[2/8] PRE-PICK        → Approche au-dessus de l'objet (+20 cm)
[3/8] GRASP           → Descente vers l'objet (hauteur wrist_3 = oz + 0.130 m)
[4/8] CLOSE + ATTACH  → Fermer gripper, attacher l'objet à wrist_3_link dans MoveIt2
[5/8] LIFT            → Remonter (retour à PRE-PICK)
[6/8] PRE-PLACE       → Approche de la zone de dépôt
[7/8] PLACE           → Descente vers la zone de dépôt
[8/8] RELEASE         → Ouvrir gripper, détacher l'objet
[+]   RETREAT + HOME  → Retraite et retour position repos
```

---

## Configuration du robot

### Positions articulaires nommées (en radians)

| Nom | shoulder_pan | shoulder_lift | elbow | wrist_1 | wrist_2 | wrist_3 |
|---|---|---|---|---|---|---|
| home | 0.0 | -1.5708 | 1.5708 | -1.5708 | -1.5708 | 0.0 |
| pre_pick | 0.5236 | -1.3963 | 1.5708 | -1.1781 | -1.5708 | 0.5236 |
| grasp | 0.5236 | -1.0472 | 1.5708 | -1.5708 | -1.5708 | 0.5236 |
| lift | 0.5236 | -1.5708 | 1.5708 | -1.5708 | -1.5708 | 0.5236 |
| pre_place | -1.0472 | -1.3963 | 1.5708 | -1.1781 | -1.5708 | -1.0472 |
| place | -1.0472 | -1.0472 | 1.5708 | -1.5708 | -1.5708 | -1.0472 |
| retreat | -1.0472 | -1.5708 | 1.5708 | -1.5708 | -1.5708 | -1.0472 |

### Paramètres de planification

- Planificateur : **OMPL / RRTConnect**
- Solveur IK : **KDL** (`kdl_kinematics_plugin`)
- Vitesse max : 25% (configurable, 15% par défaut dans le launch)
- Accélération max : 25%
- Temps de planification : 15–30 s
- Tentatives de planification : 20–30

### Gripper Robotiq 85

| Profil | Position | Effort |
|---|---|---|
| open | 0.0 | 0.0 N |
| medium_grasp | 0.6 | 100 N |
| heavy_grasp (closed) | 0.72 | 135 N |

- Range d'ouverture : 15–90 mm
- Objet cible : 50×50×100 mm (largeur 50 mm, compatible)
- Action : `/hand_controller/gripper_cmd`
- Joint commandé : `robotiq_85_left_knuckle_joint`

---

## Scène de simulation

### Environnement Gazebo

Le monde SDF (`gazebo_world.sdf`) contient :
- Sol (ground_plane)
- Soleil (éclairage directionnel)
- Le robot UR5 (spawné via `robot_description`)

### Objets dans la planning scene MoveIt2

Ajoutés programmatiquement à l'exécution :

| Objet | Position (x, y, z) m | Taille (x, y, z) m |
|---|---|---|
| table | (0.65, 0.0, -0.025) | 0.8 × 0.8 × 0.05 |
| target_box | (0.50, 0.10, 0.05) | 0.05 × 0.05 × 0.05 |

### Calculs de portée UR5

- Portée max réelle : **850 mm**
- Robot → table : 650 mm (76%)
- Robot → objet : 510 mm (60%)
- Robot → zone place : 580 mm (68%)

---

## Contrôleurs ros2_control

| Contrôleur | Type | Topic/Action |
|---|---|---|
| `ur5_arm_controller` | JointTrajectoryController | `/ur5_arm_controller/follow_joint_trajectory` |
| `hand_controller` | GripperActionController | `/hand_controller/gripper_cmd` |
| `joint_state_broadcaster` | JointStateBroadcaster | `/joint_states` |

Fréquence de mise à jour : 100 Hz

---

## Interfaces ROS2

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Lecture |
| `/object_pose` | `geometry_msgs/PoseStamped` | Lecture |
| `/robot_description` | `std_msgs/String` | Publication |

### Services

| Service | Type |
|---|---|
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` |

### Actions

| Action | Type |
|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup` |
| `/ur5_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/hand_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |

---

## Lancement du système

### Procédure complète (3 terminaux)

**Terminal 1 — Simulation Gazebo**
```bash
source /opt/ros/humble/setup.bash
source src/ur5_ws/install/setup.bash
ros2 launch ur5_description gazebo.launch.py
```

**Terminal 2 — MoveIt2 + ros2_control (mock hardware)**
```bash
source /opt/ros/humble/setup.bash
source src/ur5_ws/install/setup.bash
ros2 launch ur5_moveit moveit.launch.py
# Attendre "Move group ready" (~5 secondes)
```

**Terminal 3 — Séquence pick & place**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_pick_place pick_place.launch.py
# Options :
# velocity_scale:=0.3  (vitesse plus rapide)
# use_fake_pose:=false  (utiliser caméra réelle)
```

### Commandes individuelles disponibles

```bash
# Nœuds principaux
ros2 run ur5_pick_place pick_place_node
ros2 run ur5_pick_place async_pick_place

# Utilitaires
ros2 run ur5_pick_place add_object
ros2 run ur5_pick_place fake_object_pose
ros2 run ur5_pick_place pose_sender

# Diagnostic
ros2 run ur5_pick_place moveit_diagnostics
ros2 run ur5_pick_place moveit_cartesian_demo

# Tests
ros2 run ur5_pick_place test_gripper
ros2 run ur5_pick_place test_joints
ros2 run ur5_pick_place test_ik_solver
ros2 run ur5_pick_place test_moveit
ros2 run ur5_pick_place test_moveit_levels
ros2 run ur5_pick_place test_moveit_full_pipeline
ros2 run ur5_pick_place test_moveit_scene
ros2 run ur5_pick_place test_scene_initialization
```

---

## Corrections techniques majeures appliquées

Le projet a fait l'objet d'un cycle de corrections documentées :

| # | Problème corrigé |
|---|---|
| C1 | Quaternion IK : `qy=0.7071, qw=0.7071` (rotation -90° autour Y) — l'ancien `qx=1.0` était géométriquement faux |
| C2 | `grasp_height = 0.130 m` (d6=0.0823 + doigts Robotiq ≈ 0.050) |
| C3 | `MultiThreadedExecutor` (4 workers) obligatoire pour `spin_until_future_complete()` sans deadlock |
| C4 | `time.sleep()` remplace `rclpy.spin_once()` dans la boucle d'attente (interdit avec executor actif) |
| C5 | Attach/detach sur `wrist_3_link` (tip réel du groupe `ur5_arm`) |
| C6 | Table ajoutée dans la planning scene MoveIt2 avant tout mouvement |
| C7 | IK robuste : 3 seeds (HOME, neutre, proche-pick) pour maximiser le taux de succès KDL |
| C8 | Messages d'erreur distincts dans `_send_and_wait()` |
| C9 | Nettoyage de la scène à la fin (`remove_box`) pour relances propres |
| C10 | Limites articulaires corrigées à ±2π (±6.28319 rad) pour tous les joints |
| C11 | Timeout KDL : 0.005 s → 0.15 s, tentatives × 1.67 |

---

## Détails d'implémentation notables

### IK — Cinématique inverse

`pick_place_node.py` appelle `/compute_ik` (service MoveIt2 KDL) avec :
- Orientation cible : `qy=0.7071, qw=0.7071` (X du wrist3 pointe vers le sol)
- Contraintes articulaires ±2.5 rad autour de la seed (évite les configurations retournées)
- 3 seeds essayées dans `compute_ik_robust()` : HOME → IK_SEED_PICK → IK_SEED_NEUTRAL

### Exécution des trajectoires

`base_pick_place.py` utilise `FollowJointTrajectory` action (et non un publisher topic aveugle).
La durée est calculée dynamiquement : `max_delta_angle / max_vel`, minimum 1.5 s.

### Séquence de démarrage MoveIt2 (`moveit.launch.py`)

```
t = 0 s : ros2_control_node (charge URDF + contrôleurs)
t = 3 s : spawners (joint_state_broadcaster + ur5_arm_controller + hand_controller)
t = 5 s : move_group (démarre après réception de /joint_states)
```

---

## Dépendances principales

```xml
rclpy, std_msgs, geometry_msgs, sensor_msgs,
trajectory_msgs, builtin_interfaces,
action_msgs, control_msgs,
moveit_msgs, shape_msgs,
pyyaml, rviz2
```

Outils de simulation : **Gazebo Harmonic** (via `ros_gz_sim`, `ros_gz_bridge`)

---

## Points d'extension possibles

- Intégration d'une caméra réelle (topic `/object_pose` déjà prévu, `object_detector_node.py` existant)
- Déploiement sur robot UR5 réel (remplacer `mock_components` par le driver `ur_robot_driver`)
- Ajout de variantes d'objets dans la scène Gazebo (table + objet déjà paramétrés en YAML)
- Planification cartésienne (démo existante dans `moveit_cartesian_demo.py`)
- Contrôle gestuel (module `hand_arm_controller.py` présent)
