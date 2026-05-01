# 🤖 UR5 MoveIt2 Pick & Place

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![MoveIt2](https://img.shields.io/badge/MoveIt2-2.x-orange?style=for-the-badge)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

**Système complet de pick & place pour bras robotique UR5 6-DOF sous ROS2 Humble + MoveIt2**

[Démarrage rapide](#-démarrage-rapide) · [Architecture](#-architecture) · [Configuration](#️-configuration) · [Tests](#-tests) · [Roadmap](#-roadmap)

</div>

---

## 📋 Table des matières

- [Vue d'ensemble](#-vue-densemble)
- [Fonctionnalités](#-fonctionnalités)
- [Prérequis](#-prérequis)
- [Installation](#-installation)
- [Démarrage rapide](#-démarrage-rapide)
- [Architecture](#-architecture)
- [Configuration](#️-configuration)
- [Interfaces ROS2](#-interfaces-ros2)
- [Tests](#-tests)
- [Dépannage](#-dépannage)
- [Roadmap](#-roadmap)
- [Contribuer](#-contribuer)

---

## 🎯 Vue d'ensemble

Ce projet implémente une séquence de **pick & place** complète pour un bras robotique **Universal Robots UR5** (6-DOF) équipé d'un **préhenseur Robotiq 85**, dans un environnement simulé sous **Gazebo Harmonic**.

Le système saisit un objet posé sur une table et le dépose dans une zone cible prédéfinie, en utilisant la cinématique inverse analytique, la planification de trajectoire OMPL (RRTConnect) et le contrôle du préhenseur via `ros2_control`.

```
Scène par défaut
───────────────────────────────────────────────────────
  Table       : 0.8 × 0.8 × 0.05 m  →  centre (0.65, 0.0, 0.0)
  Objet cible : 0.05 × 0.05 × 0.10 m  →  centre (0.50, 0.10, 0.05)
  Zone dépôt  : (0.50, -0.30, 0.05)
```

---

## ✨ Fonctionnalités

| Fonctionnalité | Description |
|---|---|
| 🔄 **Séquence 8 étapes** | HOME → PRE-PICK → GRASP → LIFT → PRE-PLACE → PLACE → RELEASE → HOME |
| 🧠 **IK Analytique** | Solveur cinématique inverse basé sur les paramètres DH du UR5, avec variante multi-seed robuste |
| 📐 **Planification OMPL** | Trajectoires joint-space via RRTConnect + descentes cartésiennes strictes (`min_fraction=0.98`) |
| 🦾 **Contrôle préhenseur** | Robotiq 85 non-bloquant via timer callbacks |
| 🗺️ **Planning Scene MoveIt2** | Table et objet ajoutés dynamiquement, attach/detach automatique |
| ⚙️ **Configuration YAML** | Toutes les poses, profils gripper et paramètres modifiables sans recompilation |
| 🧪 **Suite de tests** | ~1 200 lignes couvrant IK, gripper, scene, pipeline MoveIt2 complet |
| 🔌 **Extensible** | Prêt pour caméra réelle (`/object_pose`) et déploiement sur robot physique |

---

## 📦 Prérequis

| Dépendance | Version recommandée |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| MoveIt2 | 2.x |
| Gazebo | Harmonic |
| Python | 3.10+ |
| colcon | latest |

### Paquets ROS2 requis

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  python3-colcon-common-extensions
```

---

## 🛠️ Installation

### 1. Cloner le dépôt

```bash
git clone https://github.com/Ngartoudjina/UR5_Jazzy_Moveit2_Pick_Place.git
cd UR5_Jazzy_Moveit2_Pick_Place
```

### 2. Construire le workspace de simulation

```bash
cd src/ur5_ws
colcon build --symlink-install
source install/setup.bash
cd ../..
```

### 3. Construire le package pick & place

```bash
colcon build --packages-select ur5_pick_place --symlink-install
source install/setup.bash
```

---

## 🚀 Démarrage rapide

### Lancement unifié (recommandé)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_pick_place pick_place_moveit2.launch.py
```

### Lancement modulaire (3 terminaux)

**Terminal 1 — Simulation Gazebo**
```bash
source /opt/ros/humble/setup.bash
source src/ur5_ws/install/setup.bash
ros2 launch ur5_description gazebo.launch.py
```

**Terminal 2 — MoveIt2 + ros2_control**
```bash
source /opt/ros/humble/setup.bash
source src/ur5_ws/install/setup.bash
ros2 launch ur5_moveit moveit.launch.py
# Attendre le message "Move group ready" (~5 secondes)
```

**Terminal 3 — Séquence pick & place**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_pick_place pick_place.launch.py
```

### Options de lancement

```bash
# Ajuster la vitesse d'exécution (défaut : 0.15)
ros2 launch ur5_pick_place pick_place.launch.py velocity_scale:=0.3

# Utiliser une caméra réelle au lieu de la pose simulée
ros2 launch ur5_pick_place pick_place.launch.py use_fake_pose:=false
```

> ⏱️ **Durée d'un cycle complet : ~27 secondes**

---

## 🏗️ Architecture

### Séquence pick & place

```
┌─────────────────────────────────────────────────────────────────┐
│                    Séquence Pick & Place                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  [0/8] SETUP     → Ouvrir gripper, init scène MoveIt2          │
│     ↓                                                           │
│  [1/8] HOME      → Position de repos sécurisée                 │
│     ↓                                                           │
│  [2/8] PRE-PICK  → Survol objet +20 cm  (joint-space)          │
│     ↓                                                           │
│  [3/8] GRASP     → Descente verticale   (cartésien linéaire)   │
│     ↓                                                           │
│  [4/8] CLOSE     → Fermer gripper + attach objet               │
│     ↓                                                           │
│  [5/8] LIFT      → Remontée verticale   (cartésien linéaire)   │
│     ↓                                                           │
│  [6/8] PRE-PLACE → Survol zone cible    (joint-space)          │
│     ↓                                                           │
│  [7/8] PLACE     → Descente verticale   (cartésien linéaire)   │
│     ↓                                                           │
│  [8/8] RELEASE   → Ouvrir gripper + detach objet               │
│     ↓                                                           │
│  [+]   RETREAT   → Retraite + retour HOME                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Couches logicielles

```
┌─────────────────────────────────────────────────────────────┐
│                   pick_place_node.py                        │
│         Orchestrateur principal — séquence 8 étapes         │
│         Utilise MoveGroup action + IK service               │
├─────────────────────────────────────────────────────────────┤
│                   base_pick_place.py                        │
│     Classe de base — ActionClient FollowJointTrajectory     │
├────────────────────┬────────────────────────────────────────┤
│  config_loader.py  │   validators.py                        │
│  (YAML Singleton)  │   (JointValidator + Workspace)         │
├────────────────────┴────────────────────────────────────────┤
│  gripper_controller.py     │   ik_solver.py                 │
│  (timer callbacks)         │   (DH analytique + multi-seed) │
└────────────────────────────┴────────────────────────────────┘
```

### Modules Python

| Module | Rôle | Pattern |
|---|---|---|
| `pick_place_node.py` | Orchestration complète de la séquence | Asynchrone séquentiel |
| `base_pick_place.py` | Envoi et attente de trajectoires via action | Classe de base |
| `ik_solver.py` | IK analytique (paramètres DH UR5) | Solveur direct |
| `ik_solver_robust.py` | IK multi-seed + filtrage singularités | Stratégie robuste |
| `gripper_controller.py` | Contrôle préhenseur non-bloquant | Timer callbacks |
| `config_loader.py` | Chargement YAML des poses nommées | Singleton |
| `validators.py` | Validation joints (±2π) + workspace (≤0.85 m) | Validation frontière |
| `pick_place_state_machine.py` | Machine à états explicite | State machine |
| `add_object.py` | Initialisation scène MoveIt2 | — |
| `fake_object_pose.py` | Pose simulée pour tests sans caméra | — |

### Paramètres DH du UR5

```
d1 = 0.089159 m
a2 = -0.425 m    a3 = -0.39225 m
d4 = 0.10915 m   d5 = 0.09465 m   d6 = 0.0823 m
```

---

## ⚙️ Configuration

Le fichier `config/ur5_pick_place_config.yaml` centralise tous les paramètres :

### Positions articulaires (radians)

| Pose | shoulder_pan | shoulder_lift | elbow | wrist_1 | wrist_2 | wrist_3 |
|---|---|---|---|---|---|---|
| `home` | 0.0 | -1.5708 | 1.5708 | -1.5708 | -1.5708 | 0.0 |
| `pre_pick` | 0.5236 | -1.3963 | 1.5708 | -1.1781 | -1.5708 | 0.5236 |
| `grasp` | 0.5236 | -1.0472 | 1.5708 | -1.5708 | -1.5708 | 0.5236 |
| `pre_place` | -1.0472 | -1.3963 | 1.5708 | -1.1781 | -1.5708 | -1.0472 |

### Profils du préhenseur Robotiq 85

| Profil | Position | Effort |
|---|---|---|
| `open` | 0.0 rad | 0 N |
| `medium_grasp` | 0.6 rad | 100 N |
| `closed` | 0.72 rad | 135 N |

### Paramètres de planification

```yaml
planner:         RRTConnect (OMPL)
ik_solver:       KDL (kdl_kinematics_plugin)
velocity_scale:  0.15  # configurable jusqu'à 1.0
planning_time:   15–30 s
attempts:        20–30
min_fraction:    0.98  # fraction minimale pour descentes cartésiennes
```

---

## 📡 Interfaces ROS2

### Topics

| Topic | Type | Rôle |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Lecture état joints |
| `/object_pose` | `geometry_msgs/PoseStamped` | Pose cible de l'objet |
| `/robot_description` | `std_msgs/String` | Description URDF |

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

### Contrôleurs ros2_control

| Contrôleur | Type | Fréquence |
|---|---|---|
| `ur5_arm_controller` | JointTrajectoryController | 100 Hz |
| `hand_controller` | GripperActionController | 100 Hz |
| `joint_state_broadcaster` | JointStateBroadcaster | 100 Hz |

---

## 🧪 Tests

La suite de tests couvre ~1 200 lignes. Pour exécuter les tests individuellement :

```bash
# Validation du solveur IK
ros2 run ur5_pick_place test_ik_solver

# Contrôle du préhenseur
ros2 run ur5_pick_place test_gripper

# États et limites des joints
ros2 run ur5_pick_place test_joints

# Pipeline MoveIt2 complet
ros2 run ur5_pick_place test_moveit_full_pipeline

# Gestion de la scène MoveIt2
ros2 run ur5_pick_place test_moveit_scene

# Test niveaux MoveIt2
ros2 run ur5_pick_place test_moveit_levels
```

### Validation rapide du système

```bash
python3 quick_validation.py
```

---

## 🔧 Commandes utilitaires

```bash
# Publier une pose simulée (pour tests sans caméra)
ros2 run ur5_pick_place fake_object_pose

# Ajouter les objets dans la planning scene
ros2 run ur5_pick_place add_object

# Diagnostics système MoveIt2
ros2 run ur5_pick_place moveit_diagnostics

# Démo trajectoires cartésiennes
ros2 run ur5_pick_place moveit_cartesian_demo
```

---

## 🐛 Dépannage

### Le mouvement de descente échoue (`min_fraction < 0.98`)

> Vérifier que la scène MoveIt2 est correctement initialisée (table et objet présents). Essayer de réduire `max_step` dans la configuration cartésienne.

### Deadlock lors du `spin_until_future_complete()`

> S'assurer que le nœud utilise bien un `MultiThreadedExecutor` avec au moins 4 workers.

### IK ne converge pas

> Le solveur KDL tente 3 seeds différentes (HOME, neutre, proche-pick). Vérifier que la cible est dans l'enveloppe de travail (≤ 0.85 m du centre de la base).

### Gripper ne répond pas

> Vérifier que `hand_controller` est actif : `ros2 control list_controllers`

---

## 🗺️ Roadmap

- [x] Séquence pick & place complète (8 étapes)
- [x] IK analytique + variante robuste multi-seed
- [x] Géométrie wrist_3 ↔ tool0 correctement dérivée de l'URDF
- [x] Saisie verticale par le haut opérationnelle
- [x] Suite de tests complète
- [ ] **Finaliser `object_detector_node.py`** — intégration caméra réelle
- [ ] **Calibration adaptative du gripper** — fermeture selon la largeur réelle de l'objet
- [ ] **Retry cartésien adaptatif** — réessai avec `max_step` réduit si fraction < 0.98
- [ ] **Déploiement robot réel** — remplacement de `mock_components` par `ur_robot_driver`
- [ ] **Intégration `hand_arm_controller.py`** dans le flux principal
- [ ] **Squash des commits historiques** — nettoyage des messages non descriptifs

---

## 📁 Structure du projet

```
UR5_Jazzy_Moveit2_Pick_Place/
├── src/
│   ├── ur5_pick_place/                    ← Package principal
│   │   ├── ur5_pick_place/                ← Modules Python
│   │   │   ├── pick_place_node.py         ← Orchestrateur ⭐
│   │   │   ├── base_pick_place.py         ← Classe de base trajectoires
│   │   │   ├── ik_solver.py               ← IK analytique
│   │   │   ├── ik_solver_robust.py        ← IK multi-seed
│   │   │   ├── gripper_controller.py      ← Contrôle préhenseur
│   │   │   ├── config_loader.py           ← Chargeur YAML Singleton
│   │   │   ├── validators.py              ← Validation joints/workspace
│   │   │   ├── pick_place_state_machine.py← Machine à états
│   │   │   └── test_*.py                  ← Suite de tests (8 fichiers)
│   │   └── launch/
│   │       ├── pick_place.launch.py       ← Lancement principal ⭐
│   │       └── pick_place_moveit2.launch.py← Lancement unifié ⭐
│   └── ur5_ws/
│       └── src/
│           ├── ur5_description/           ← URDF, meshes, monde Gazebo
│           ├── ur5_controller/            ← Configuration ros2_control
│           └── ur5_moveit/                ← Config MoveIt2 + OMPL + KDL
├── config/
│   └── ur5_pick_place_config.yaml
├── scripts/
│   └── launch_pick_place_complete.sh
└── quick_validation.py
```

---

## 🤝 Contribuer

Les contributions sont les bienvenues ! Merci de :

1. Forker le dépôt
2. Créer une branche feature : `git checkout -b feature/ma-fonctionnalite`
3. Commiter avec un message descriptif
4. Ouvrir une Pull Request

---

## 📄 Licence

Ce projet est sous licence **MIT** — voir le fichier [LICENSE](LICENSE) pour les détails.

---

<div align="center">

Développé par NGARTOUDJINA ABEL pour la robotique open source

</div>
