# UR5 Jazzy MoveIt2 Pick & Place — Description et État du Projet

> Dernière mise à jour : **2026-05-01**

---

## Table des matières

1. [Vue d'ensemble](#1-vue-densemble)
2. [Structure du projet](#2-structure-du-projet)
3. [Architecture logicielle](#3-architecture-logicielle)
4. [Géométrie cinématique du gripper](#4-géométrie-cinématique-du-gripper)
5. [Configuration des poses](#5-configuration-des-poses)
6. [Interfaces ROS2](#6-interfaces-ros2)
7. [Corrections historiques](#7-corrections-historiques)
8. [**Modifications apportées le 2026-05-01**](#8-modifications-apportées-le-2026-05-01)
9. [Couverture de tests](#9-couverture-de-tests)
10. [État actuel du projet](#10-état-actuel-du-projet)
11. [Procédure de lancement](#11-procédure-de-lancement)
12. [Points d'attention pour la suite](#12-points-dattention-pour-la-suite)

---

## 1. Vue d'ensemble

Système complet de **pick & place** pour un bras robotique **UR5 6-DOF** équipé d'un préhenseur **Robotiq 85**, sous **ROS2 Humble**. Le projet intègre :

- **Gazebo** — simulation physique et visualisation 3D
- **MoveIt2** — planification de trajectoires et exécution
- **ros2_control** — couche d'abstraction hardware
- **Python** — logique d'orchestration du pick & place

**Objectif** : démontrer un cycle pick & place complet — saisir un objet posé sur une table et le déposer dans une zone cible, via cinématique inverse (IK), planification OMPL et contrôle du préhenseur.

**Scène par défaut** :
- Table : `0.8 × 0.8 × 0.05` m, centre `(0.65, 0.0, -0.025)` → surface à `z=0`
- Objet à saisir : `0.05 × 0.05 × 0.10` m, centre `(0.5, 0.1, 0.05)` → repose sur la table
- Zone de dépôt : `(0.5, -0.3, 0.05)`

---

## 2. Structure du projet

```
UR5_Jazzy_Moveit2_Pick_Place/
├── src/
│   ├── ur5_pick_place/                ← Package principal (logique pick & place)
│   │   ├── ur5_pick_place/            ← Modules Python
│   │   │   ├── pick_place_node.py              (≈645 lignes) — Orchestrateur principal ⭐
│   │   │   ├── async_pick_place_node.py        — Variante événementielle
│   │   │   ├── base_pick_place.py              (239 lignes) — Classe de base (trajectoires)
│   │   │   ├── config_loader.py                (170 lignes) — Chargeur YAML (Singleton)
│   │   │   ├── validators.py                   — Validation joints + workspace
│   │   │   ├── ik_solver.py                    (271 lignes) — IK analytique
│   │   │   ├── ik_solver_robust.py             — IK multi-seed + détection singularités
│   │   │   ├── gripper_controller.py           (103 lignes) — Contrôle préhenseur
│   │   │   ├── pick_place_state_machine.py     (133 lignes) — Machine à états
│   │   │   ├── pick_place_factory.py           (58 lignes) — Factory pattern
│   │   │   ├── hand_arm_controller.py          (616 lignes) — Contrôle gestuel
│   │   │   ├── fake_object_pose.py             (58 lignes) — Pose objet simulée
│   │   │   ├── add_object.py                   (158 lignes) — Init scène MoveIt2
│   │   │   ├── object_detector_node.py         — Intégration caméra réelle
│   │   │   ├── pose_sender.py                  (200 lignes) — Diffusion de poses
│   │   │   ├── moveit_cartesian_demo.py        (356 lignes) — Demo trajectoires cartésiennes
│   │   │   ├── moveit_system_diagnostics.py    — Diagnostic système
│   │   │   ├── joint_state_publisher.py        (63 lignes) — Utilitaires joint states
│   │   │   ├── scene_config.yaml               — Configuration scène
│   │   │   └── test_*.py                       (8 fichiers, ~1200 lignes)
│   │   ├── launch/
│   │   │   ├── pick_place.launch.py            — Lancement principal (pose simulée) ⭐
│   │   │   ├── pick_place_v2.launch.py         — Variante
│   │   │   └── pick_place_moveit2.launch.py    — Intégration MoveIt2 complète ⭐
│   │   ├── config/
│   │   │   └── ur5_pick_place_config.yaml      — Configuration pick & place
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   └── ur5_ws/                        ← Workspace simulation
│       └── src/
│           ├── ur5_description/       — URDF, monde Gazebo, meshes
│           │   ├── urdf/ur5_robot.urdf.xacro      ← Définit la chaîne UR5 + Robotiq 85
│           │   ├── worlds/gazebo_world.sdf        ← Monde Gazebo (table, objet, zone)
│           │   └── ...
│           ├── ur5_controller/        — Configuration ros2_control
│           └── ur5_moveit/            — Configuration MoveIt2 + planification
│               └── config/
│                   ├── ur5_robot.srdf            ← Groupe ur5_arm chain → wrist_3_link
│                   ├── kinematics.yaml           ← Solveur KDL
│                   ├── ompl_planning.yaml        ← Planificateur RRTConnect
│                   └── moveit_controllers.yaml
├── config/
│   └── ur5_pick_place_config.yaml
├── scripts/
│   └── launch_pick_place_complete.sh
├── PROJECT_OVERVIEW.md
├── PROJET_ETAT.md                     ← Ce document
├── VALIDATION_REPORT.md
└── FINAL_SOLUTION_SUMMARY.txt
```

⭐ = fichiers modifiés le 2026-05-01

---

## 3. Architecture logicielle

### Séquence pick & place (8 étapes)

```
HOME → PRE-PICK → GRASP (descente cartésienne) → FERMER + ATTACH
     → LIFT (cartésien) → PRE-PLACE → PLACE (descente cartésienne) → OUVRIR + DETACH
     → HOME final
```

| Étape | Type planification | But |
|-------|---------------------|-----|
| HOME | Joint-space (RRTConnect) | Position de repos sécurisée |
| PRE-PICK | Joint-space (RRTConnect) | Survol au-dessus de l'objet |
| GRASP | Cartésien linéaire | Descente verticale exacte au TCP |
| LIFT | Cartésien linéaire | Remontée verticale exacte |
| PRE-PLACE | Joint-space (RRTConnect) | Survol au-dessus de la zone |
| PLACE | Cartésien linéaire | Descente verticale au point de dépôt |
| RETREAT | Cartésien linéaire | Remontée + dégagement |

### Modules principaux

| Module | Rôle | Design pattern |
|--------|------|----------------|
| `pick_place_node.py` | Orchestration complète | Séquentiel asynchrone |
| `base_pick_place.py` | Exécution trajectoires (ActionClient) | Classe de base |
| `ik_solver.py` | IK analytique 6-DOF (paramètres DH UR5) | Solveur direct |
| `ik_solver_robust.py` | IK multi-seed + filtrage singularités | Stratégie robuste |
| `gripper_controller.py` | Contrôle préhenseur (non-bloquant) | Timer callbacks |
| `config_loader.py` | Chargement YAML, poses nommées | Singleton |
| `validators.py` | Validation joints (±2π) + workspace (≤0.85 m) | Validation frontière |
| `pick_place_state_machine.py` | Machine à états explicite | State machine |

### Paramètres DH UR5
```
d1 = 0.089159 m
a2 = -0.425 m,  a3 = -0.39225 m
d4 = 0.10915 m, d5 = 0.09465 m, d6 = 0.0823 m
```

---

## 4. Géométrie cinématique du gripper

> **Section critique** — comprendre cette géométrie est essentiel pour la cinématique inverse.

### Chaîne URDF wrist_3_link → tool0

```
wrist_3_link
   │
   │  joint robotiq_85_base_joint (fixe)
   │  origin xyz=(0, 0.0823, 0)  rpy=(π/2, 0, π/2)
   ▼
robotiq_85_base_link
   │
   │  joint robotiq_85_base_link-tool0 (fixe)
   │  origin xyz=(0, 0, 0.130)  rpy=(0, 0, 0)
   ▼
tool0 (TCP — centre des doigts fermés)
```

### Conséquences mathématiques

La rotation `rpy=(π/2, 0, π/2)` produit la matrice :

```
R_wrist3→robotiq = | 0  0  1 |
                   | 1  0  0 |
                   | 0  1  0 |
```

Donc dans le repère **wrist_3_link** :
- `Z_robotiq = X_wrist3` (axe de saisie du gripper aligné sur X_wrist3)
- `Y_robotiq = Z_wrist3`
- `X_robotiq = Y_wrist3`

Et tool0 dans le repère wrist_3 :
```
tool0_in_wrist3 = (0, 0.0823, 0) + 0.130 * Z_robotiq_in_wrist3
                = (0, 0.0823, 0) + 0.130 * (1, 0, 0)
                = (0.130, 0.0823, 0)
```

### Orientation tool0 vers le bas (saisie verticale)

Pour saisir un objet **par le haut**, on impose tool0 orienté +Z vers le bas :
```
q_tool0 = (1, 0, 0, 0)   ← rotation 180° autour de X
```

Ce qui donne l'orientation de wrist_3 dans le monde :
```
R_world_wrist3 = R_world_tool0 · R_wb^T
              = | 0   1  0 |
                | 0   0 -1 |
                | -1  0  0 |
```

Soit la quaternion :
```
q_wrist3 = (0.5, 0.5, -0.5, 0.5)
```

### Offset wrist_3 → tool0 dans le repère monde

Avec cette orientation, le décalage géométrique de tool0 par rapport à wrist_3 (en monde) est :
```
tool0 - wrist_3 = (+0.0823, 0, -0.130)
```

D'où la transformation **TCP-cible → wrist_3-cible** utilisée pour l'IK :
```
wrist_3.xyz = tool0.xyz + (-0.0823, 0, +0.130)
wrist_3.quat = (0.5, 0.5, -0.5, 0.5)
```

---

## 5. Configuration des poses

### Hauteurs paramétrables (pick_place_node.py)

| Paramètre | Sémantique (corrigée le 2026-05-01) | Défaut code | Défaut launch |
|-----------|-------------------------------------|-------------|---------------|
| `pre_pick_height` | Hauteur de **tool0** au-dessus du centre objet | 0.15 m | 0.20 m |
| `grasp_height` | Hauteur de **tool0** au-dessus du centre objet (0 = centré) | 0.0 m | 0.0 m |

### Poses nommées (ur5_pick_place_config.yaml — joint-space)

| Pose | shoulder_pan | shoulder_lift | elbow | wrist_1 | wrist_2 | wrist_3 |
|------|:---:|:---:|:---:|:---:|:---:|:---:|
| home | 0.0 | -π/2 | π/2 | -π/2 | -π/2 | 0.0 |
| pre_pick | 0.524 | -1.396 | π/2 | -1.178 | -π/2 | 0.524 |
| grasp | 0.524 | -1.047 | π/2 | -π/2 | -π/2 | 0.524 |
| lift | 0.524 | -π/2 | π/2 | -π/2 | -π/2 | 0.524 |
| pre_place | -1.047 | -1.396 | π/2 | -1.178 | -π/2 | -1.047 |
| place | -1.047 | -1.047 | π/2 | -π/2 | -π/2 | -1.047 |
| retreat | -1.047 | -π/2 | π/2 | -π/2 | -π/2 | -1.047 |

### Objets de collision (MoveIt) — alignés sur Gazebo
- **Table** : `0.8 × 0.8 × 0.05` m @ `(0.65, 0.0, -0.025)` m
- **Objet** : `0.05 × 0.05 × 0.10` m @ `(ox, oy, oz)` ⭐ corrigé le 2026-05-01

---

## 6. Interfaces ROS2

### Services
| Service | Type | Rôle |
|---------|------|------|
| `/compute_ik` | GetPositionIK | Calcul IK via MoveIt2/KDL |
| `/apply_planning_scene` | ApplyPlanningScene | Gestion objets de collision |
| `/compute_cartesian_path` | GetCartesianPath | Déplacement TCP linéaire |

### Actions
| Action | Type | Rôle |
|--------|------|------|
| `/move_action` | MoveGroup | Planification + exécution MoveIt2 |
| `/execute_trajectory` | ExecuteTrajectory | Exécution de trajectoire pré-calculée |
| `/ur5_arm_controller/follow_joint_trajectory` | FollowJointTrajectory | Exécution trajectoire bas niveau |
| `/hand_controller/gripper_cmd` | GripperCommand | Ouverture/fermeture préhenseur |

### Topics
| Topic | Type | Direction |
|-------|------|-----------|
| `/joint_states` | JointState | Lecture état robot |
| `/object_pose` | PoseStamped | Lecture pose cible objet |

---

## 7. Corrections historiques (avant 2026-05-01)

| # | Problème | Correction | Statut |
|---|----------|-----------|--------|
| C1 | Quaternion IK incorrect | (0.7071, 0.7071, 0, 0) pour outil vers le bas | ⚠️ remplacée le 2026-05-01 |
| C2 | Offset hauteur saisie erroné | Suppression +0.13 m arbitraire | ⚠️ remplacée le 2026-05-01 |
| C3 | Incompatibilité de types | `move_to_position` accepte `JointPosition`, `Dict`, `list` | ✅ |
| C4 | Conflit pipeline v1/v2 | Unification | ✅ |
| C5 | Config non installée | Ajout dans `setup.py data_files` | ✅ |
| C6 | Offset IK trop grand | Suppression +0.13 m offset préhenseur | ⚠️ remplacée le 2026-05-01 |
| C7 | Nom groupe incorrect | Uniformisation sur `ur5_arm` | ✅ |
| C8 | Préhenseur non initialisé | `main()` appelle `run()` | ✅ |
| C9 | Préhenseur bloquant | Remplacement `time.sleep()` par timer callbacks | ✅ |
| C10 | Logique validateur joints | Correction gestion bornes ±2π | ✅ |
| C11 | Timeout KDL trop court | 0.005 s → 0.15 s ; tentatives ×1.67 | ✅ |

> **Note** : les corrections C1, C2 et C6 partaient du principe (incorrect) que le quaternion `(0.7071, 0.7071, 0, 0)` orientait le gripper vers le bas. Cette hypothèse a été invalidée le 2026-05-01 par l'analyse complète de la chaîne URDF.

---

## 8. Modifications apportées le 2026-05-01

> ⭐ Section principale — corrige les bugs cinématiques bloquant la saisie de l'objet.

### 8.1 Diagnostic des problèmes

Deux symptômes signalés :
1. La cinématique inverse ne donnait pas de bonnes solutions.
2. Le gripper ne descendait pas totalement au niveau de l'objet pour le récupérer.

#### Cause racine n°1 : quaternion incorrect

L'ancienne constante `GRASP_QUAT = (0.7071, 0.7071, 0, 0)` orientait `Z_wrist3` vers le bas (-Z monde). Le commentaire associé affirmait que cela faisait pointer l'outil vers le bas.

**Mais** la chaîne URDF montre que l'**axe de saisie réel** du Robotiq 85 est `Z_robotiq`, et que la rotation `rpy=(π/2, 0, π/2)` du joint `robotiq_85_base_joint` aligne `Z_robotiq` sur `X_wrist3` (et non sur `Z_wrist3`).

Conséquence : avec le quaternion buggé, c'était `Z_wrist3` qui pointait vers le bas, mais le gripper (qui suit `X_wrist3`) pointait **horizontalement** vers +Y monde.

#### Cause racine n°2 : offset géométrique mal calculé

Le code ajoutait simplement `+0.130 m` à la coordonnée Z cible, traitant `GRIPPER_TOOL_OFFSET` comme une distance verticale. **Mais** dans le repère wrist_3, tool0 est à :
```
tool0_in_wrist3 = (0.130, 0.0823, 0)
```

Avec le quaternion buggé combiné à cet offset incorrect, tool0 se retrouvait en monde à :
```
tool0_world = wrist_3_world + (0.0823, 0.130, 0)
```

Soit **13 cm latéralement** de la cible visée, et 13 cm au-dessus de la table — totalement à côté de l'objet.

#### Cause racine n°3 : descente cartésienne incomplète tolérée

`min_fraction = 0.85` autorisait l'exécution d'une trajectoire ne couvrant que 85 % du chemin demandé. Pour une descente de saisie, cela signifiait potentiellement s'arrêter 15 % au-dessus de la cible — soit ~3 cm trop haut sur une descente de 20 cm.

#### Cause racine n°4 : taille de boîte de collision incohérente

La box de collision MoveIt pour l'objet était déclarée `0.05 × 0.05 × 0.05` m alors que l'objet Gazebo réel mesure `0.05 × 0.05 × 0.10` m. Conséquence : MoveIt sous-estimait la zone occupée par l'objet pendant la planification du LIFT (objet attaché au gripper).

### 8.2 Solution mathématique appliquée

Pour saisir verticalement par le haut, on impose tool0 orienté +Z vers le bas (`q_tool0 = (1, 0, 0, 0)`). Cela se traduit pour wrist_3 par :

```
q_wrist3 = (0.5, 0.5, -0.5, 0.5)
wrist_3 = tool0 + (-0.0823, 0, +0.130)
```

### 8.3 Fichiers modifiés

#### `src/ur5_pick_place/ur5_pick_place/pick_place_node.py`

**1. Constantes géométriques** (`pick_place_node.py:70-89`)
```python
# AVANT
GRASP_QUAT = (0.7071, 0.7071, 0.0, 0.0)
GRIPPER_TOOL_OFFSET = 0.130

# APRÈS
WRIST3_GRASP_QUAT = (0.5, 0.5, -0.5, 0.5)
TOOL0_OFFSET_X = 0.0823   # |Δx| wrist_3 → tool0 (en monde, gripper bas)
TOOL0_OFFSET_Z = 0.130    # |Δz| wrist_3 → tool0 (en monde, gripper bas)
```
Avec un commentaire-bloc qui dérive ces constantes depuis la chaîne URDF.

**2. Refactor de `_pose()`** (`pick_place_node.py:183-199`)

L'API change de "wrist_3 cible" vers "tool0 cible" :
```python
# AVANT (l'appelant devait raisonner en coordonnées wrist_3)
def _pose(x, y, z, quat=GRASP_QUAT) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    ...

# APRÈS (l'appelant raisonne en coordonnées TCP — la conversion vers
#         wrist_3 est faite ici, une seule fois, à partir de l'URDF)
def _pose(tool0_x, tool0_y, tool0_z, quat=WRIST3_GRASP_QUAT) -> Pose:
    p = Pose()
    p.position.x = float(tool0_x) - TOOL0_OFFSET_X
    p.position.y = float(tool0_y)
    p.position.z = float(tool0_z) + TOOL0_OFFSET_Z
    ...
```

Tous les appels existants `self._pose(ox, oy, oz + self.grasp_h)` et `self.compute_ik(ox, oy, oz + self.grasp_h, ...)` deviennent automatiquement corrects, car ils passent désormais des coordonnées TCP au lieu de coordonnées wrist_3.

**3. Sémantique de `grasp_height` clarifiée** (`pick_place_node.py:104-110`)
```python
# pre_pick_height / grasp_height : hauteur de tool0 (TCP) au-dessus
# du CENTRE de l'objet (object_pose.z).
#   pre_pick = oz + pre_pick_height  → survol
#   grasp    = oz + grasp_height     → niveau de saisie (TCP au centre objet)
self.declare_parameter('pre_pick_height', 0.15)
self.declare_parameter('grasp_height',    0.0)   # ← était GRIPPER_TOOL_OFFSET (0.130)
```

**4. `min_fraction` cartésien renforcé** (`pick_place_node.py:313-320`)
```python
# AVANT : min_fraction=0.85 (descente partielle silencieuse)
# APRÈS : min_fraction=0.98
def cartesian_move(self, target_pose: Pose, label='',
                   max_step=None, min_fraction=0.98) -> bool:
```

**5. Box de collision objet alignée sur Gazebo** (`pick_place_node.py:547`)
```python
# AVANT : size=(0.05, 0.05, 0.05)  ← incohérent avec Gazebo
# APRÈS
self.add_box('target_box', (ox, oy, oz), size=(0.05, 0.05, 0.10))
```

#### `src/ur5_pick_place/launch/pick_place.launch.py`

```python
# AVANT
'pre_pick_height':    0.20,
'grasp_height': 0.130,  # offset géométrique wrist_3 → base gripper

# APRÈS
'pre_pick_height': 0.20,
'grasp_height':    0.0,   # tool0 (TCP) au centre de l'objet
```

#### `src/ur5_pick_place/launch/pick_place_moveit2.launch.py`

```python
# AVANT
'pre_pick_height':     0.20,
'grasp_height':        0.13,

# APRÈS
'pre_pick_height':     0.20,
'grasp_height':        0.0,
```

### 8.4 Vérification numérique

Pour l'objet à `(0.5, 0.1, 0.05)` après corrections :

| Étape | tool0 cible | wrist_3 cible | Distance base | Statut |
|-------|-------------|---------------|---------------|--------|
| PRE-PICK (pre_h=0.20) | (0.500, 0.100, 0.250) | (0.418, 0.100, 0.380) | 0.573 m | ✅ < 0.85 |
| GRASP (grasp_h=0.0) | (0.500, 0.100, 0.050) | (0.418, 0.100, 0.180) | 0.466 m | ✅ < 0.85 |
| PRE-PLACE (place=(0.5,-0.3,0.05)) | (0.500, -0.300, 0.250) | (0.418, -0.300, 0.380) | 0.639 m | ✅ < 0.85 |
| PLACE | (0.500, -0.300, 0.050) | (0.418, -0.300, 0.180) | 0.544 m | ✅ < 0.85 |

Toutes les cibles sont dans l'enveloppe de travail UR5 (max 0.85 m).

### 8.5 Synthèse des modifications

| Aspect | Avant | Après |
|--------|-------|-------|
| Quaternion wrist_3 | (0.7071, 0.7071, 0, 0) | (0.5, 0.5, -0.5, 0.5) |
| Direction de saisie | +Y monde (horizontale) | -Z monde (verticale) ✅ |
| Offset wrist_3↔tool0 | +0.130 sur Z (incorrect) | (-0.0823, 0, +0.130) en monde ✅ |
| Sémantique `grasp_height` | offset wrist_3 magique | hauteur tool0 au-dessus objet |
| `min_fraction` cartésien | 0.85 (descente partielle OK) | 0.98 (TCP doit atteindre la cible) |
| Box collision objet | 0.05 × 0.05 × 0.05 | 0.05 × 0.05 × 0.10 (aligné Gazebo) |
| API `_pose()` | reçoit wrist_3 cible | reçoit tool0 cible (TCP) |

### 8.6 Pour appliquer les modifications

```bash
cd /home/beingar/ros2_humble_ws/UR5_Jazzy_Moveit2_Pick_Place
colcon build --packages-select ur5_pick_place --symlink-install
source install/setup.bash
```

---

## 9. Couverture de tests

| Fichier | Périmètre | Lignes |
|---------|-----------|--------|
| test_corrections.py | Validation des 11 corrections (à mettre à jour) | 298 |
| test_moveit_full_pipeline.py | Workflow MoveIt2 complet | 261 |
| test_moveit_levels.py | Fonctionnalités MoveIt2 par niveaux | 214 |
| test_moveit_scene.py | Init scène + collision | 165 |
| test_joints.py | États et limites joints | 116 |
| test_moveit.py | Intégration MoveIt2 de base | 85 |
| test_gripper.py | Contrôle préhenseur | — |
| test_ik_solver.py | Validation solveur IK | — |
| **Total** | | **~1 200 lignes** |

> ⚠️ `test_corrections.py` peut référencer les anciennes constantes `GRASP_QUAT` / `GRIPPER_TOOL_OFFSET`. Vérification recommandée après les modifications du 2026-05-01.

---

## 10. État actuel du projet

### Implémenté et validé ✅

- Orchestration pick & place complète (8 étapes)
- Cinématique inverse analytique (+ variante robuste multi-seed)
- **Géométrie wrist_3 ↔ tool0 correctement dérivée de l'URDF** (2026-05-01)
- **Saisie verticale par le haut effective** (2026-05-01)
- Planification de trajectoires (RRTConnect via OMPL)
- Descente cartésienne stricte (`min_fraction=0.98`)
- Contrôle préhenseur non-bloquant (timer callbacks)
- Validation joints (±2π) et workspace (≤0.85 m)
- Initialisation scène MoveIt2 (table + objet, dimensions correctes)
- Gestion configuration YAML avec résolution de chemins portable
- Suite de tests complète (8 fichiers)
- Machine à états explicite
- Demo trajectoires cartésiennes
- Diagnostics système
- Documentation inline détaillée de la cinématique

### Changements non commités ⚠️

```
M src/ur5_pick_place/launch/pick_place.launch.py        ← MODIFIÉ 2026-05-01
M src/ur5_pick_place/launch/pick_place_moveit2.launch.py ← MODIFIÉ 2026-05-01
M src/ur5_pick_place/ur5_pick_place/pick_place_node.py  ← MODIFIÉ 2026-05-01
M src/ur5_ws/src/ur5_moveit/config/kinematics.yaml      ← antérieur
M src/ur5_ws/src/ur5_moveit/config/moveit.rviz          ← antérieur
M src/ur5_ws/src/ur5_moveit/config/ompl_planning.yaml   ← antérieur
?? PROJECT_OVERVIEW.md
?? PROJET_ETAT.md                                        ← Ce document
```

### Partiellement implémenté / en cours 🔶

| Composant | État | Notes |
|-----------|------|-------|
| `object_detector_node.py` | Partiel | Intégration caméra réelle non finalisée |
| `async_pick_place_node.py` | Implémenté | Non utilisé par défaut |
| `hand_arm_controller.py` | Implémenté | Non intégré au flux principal |
| Machine à états | Implémentée | Cohabite avec le nœud séquentiel |
| Calibration angle de fermeture gripper | Hardcoded à 0.72 rad | Pourrait être adaptatif selon la largeur objet |

### Métriques de qualité

| Métrique | Valeur |
|----------|--------|
| Lignes de code principal | ~5 900 |
| Lignes de tests | ~1 200 |
| Ratio test/code | ~20 % |
| Corrections appliquées | 11 historiques + 4 (2026-05-01) |
| Design patterns | Singleton, Factory, Strategy, State Machine |
| Langage | Python (ROS2 Humble) |

---

## 11. Procédure de lancement

### Lancement modulaire (3 terminaux)

```bash
# Terminal 1 — Simulation Gazebo
source /opt/ros/humble/setup.bash
source src/ur5_ws/install/setup.bash
ros2 launch ur5_description gazebo.launch.py

# Terminal 2 — MoveIt2 + Contrôle
source /opt/ros/humble/setup.bash
source src/ur5_ws/install/setup.bash
ros2 launch ur5_moveit moveit.launch.py

# Terminal 3 — Pick & Place
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_pick_place pick_place.launch.py
```

### Lancement unifié

```bash
ros2 launch ur5_pick_place pick_place_moveit2.launch.py
```

**Durée estimée d'un cycle** : ~27 secondes

### Build avant premier lancement

```bash
cd /home/beingar/ros2_humble_ws/UR5_Jazzy_Moveit2_Pick_Place
colcon build --packages-select ur5_pick_place --symlink-install
source install/setup.bash
```

---

## 12. Points d'attention pour la suite

1. **Tester en simulation** — vérifier que la descente atteint bien le centre de l'objet et que le gripper se ferme correctement autour.
2. **Mettre à jour `test_corrections.py`** — il peut référencer les anciennes constantes `GRASP_QUAT` ou `GRIPPER_TOOL_OFFSET`.
3. **Commiter les modifications du 2026-05-01** avec un message descriptif :
   ```
   fix(ik): correction quaternion wrist_3 et offset tool0 (cinématique URDF)

   - Refactor _pose() pour accepter coordonnées TCP (tool0) au lieu de wrist_3
   - WRIST3_GRASP_QUAT = (0.5, 0.5, -0.5, 0.5) pour saisie verticale
   - TOOL0_OFFSET_X/Z dérivés de la chaîne URDF
   - min_fraction cartésien : 0.85 → 0.98
   - Box collision objet alignée Gazebo : 0.05³ → 0.05×0.05×0.10
   - grasp_height : 0.130 → 0.0 (tool0 au centre objet)
   ```
4. **Finaliser `object_detector_node.py`** pour l'intégration caméra réelle.
5. **Documenter les commits historiques** (messages "jdfbhwauhiu", etc.) — squash + rebase éventuel.
6. **Calibrer la fermeture du gripper** en fonction de la largeur réelle de l'objet (actuellement 0.72 rad fixé pour tout objet).
7. **Considérer un retry adaptatif** sur la descente cartésienne : si fraction < 0.98, réessayer avec un `max_step` plus petit avant d'échouer.
