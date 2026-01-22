# Projet Robotique - SLAM et Navigation Autonome

Ce dépôt contient un workspace ROS2 pour un projet robotique intégrant SLAM (Simultaneous Localization and Mapping), navigation autonome, et modélisation du robot via URDF. Le projet utilise des algorithmes avancés comme Fast-LIO2 et LIO-SAM pour le SLAM, Nav2 pour la navigation.

## Structure du Projet

```
robot/
│
├── README.md                          # Documentation principale
├── LICENSE                            # Licence MIT
├── requirements.txt                   # Dépendances Python
│
├── docs/                              # Documentation détaillée
│   ├── architecture.md                # Architecture du système
│   ├── installation.md                # Guide d'installation complet
│   └── user_guide.md                  # Guide d'utilisation
│
├── src/                               # Code source et configurations
│   ├── slam/                          # Modules SLAM
│   │   ├── fast_lio2/                 # Configuration Fast-LIO2
│   │   └── lio_sam/                   # Configuration LIO-SAM
│   │
│   ├── navigation/                    # Module de navigation
│   │   └── nav2_configs/              # Configurations Nav2
│   │       └── nav2_params.yaml       # Paramètres Nav2
│   │
│   └── robot_description/             # Modélisation du robot
│       ├── urdf/                      # Fichiers URDF/XACRO
│       ├── meshes/                    # Modèles 3D
│       ├── rviz/                      # Configurations RViz
│       ├── launch/                    # Fichiers de lancement Gazebo/RViz
│       ├── config/                    # Configurations contrôleurs
│       └── CMakeLists.txt             # Build ROS2
│
│   └── web_controller/                # Contrôleur web et évaluation
│       ├── web_controller_server.py  # Serveur web
│       ├── web_controller.html        # Interface web
│       ├── run_kitti_eval.py          # Évaluation KITTI
│       └── scripts/                   # Scripts utilitaires
│
├── config/                            # Fichiers de configuration globaux
│   ├── slam_params.yaml               # Paramètres SLAM
│   ├── nav2_params.yaml               # Paramètres Nav2
│   └── sensor_configs.yaml            # Configurations capteurs
│
├── scripts/                           # Scripts utilitaires
│   ├── setup_environment.sh           # Configuration environnement
│   ├── run_simulation.sh              # Lancement simulation
│   └── evaluation/                    # Évaluation et métriques
│       ├── evo_analysis.py            # Analyse avec EVO
│       ├── run_evo.sh                 # Script EVO
│       └── README.md                  # Guide évaluation
│
├── launch/                            # Fichiers de lancement ROS2
│   ├── slam_launch.py                 # Lancement SLAM
│   ├── navigation_launch.py           # Lancement navigation
│   └── full_system_launch.py          # Lancement système complet
│
│
├── results/                           # Résultats expérimentaux
│   ├── metrics/                       # Métriques APE/RPE
│   ├── trajectories/                  # Trajectoires enregistrées
│   └── plots/                         # Graphiques et visualisations
│
└── tests/                             # Tests unitaires
    └── test_slam.py                   # Tests SLAM
```

## Prérequis

- **OS** : Ubuntu 22.04 LTS
- **ROS2** : Humble Hawksbill
- **Python** : 3.10+
- **Dépendances** : Voir `requirements.txt` et `docs/installation.md`

## Installation Rapide

1. **Cloner le dépôt** :
   ```bash
   git clone https://github.com/Ilhemmechi/robot.git
   cd robot
   ```

2. **Installer ROS2 Humble** :
   Suivez le guide officiel ou consultez `docs/installation.md`

3. **Configurer l'environnement** :
   ```bash
   ./scripts/setup_environment.sh
   ```

4. **Installer les dépendances Python** :
   ```bash
   pip install -r requirements.txt
   ```

5. **Construire le workspace** :
   ```bash
   colcon build
   source install/setup.bash
   ```

## Utilisation
Contenu principal

- `src/` : sources et ressources principales fournies.
  - `fastlio_maps/` : cartes et nuages de points générés par un pipeline LIO/SLAM (Fast-LIO 2).
    - `src/.../FAST_LIO/` : sources de Fast-LIO 2 présentes dans l'arborescence (`FAST_LIO` package).
      - `src/.../FAST_LIO/config/velodyne.yaml` ou `velodyne_gazebo.yaml` : fichiers de configuration LiDAR utilisés par Fast-LIO 2 (ex : extrinsèques, modèle de capteur, topics).
  - `lidar_detector/` : package Python ROS (launch, nodes, ressources, tests) pour détection LiDAR.
  - `LIO-SAM/` : source et configuration pour LIO-SAM (SLAM) avec CMake/launch/config.
    - `LIO-SAM/config/params.yaml` : fichier de configuration principal pour LIO-SAM (présent dans le dossier `config`).
  - `mon_urdf/` : URDF/xacro et fichiers RViz pour visualisation du robot.
  - `nav2_config/` : configuration pour Nav2 (navigation).
    - `nav2_config/nav2_params.yaml` : fichier de configuration utilisé pour lio_sam / Nav2.
    `robot/confignav2.params.yaml`  `FAST_LIO/launch/*` : fichier de config pour fast_lio2

## Fichiers de configuration

- `LIO-SAM/config/params.yaml` : paramètres principaux de LIO-SAM (calibration IMU, constantes SLAM, topics, etc.).
- `nav2_config/nav2_params.yaml` : paramètres Nav2 et configuration utilisée par le pipeline lio_sam / navigation.


### Lancement de la Simulation Complète

```bash
# Terminal 1 : Environnement Gazebo
ros2 launch mon urdf gazebo.launch.py

# Terminal 2 : SLAM (Fast-LIO2)
ros2 launch fast_lio2 mapping_launch.py
ou
ros2 launch lio_sam run.launch.py ( pour lancer lio_sam)
# Terminal 3 : Navigation Nav2
ros2 launch nav2_bringup navigation_launch.py


### Évaluation des Performances

Utilisez les scripts dans `scripts/evaluation/` pour analyser les trajectoires avec EVO :

```bash
cd scripts/evaluation
./run_evo.sh
```

Voir `docs/user_guide.md` pour des instructions détaillées.

## Modules Principaux

- **SLAM** : Fast-LIO2 et LIO-SAM pour cartographie et localisation simultanées
- **Navigation** : Nav2 pour planification de trajectoire et contrôle
- **Robot Description** : Modèle URDF/XACRO avec configurations Gazebo et RViz
- **Web Controller** : Interface web pour contrôle et évaluation des trajectoires


Adaptez ces fichiers selon votre matériel avant utilisation.

