# Pipeline LiDAR SLAM pour Navigation Robotique Autonome

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/) [![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/) [![C++](https://img.shields.io/badge/C++-17-orange)](https://isocpp.org/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Gazebo](https://img.shields.io/badge/Gazebo-Classic-red)](http://gazebosim.org/)

> Mémoire de Master — Systèmes Électroniques Embarqués  
> Auteure : Mechi Ilhem  
> Encadrante : Dr. Rzouga Lamia  
> Institution : Institut Supérieur des Sciences Appliquées et de Technologies (ISSAT), Sousse  
> Laboratoire : LATIS - Technologies Avancées et Systèmes Intelligents, ENISo  
> Année : 2024–2025

Sommaire
- Présentation
- Architecture & pipelines SLAM
  - Pipeline A : Fast-LIO2 (SLAM temps réel)
  - Pipeline B : LIO-SAM + AMCL (Cartographie puis localisation)
- Démonstrations et résultats
- Quand utiliser chaque pipeline
- Installation rapide
- Démarrage de la simulation
- Évaluation des performances (EVO)
- Structure du projet
- Contribuer & Licence
- Contact

---

## Présentation

Ce projet fournit un workspace ROS2 pour la cartographie et la navigation autonome basée sur LiDAR et IMU. Il inclut deux pipelines complémentaires (temps réel vs cartographie préalable), des outils d'évaluation, une interface web de contrôle et des scripts d'analyse.

<img width="866" height="522" alt="Architecture" src="https://github.com/user-attachments/assets/9a5a5fc1-a739-4b10-9ad4-febfb6ca48a0" />

---

## Architecture & pipelines SLAM

### Résumé des deux approches
Nous proposons deux stratégies pour la navigation autonome :

- Pipeline A — Fast-LIO2 : SLAM pur, temps réel, orienté exploration et navigation sans carte préalable.
- Pipeline B — LIO-SAM + AMCL : cartographie offline (ou en ligne avec optimisation), conversion vers carte 2D puis localisation probabiliste pour navigation sur carte connue.

### Pipeline A : Fast-LIO2 (SLAM Temps Réel)
Flux :
```
LiDAR + IMU → ESEKF (Filtre Kalman Itéré) → ikd-tree dynamique
       ↓
Carte 3D dense (/cloud) + Odometry (/odom)
       ↓
Navigation directe (sans carte préalable)
```

Caractéristiques :
- Avantages : exploration sans carte, haute précision (RMSE ≈ 0.017 m), faible dérive, mise à jour à ~20 Hz.
- Performances mesurées : RMSE 1.7 cm, STD 1.1 cm, latence ≈ 16 ms.
- Ressources : CPU ≈ 55%, RAM ≈ 11 GB
- Robustesse : ~94% de réussite en tests

### Pipeline B : LIO-SAM + AMCL (Cartographie puis Localisation)
Phase 1 — Cartographie :
```
LiDAR + IMU → Graph SLAM + iSAM2 → Optimisation globale
       ↓
Carte 3D optimisée (.pcd) + fermeture de boucles
       ↓
Conversion 3D → 2D (.pgm + .yaml)
```
Phase 2 — Localisation :
```
Carte statique (.pgm) → map_server
       ↓
AMCL (Monte Carlo) → Localisation probabiliste
       ↓
Navigation sur carte connue (Nav2)
```

Caractéristiques :
- Avantages : planification globale possible, stabilité (zéro dérive sur carte fixe), consommation modérée.
- Performances mesurées : RMSE ≈ 0.504 m (50.4 cm), STD 0.338 m.
- Ressources : CPU ≈ 40%, RAM ≈ 6 GB
- Contraintes : temps de cartographie initial (≈ 15–20 min), moins adapté aux environnements dynamiques.

Cartes générées :
1. Carte 3D LIO-SAM (`.pcd`) — nuage de points optimisé avec loop closure.  
2. Carte 2D Nav2 (`.pgm` + `.yaml`) — résolution typique : 5 cm/pixel.

Démonstrations (exemples) :
- Carte 3D générée : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/gif/59f480a2-45b7-433f-88e8-dfac460ea17f.png
- Carte 2D compatible Nav2 : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/gif/3bc4a791-d7f3-4d02-bd32-cd090e41ec80.png
- Interface web de contrôle : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/gif/interface_web.gif.gif
- Navigation autonome avec Fast-LIO2 : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/gif/fast_lio2_nav.gif.gif
- Navigation autonome avec LIO-SAM : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/gif/lio_sam_nav.gif.gif
- Détection d'objets (exemples) :
  - Piétons : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/gif/eb547efb-6b75-46aa-8e53-af973bd643cd.png
  - Véhicules : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/gif/1b0de6ba-cf30-48a7-9b27-9f2d01d6f140.jpg

---

## Résultats quantitatifs

Métriques SLAM (exemples) :
- Fast-LIO2
  - RMSE : 0.017 m (1.7 cm)
  - STD  : 0.011 m
  - Max  : 0.053 m  
  - Rapport : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/odom_vs_odometry-1.pdf
- LIO-SAM + AMCL
  - RMSE : 0.504 m (50.4 cm)
  - STD  : 0.338 m
  - Max  : 1.010 m  
  - Rapport : https://github.com/Ilhemmechi/Lidar-slam-navigation/blob/main/robot/results/trajectories/amcl_vs_odom-2.pdf

Conclusion : Fast-LIO2 offre une précision nettement supérieure (≈ 30× sur les jeux de tests présentés) et est recommandé pour les applications nécessitant une précision centimétrique.

Détection PV-RCNN (exemple de performance) :
| Classe       | AP Easy | AP Moderate | AP Hard |
|--------------|---------|-------------|---------|
| Car          | 91.2%   | 86.7%       | 83.5%   |
| Pedestrian   | 82.4%   | 74.9%       | 68.2%   |

Dataset d'entraînement : 2847 scènes (Gazebo). Entraînement : 80 époques (~8 h).

---

## Quand utiliser chaque pipeline ?

Comparaison synthétique :
| Critère | Fast-LIO2 | LIO-SAM + AMCL |
|---------|-----------|----------------|
| Mode opératoire | SLAM continu | Cartographie → Localisation |
| Carte requise | Non | Oui |
| Précision | ±2 cm | ±50 cm |
| Temps préparation | 0 min | 15–20 min |
| Adaptation à l’environnement | Oui (temps réel) | Non (carte statique) |
| Ressources | Élevées | Modérées |
| Fiabilité (tests) | ~94% | ~82% |

Choisir Fast-LIO2 si :
- Environnement inconnu ou non cartographié
- Besoin de précision centimétrique
- Environnement dynamique / obstacles mobiles
- Exploration autonome requise

Choisir LIO-SAM + AMCL si :
- Environnement connu et stable
- Navigation répétée sur la même zone
- Ressources matérielles limitées
- Planification globale et chemins optimisés requis

---

## Prérequis

- OS : Ubuntu 22.04 LTS  
- ROS2 : Humble Hawksbill  
- Python : 3.10+  
- Dépendances : consulter `requirements.txt` et `docs/installation.md`

---

## Installation rapide

1. Cloner le dépôt :
```bash
git clone https://github.com/Ilhemmechi/Lidar-slam-navigation.git
cd Lidar-slam-navigation/robot
```

2. Installer ROS2 Humble : suivez le guide officiel ROS2 ou `docs/installation.md`.

3. Configurer l'environnement :
```bash
./scripts/setup_environment.sh
```

4. Installer les dépendances Python :
```bash
pip install -r requirements.txt
```

5. Construire le workspace :
```bash
colcon build
source install/setup.bash
```

---

## Lancement de la simulation complète

Exemples de commandes (trois terminaux recommandés) :

Terminal 1 — Gazebo / simulation :
```bash
ros2 launch mon_urdf gazebo.launch.py
```

Terminal 2 — SLAM :
- Fast-LIO2 :
```bash
ros2 launch fast_lio2 mapping_launch.py
```
- Ou LIO-SAM :
```bash
ros2 launch lio_sam run.launch.py
```

Terminal 3 — Navigation (Nav2) :
```bash
ros2 launch nav2_bringup navigation_launch.py


## Évaluation des performances (EVO)

Scripts d'évaluation disponibles dans `robot/scripts/evaluation/`.

Formats attendus :
- TUM : timestamp tx ty tz qx qy qz qw
- KITTI : format KITTI

Exemples de commandes :
```bash
cd robot/scripts/evaluation
./run_evo.sh

# Comparaison APE
evo_ape tum Odometry_tum.txt odom_tum.txt --align --plot --save_plot amcl_vs_odom.pdf

# Comparaison de trajectoires
evo_traj tum amcl_tum.txt odom_tum.txt --plot --save_plot traj_amcl_odom.pdf
```
Si `evo` n'est pas installé : `pip install evo` ou consultez `docs/installation.md`.

---

## Structure du projet (aperçu)

```
robot/
├── README.md
├── LICENSE
├── requirements.txt
├── docs/
│   ├── architecture.md
│   ├── installation.md
│   └── user_guide.md
├── src/
│   ├── slam/
│   │   ├── fast_lio2/
│   │   └── lio_sam/
│   ├── navigation/
│   │   └── nav2_configs/
│   └── robot_description/
│       ├── urdf/
│       ├── meshes/
│       └── launch/
├── config/
├── scripts/
│   ├── setup_environment.sh
│   └── evaluation/
├── launch/
└── results/
    └── trajectories/

## 🤝 Contribuer

Les contributions sont les bienvenues !

- Avant toute modification majeure, veuillez ouvrir une *issue* pour décrire votre proposition.
- Pour les corrections ou ajouts de code :  
  **fork → nouvelle branche → pull request**, avec une description claire et des tests si possible.
- Respectez les conventions ROS 2, le style de code et les bonnes pratiques en matière de sécurité et de gestion des ressources.


## Licence

Ce projet est publié sous licence MIT — voir le fichier `LICENSE`.


 📬 Contact

**Auteure :** Mechi Ilhem  
📧 Email :ilhemmechi5@gmail.com 
🔗 LinkedIn : https://www.linkedin.com/in/ilhem-mechi-0035a9283




