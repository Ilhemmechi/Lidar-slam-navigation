# User Guide

Démarrage rapide

### Lancement de la simulation complète (Fast-LIO2 + Navigation)

```bash
# Terminal 1 : Lancer Gazebo avec le robot
cd ~/robot/
source install/setup.bash
ros2 launch mon_urdf gazebo.launch.py

# Terminal 2 : Lancer Fast-LIO2
ros2 launch fast_lio2 mapping.launch.py
ou
ros2 launch lio_sam run.launch.py
# Terminal 3 : Lancer Nav2
ros2 launch nav2_bringup navigation_launch.py

```

## Évaluation

Pour comparer des trajectoires et calculer des métriques APE/RPE, utilisez `evo`.

- Instructions et exemples : `scripts/evaluation/README.md`.
- Script d'exemple : `scripts/evaluation/run_evo.sh` (génère des rapports et graphiques dans `results/`).

Méthode rapide (manuel) :

```bash
cd ~/robot/src/web_controller

# Lancer le serveur web
python3 web_controller_server.py

# Comparer AMCL avec EVO
evo_ape tum Odometry_tum.txt odom_tum.txt \
    --align --plot --save_plot amcl_vs_odom.pdf

# Comparer trajectoires
evo_traj tum amcl_tum.txt odom_tum.txt \
    --plot --save_plot traj_amcl_odom.pdf

# Comparaison globale
evo_traj tum ~/lidar_slam/Odometry_tum.txt ~/lidar_slam/odom_tum.txt \
    --plot --save_plot ~/lidar_slam/trajectories_comparison.pdf
```

Remarques :

- Installez `evo` dans votre virtualenv : `pip install evo`.
- Les fichiers TUM attendus sont des fichiers texte contenant : timestamp tx ty tz qx qy qz qw.
