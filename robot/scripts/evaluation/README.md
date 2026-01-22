# Évaluation avec EVO

Ce dossier contient des scripts et des instructions pour évaluer des trajectoires avec `evo` (APE/RPE, comparaisons de trajectoires).

## Formats attendus

- TUM : fichiers texte avec timestamp tx ty tz qx qy qz qw
- KITTI : format spécifique indexé (consultez la documentation `evo`)

## Méthode 3 : Commandes manuelles

Placez vos fichiers de trajectoire (TUM) dans le répertoire racine du projet ou fournissez des chemins absolus.

```bash
cd ~/lidar_slam/src/web_controller

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

Notes :
- Remplacez les noms de fichiers par vos fichiers réels (`amcl_tum.txt`, `odom_tum.txt`, etc.).
- Si `evo` n'est pas installé, installez-le via `pip install evo` ou suivez les instructions de `docs/installation.md`.
