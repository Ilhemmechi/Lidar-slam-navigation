#!/usr/bin/env python3
import numpy as np

# Charger les trajectoires
print("Chargement des trajectoires...")
gt = np.loadtxt('/home/ilhem/lidar_slam/ground_truth_real.txt', comments='#')
amcl = np.loadtxt('amcl_tum.txt')

print(f"Ground truth: {len(gt)} poses")
print(f"AMCL: {len(amcl)} poses")

# Calculer l'offset temporel
offset = amcl[0, 0] - gt[0, 0]
print(f"\nDécalage temporel: {offset:.2f} secondes ({offset/86400:.2f} jours)")

# Appliquer l'offset au ground truth
gt_synced = gt.copy()
gt_synced[:, 0] += offset

# Sauvegarder
np.savetxt('ground_truth_synced.txt', gt_synced, 
           fmt='%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f')

print(f"\n✓ Ground truth synchronisé sauvegardé")
print(f"  Nouveau début GT: {gt_synced[0, 0]:.6f}")
print(f"  Début AMCL:       {amcl[0, 0]:.6f}")
print(f"  Différence:       {abs(gt_synced[0, 0] - amcl[0, 0]):.6f} s")
