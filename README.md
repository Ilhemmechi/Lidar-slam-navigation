Pipeline LiDAR SLAM pour Navigation Robotique Autonome :
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-17-orange)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-red)](http://gazebosim.org/)

> **Mémoire de Master** - Systèmes Électroniques Embarqués  
> **Auteure** : Mechi Ilhem  
> **Encadrante** : Dr. Rzouga Lamia  
> **Institution** : Institut Supérieur des Sciences Appliquées et de Technologies (ISSAT), Sousse  
> **Laboratoire** : LATIS - Technologies Avancées et Systèmes Intelligents, ENISo  
> **Année** : 2024-2025

🔧 Architecture Technique Complète: 


<img width="866" height="522" alt="3e27ca5d-d6ef-485c-aaff-0aa07784bd1b" src="https://github.com/user-attachments/assets/9a5a5fc1-a739-4b10-9ad4-febfb6ca48a0" />



## 🏗️ Architecture et Pipelines SLAM

### 🔄 Deux Approches Complémentaires

**Pipeline A : Fast-LIO2 (SLAM Pur - Temps Réel)**

```
LiDAR + IMU → ESEKF (Filtre Kalman Itéré) → ikd-tree dynamique
       ↓
   Carte 3D dense (/cloud) + Odométrie continue (/Odom)
       ↓
   Navigation directe (pas de carte préalable)

**Caractéristiques** :
- ✅ **Exploration** : Fonctionne sans carte existante
- ✅ **Précision** : RMSE 1.7 cm, dérive minimale
- ✅ **Temps réel** : Mise à jour 20 Hz, latence 16 ms
- ✅ **Robustesse** : 94% réussite, stable en mouvement
- ⚠️ **Ressources** : CPU 55%, RAM 11 GB

**Pipeline B : LIO-SAM + AMCL (Cartographie puis Localisation)**

**Phase 1 : Cartographie avec LIO-SAM**
```
LiDAR + IMU → Graph SLAM + iSAM2 → Optimisation globale
       ↓
   Carte 3D optimisée + Fermeture de boucles
       ↓
   Conversion 3D → 2D (.pgm + .yaml)
```

**Phase 2 : Localisation avec AMCL**
```
Carte statique (.pgm) → map_server
       ↓
   AMCL (Monte Carlo) → Localisation probabiliste
       ↓
   Navigation sur carte connue
```

**Caractéristiques** :
- ✅ **Planification globale** : Carte complète disponible
- ✅ **Stabilité** : 0 dérive cumulée (carte fixe)
- ✅ **Efficacité** : CPU 40%, RAM 6 GB (-30% / -45%)
- ⚠️ **Préparation** : 15-20 min cartographie initiale
- ⚠️ **Précision** : RMSE 50.4 cm (vs 1.7 cm Fast-LIO2)
- ⚠️ **Adaptabilité** : Ne s'adapte pas aux changements
  **Cartes générées** :
1. **Carte 3D LIO-SAM** (`.pcd`) : Nuage de points optimisé avec loop closure
2. **Carte 2D Nav2** (`.pgm` + `.yaml`) : Grille d'occupation pour navigation
   - Résolution : 5 cm/pixel
   - Zones : Libre (blanc), Occupé (noir), Inconnu (gris)
  
*Démonstrations :
1-Carte 3D génerée avec 3D :

<img width="862" height="681" alt="59f480a2-45b7-433f-88e8-dfac460ea17f" src="https://github.com/user-attachments/assets/5190634d-025b-4d30-87c6-5357ece601d8" />

2-carte 2D compatible avec Nav2
<img width="551" height="484" alt="3bc4a791-d7f3-4d02-bd32-cd090e41ec80" src="https://github.com/user-attachments/assets/8a0b54b1-ef14-4022-a6b4-f57fd0a6bb5a" />
3-Interface web pour le controle du robot 
![interface_web gif](https://github.com/user-attachments/assets/f5bc17b0-ab9a-44df-a02c-a99d27b583ba)
4-Navigation autonome avec fast_lio2

![fast_lio2_nav gif](https://github.com/user-attachments/assets/2315c8eb-047e-4934-8fce-48b47e75c07d)


5-Navigation autonome avec lio_sam 

![lio_sam_nav gif](https://github.com/user-attachments/assets/5e4beed7-8c45-42d9-bfcc-d5c2fb6fc226)
6-Détection d'objets :
* Détection des pédesteriens
  <img width="226" height="269" alt="eb547efb-6b75-46aa-8e53-af973bd643cd" src="https://github.com/user-attachments/assets/1707ec2d-2a1c-48c1-96cd-8dc5d19e3e54" />
*Détection des voitures
![1b0de6ba-cf30-48a7-9b27-9f2d01d6f140](https://github.com/user-attachments/assets/96fd6db0-76b7-443a-8ca6-523f65b4878a)

📊 Résultats Quantitatifs

### Métriques SLAM

**Fast-LIO2** :
```
RMSE : 0.017 m (1.7 cm)  ⭐ Précision exceptionnelle
STD  : 0.011 m           ⭐ Très stable
Max  : 0.053 m           ⭐ Robuste
```

**LIO-SAM + AMCL** :
```
RMSE : 0.504 m (50.4 cm)
STD  : 0.338 m
Max  : 1.010 m
```

**Conclusion** : Fast-LIO2 **30× plus précis**, recommandé pour applications haute précision

---

### Performance Navigation

| Système | Succès | Échecs | CPU | RAM |
|---------|--------|--------|-----|-----|
| **Fast-LIO2** | **94%** | 6% (1) | 55% | 11 GB |
| **LIO-SAM + AMCL** | 82% | 18% (3) | 40% | 6 GB |

**Seuil critique** : APE > 0.8m → Échec probable (LIO-SAM)

---

### Détection PV-RCNN

| Classe | AP Easy | AP Moderate | AP Hard |
|--------|---------|-------------|---------|
| **Car** | 91.2% | 86.7% | 83.5% |
| **Pedestrian** | 82.4% | 74.9% | 68.2% |

**Dataset** : 2847 scènes Gazebo | **Entraînement** : 80 époques (8h)
🎯 Quand Utiliser Chaque Pipeline ?
   📊 Comparaison des Pipelines

| Critère | Fast-LIO2 | LIO-SAM + AMCL |
|---------|-----------|----------------|
| **Mode opératoire** | SLAM continu | Cartographie → Localisation |
| **Carte requise** | ❌ Non (exploration) | ✅ Oui (préalable) |
| **Précision** | ⭐⭐⭐⭐⭐ (1.7 cm) | ⭐⭐⭐ (50 cm) |
| **Temps préparation** | 0 min | 15-20 min |
| **Adaptation environnement** | ✅ Temps réel | ❌ Carte statique |
| **Planification** | Locale (4m) | Globale (complète) |
| **Ressources** | Élevées | Modérées |
| **Fiabilité** | 94% | 82% |
#### **Choisir Fast-LIO2 si** :
- 🔹 Environnement **inconnu** ou **non cartographié**
- 🔹 Besoin de **précision centimétrique** (±2 cm)
- 🔹 Environnement **dynamique** (obstacles mobiles)
- 🔹 **Exploration** autonome requise
- 🔹 Pas de temps pour cartographie préalable

**Exemples d'usage** :
- Robots d'exploration (bâtiments inconnus, grottes)
- Interventions d'urgence (sites sinistrés)
- Navigation agricole (champs variant selon saison)
- Recherche et sauvetage

#### **Choisir LIO-SAM + AMCL si** :
- 🔹 Environnement **connu et stable**
- 🔹 Navigation **répétée** sur même zone
- 🔹 Précision **décimétrique acceptable** (±50 cm)
- 🔹 **Ressources limitées** (CPU/RAM)
- 🔹 Planification **long terme** nécessaire

**Exemples d'usage** :
- Robots logistiques (entrepôts fixes)
- Surveillance routinière (bâtiments)
- Systèmes embarqués (puissance limitée)
- Navigation industrielle (usines)
  

![1b0de6ba-cf30-48a7-9b27-9f2d01d6f140](https://github.com/user-attachments/assets/b2986d59-394d-49dd-b323-f3eef73fb08f)
