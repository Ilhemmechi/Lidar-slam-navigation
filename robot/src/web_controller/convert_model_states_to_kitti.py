#!/usr/bin/env python3
"""
Conversion Model States vers format KITTI
Conversion correcte avec mapping des classes et dimensions réalistes
"""

import os
from tf_transformations import euler_from_quaternion

# Configuration
input_dir = "/home/ilhem/lidar_slam/simulation_data/model_states"
output_dir = "./kitti_eval/label_2"

# Dimensions réalistes par type d'objet (longueur, largeur, hauteur en mètres)
OBJECT_DIMENSIONS = {
    # Véhicules
    'car': (3.9, 1.6, 1.5),
    'vehicle': (3.9, 1.6, 1.5),
    'polaris': (3.5, 1.8, 1.8),
    'ranger': (3.5, 1.8, 1.8),
    'truck': (10.0, 2.5, 3.0),
    'van': (5.0, 2.0, 2.0),
    
    # Personnes
    'person': (0.6, 0.6, 1.75),
    'pedestrian': (0.6, 0.6, 1.75),
    'standing': (0.6, 0.6, 1.75),
    'walking': (0.6, 0.6, 1.75),
    
    # Cyclistes
    'cyclist': (1.8, 0.6, 1.8),
    'bike': (1.8, 0.6, 1.8),
    'bicycle': (1.8, 0.6, 1.8),
    
    # Robots et rovers
    'robot': (1.0, 0.8, 1.2),
    'rover': (2.5, 2.0, 1.5),
    'mars_rover': (2.5, 2.0, 1.5),
    
    # Objets divers
    'cone': (0.3, 0.3, 0.7),
    'construction': (0.3, 0.3, 0.7),
    'barrier': (2.0, 0.4, 0.8),
    'jersey': (2.0, 0.4, 0.8),
    'dumpster': (2.0, 1.5, 1.5),
    'trash': (0.6, 0.6, 0.8),
    'can': (0.6, 0.6, 0.8),
    'pallet': (1.2, 0.8, 0.15),
    'euro_pallet': (1.2, 0.8, 0.15),
    
    # Nature
    'tree': (1.5, 1.5, 5.0),
    'pine': (2.0, 2.0, 8.0),
    
    # Infrastructure
    'lamp': (0.3, 0.3, 4.0),
    'post': (0.3, 0.3, 4.0),
    
    # Équipement
    'kinect': (0.3, 0.1, 0.1),
    'gripper': (0.2, 0.15, 0.1),
    'arm': (0.8, 0.3, 0.5),
    
    # Défaut
    'default': (1.0, 1.0, 1.0)
}

# Mapping vers classes KITTI standard
CLASS_MAPPING = {
    'car': 'Car',
    'vehicle': 'Car',
    'polaris': 'Car',
    'ranger': 'Car',
    'truck': 'Truck',
    'van': 'Van',
    'person': 'Pedestrian',
    'pedestrian': 'Pedestrian',
    'standing': 'Pedestrian',
    'walking': 'Pedestrian',
    'cyclist': 'Cyclist',
    'bike': 'Cyclist',
    'bicycle': 'Cyclist'
}


def get_object_class(name):
    """Détermine la classe KITTI à partir du nom de l'objet"""
    name_lower = name.lower()
    
    # Chercher des mots-clés dans le nom
    for keyword, kitti_class in CLASS_MAPPING.items():
        if keyword in name_lower:
            return kitti_class
    
    # Classe par défaut pour objets non reconnus
    return 'DontCare'


def get_object_dimensions(name):
    """Retourne les dimensions (l, w, h) en fonction du nom de l'objet"""
    name_lower = name.lower()
    
    # Chercher des mots-clés dans le nom
    for keyword, dims in OBJECT_DIMENSIONS.items():
        if keyword in name_lower:
            return dims
    
    # Dimensions par défaut
    return OBJECT_DIMENSIONS['default']


def convert_to_kitti_format():
    """Convertit les model_states au format KITTI"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Statistiques
    stats = {
        'total_frames': 0,
        'total_objects': 0,
        'classes': {}
    }
    
    print("🔄 Conversion des model_states vers format KITTI...")
    print(f"   Input:  {input_dir}")
    print(f"   Output: {output_dir}\n")
    
    # Traiter chaque fichier
    for idx, filename in enumerate(sorted(os.listdir(input_dir))):
        filepath = os.path.join(input_dir, filename)
        
        if not os.path.isfile(filepath):
            continue
        
        with open(filepath) as f:
            lines = f.readlines()
        
        kitti_lines = []
        
        for line in lines:
            data = line.strip().split()
            
            if len(data) < 7:
                continue
            
            # Extraire les données
            # Format: nom x y z qx qy qz qw [optionnel: l w h]
            name = " ".join(data[:-7])
            x, y, z = map(float, data[-7:-4])
            qx, qy, qz, qw = map(float, data[-4:])
            
            # Obtenir la classe KITTI
            obj_class = get_object_class(name)
            
            # Obtenir les dimensions
            if len(data) >= 10:
                # Si les dimensions sont dans le fichier
                l, w, h = map(float, data[-10:-7])
            else:
                # Sinon, utiliser les dimensions par défaut
                l, w, h = get_object_dimensions(name)
            
            # Calculer l'angle de rotation (yaw)
            yaw = euler_from_quaternion([qx, qy, qz, qw])[2]
            
            # Format KITTI:
            # type truncated occluded alpha bbox_left bbox_top bbox_right bbox_bottom 
            # height width length x y z rotation_y [score]
            
            # Valeurs par défaut pour les champs non utilisés
            truncated = 0.0
            occluded = 0
            alpha = -10  # Angle d'observation (on met -10 si inconnu)
            
            # Bounding box 2D (on met des valeurs par défaut car pas de caméra)
            bbox_left = 0.0
            bbox_top = 0.0
            bbox_right = 50.0
            bbox_bottom = 50.0
            
            # Score de confiance (1.0 pour ground truth)
            score = 1.0
            
            # Créer la ligne KITTI
            line_kitti = (
                f"{obj_class} "
                f"{truncated:.2f} {occluded} {alpha:.2f} "
                f"{bbox_left:.2f} {bbox_top:.2f} {bbox_right:.2f} {bbox_bottom:.2f} "
                f"{h:.2f} {w:.2f} {l:.2f} "
                f"{x:.2f} {y:.2f} {z:.2f} "
                f"{yaw:.2f} {score:.2f}\n"
            )
            
            kitti_lines.append(line_kitti)
            
            # Statistiques
            stats['total_objects'] += 1
            stats['classes'][obj_class] = stats['classes'].get(obj_class, 0) + 1
        
        # Sauvegarder le fichier
        output_file = f"{output_dir}/{idx:06d}.txt"
        with open(output_file, "w") as f_out:
            f_out.writelines(kitti_lines)
        
        stats['total_frames'] += 1
        
        # Afficher la progression tous les 100 fichiers
        if (idx + 1) % 100 == 0:
            print(f"   Traité: {idx + 1} fichiers...")
    
    # Afficher les statistiques
    print(f"\n✅ Conversion terminée!")
    print(f"   Frames converties: {stats['total_frames']}")
    print(f"   Objets totaux:     {stats['total_objects']}")
    print(f"\n📊 Répartition par classe:")
    
    for class_name, count in sorted(stats['classes'].items(), key=lambda x: -x[1]):
        percentage = (count / stats['total_objects']) * 100
        print(f"   {class_name:15s}: {count:6d} ({percentage:5.1f}%)")
    
    print(f"\n💾 Fichiers sauvegardés dans: {output_dir}/")


if __name__ == "__main__":
    convert_to_kitti_format()

