# Installation Guide

#Guide d'Installation Détaillé
Prérequis système

### Configuration matérielle recommandée

**Minimale :**
- CPU : Intel Core i5 (8ème génération) ou équivalent
- RAM : 8 GB
- GPU : Intégré (pour simulation uniquement)
- Stockage : 50 GB d'espace libre

**Recommandée :**
- CPU : Intel Core i7/i9 ou AMD Ryzen 7/9
- RAM : 16 GB ou plus
- GPU : NVIDIA avec 6GB+ VRAM (pour PV-RCNN)
- Stockage : 100 GB SSD

### Configuration logicielle

- **OS** : Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Kernel** : 5.15 ou supérieur
- **Python** : 3.10+
- **CUDA** : 11.7 ou 11.8 (pour PV-RCNN)
- **cuDNN** : Compatible avec votre version CUDA

---

## Installation de ROS2 Humble

### 1. Configuration des sources

```bash
# Configurer le locale UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Ajouter le dépôt ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe

# Ajouter la clé GPG
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Ajouter le dépôt à la liste des sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. Installation de ROS2 Humble Desktop

```bash
# Mise à jour des paquets
sudo apt update
sudo apt upgrade -y

# Installation complète de ROS2 Humble
sudo apt install ros-humble-desktop -y

# Outils de développement
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y
```

### 3. Initialisation de rosdep

```bash
sudo rosdep init
rosdep update
```

### 4. Configuration de l'environnement

```bash
# Ajouter au ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Vérifier l'installation
ros2 --version
```

---

## Installation de Gazebo Classic

```bash
# Installation de Gazebo Classic 11
sudo apt install gazebo11 -y
sudo apt install libgazebo11-dev -y

# Installation des plugins ROS2-Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-plugins -y

# Vérifier l'installation
gazebo --version
```

---

## Installation des algorithmes SLAM

### Fast-LIO2

```bash
# Créer un workspace
mkdir -p ~/robot/src
cd ~/robot/src

# Cloner Fast-LIO2
git clone https://github.com/hku-mars/FAST_LIO.git

# Installer les dépendances
sudo apt install ros-humble-pcl-conversions -y
sudo apt install ros-humble-pcl-ros -y
sudo apt install libpcl-dev -y
sudo apt install libeigen3-dev -y

# Compiler
cd ~/robot
colcon build --packages-select fast_lio2
source install/setup.bash
```

### LIO-SAM

```bash
cd ~/robot/src

# Cloner LIO-SAM (version ROS2)
git clone https://github.com/TixiaoShan/LIO-SAM.git

# Installer les dépendances
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-robot-localization -y
sudo apt install ros-humble-imu-tools -y

# Installer GTSAM
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev -y

# Compiler
cd ~/ros2_ws
colcon build --packages-select lio_sam
source install/setup.bash
```

---

## Installation de Nav2

```bash
# Installation complète de Nav2
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y

# Outils de visualisation
sudo apt install ros-humble-rviz2 -y
sudo apt install ros-humble-rqt* -y

# Planificateurs et contrôleurs
sudo apt install ros-humble-nav2-costmap-2d -y
sudo apt install ros-humble-nav2-planner -y
sudo apt install ros-humble-nav2-controller -y
```

---

## Installation de PV-RCNN

### 1. Installation de CUDA et cuDNN

```bash
# Vérifier si CUDA est installé
nvidia-smi

# Si non installé, télécharger CUDA 11.7 depuis :
# https://developer.nvidia.com/cuda-11-7-0-download-archive

# Installation de cuDNN (nécessite un compte NVIDIA)
# Télécharger depuis : https://developer.nvidia.com/cudnn
```

### 2. Création d'un environnement virtuel

```bash
# Installer virtualenv
sudo apt install python3-pip python3-venv -y

# Créer un environnement pour PV-RCNN
cd ~/ros2_ws
python3 -m venv pv_rcnn_env
source pv_rcnn_env/bin/activate
```

### 3. Installation des dépendances Python

```bash
# Mise à jour de pip
pip install --upgrade pip

# Installation de PyTorch avec CUDA
pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 --extra-index-url https://download.pytorch.org/whl/cu117

# Installation de spconv
pip install spconv-cu117

# Autres dépendances
pip install opencv-python
pip install pyyaml
pip install easydict
pip install scikit-image
pip install tqdm
pip install numba
pip install tensorboardX
pip install SharedArray
```

### 4. Installation de OpenPCDet (framework pour PV-RCNN)

```bash
cd ~/robot/src
git clone https://github.com/open-mmlab/OpenPCDet.git
cd OpenPCDet

# Compiler les modules CUDA
python setup.py develop

# Vérifier l'installation
cd tools
python demo.py --help
```

---

## Installation des dépendances du projet

### 1. Cloner le projet

```bash
cd ~/robot/src
git clone https://github.com/Ilhemmechi/lidar-slam-navigation.git
cd lidar-slam-navigation
```

### 2. Installer les dépendances Python

```bash
pip install -r requirements.txt
```

**Contenu typique de `requirements.txt` :**
```txt
numpy>=1.23.0
scipy>=1.9.0
matplotlib>=3.5.0
opencv-python>=4.6.0
pyyaml>=6.0
evo>=1.20.0
pandas>=1.5.0
seaborn>=0.12.0
```

### 3. Installer les dépendances ROS2 spécifiques

```bash
cd ~/robot
rosdep install --from-paths src --ignore-src -r -y
```
ester Fast-LIO2

```bash
# Lancer le nœud (avec un fichier bag test)
ros2 launch fast_lio mapping.launch.py
```

### 4. Tester LIO-SAM

```bash
ros2 launch lio_sam run.launch.py

### 6. Tester PV-RCNN

```bash
source ~/robot/pv_rcnn_env/bin/activate
cd ~/robot/src/OpenPCDet/tools
python demo.py --cfg_file cfgs/kitti_models/pv_rcnn.yaml \
               --ckpt ../checkpoints/pv_rcnn_8369.pth
```
Résolution des problèmes courants

### Problème 1 : "package not found"

```bash
# Réinstaller rosdep
sudo apt install python3-rosdep
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update

# Installer les dépendances manquantes
rosdep install --from-paths src --ignore-src -r -y
```

### Problème 2 : Erreur CUDA/GPU

```bash
# Vérifier CUDA
nvcc --version
nvidia-smi

# Vérifier PyTorch CUDA
python -c "import torch; print(torch.cuda.is_available())"
```

### Problème 3 : Gazebo ne démarre pas

```bash
# Réinitialiser Gazebo
killall gzserver gzclient
rm -rf ~/.gazebo/

# Réinstaller Gazebo
sudo apt install --reinstall gazebo11
```

### Problème 4 : Erreurs de compilation

```bash
# Nettoyer et recompiler
cd ~/robot
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Problème 5 : Conflits de dépendances Python

```bash
# Utiliser un environnement virtuel séparé
python3 -m venv ~/venvs/slam_env
source ~/venvs/slam_env/bin/activate
pip install -r requirements.txt
```
**Dernière mise à jour :** Janvier 2025  
**Testé sur :** Ubuntu 22.04 LTS, ROS2 Humble, CUDA 11.7
