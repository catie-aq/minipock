# MiniPock Navigation

- Maintainer status: developed
- Maintainer: Sébastien Delpeuch [s.delpeuch@catie.fr](mailto:s.delpeuch@catie.fr)
- License: Apache 2.0
- Bug / feature tracker: https://github.com/catie-aq/minipock_navigation/issues
- Source: git https://github.com/catie-aq/minipock_navigation

Ce package fournit l'implémentation de la stack de navigation 2 sur le robot MiniPock.

## Installation

Pour installer ce package, assurez-vous que votre workspace ROS est correctement configuré.

```bash
cd <your_ros2_workspace>/src
git clone git@github.com:catie-aq/minipock_navigation.git
cd ..
colcon build --packages-select minipock_navigation2
```

## Utilisation en simulation

Pour utiliser ce package il est nécessaire de lancer la simulation gazebo du robot MiniPock installée via le
package [minipock_gz](https://github.com/catie-aq/minipock_gz) en utilisant le launch file suivant:

```bash
ros2 launch minipock_gz minipock.launch.py
```

puis de lancer le launch file de navigation:

```bash
ros2 launch minipock_navigation2 navigation2.launch.py
```

## Configuration

La configuration de la stack de navigation se fait via le
fichier [minipock.yaml](https://github.com/catie-aq/minipock_navigation/blob/main/minipock_navigation2/param/minipock.yaml)
