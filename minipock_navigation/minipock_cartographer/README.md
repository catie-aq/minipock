# MiniPock Cartographer

- Maintainer status: developed
- Maintainer: Sébastien Delpeuch [s.delpeuch@catie.fr](mailto:s.delpeuch@catie.fr)
- License: Apache 2.0
- Bug / feature tracker: https://github.com/catie-aq/minipock_navigation/issues
- Source: git https://github.com/catie-aq/minipock_navigation

Ce package fournit l'implémentation de la stack de navigation 2 sur le robot MiniPock.

## Installation

Pour installer ce package, assurez-vous que votre workspace ROS est correctement configuré.

```bash
cd <your_ros2_workspace>
colcon build --packages-select minipock_cartographer
```

## Utilisation en simulation

Pour utiliser ce package il est nécessaire de lancer la simulation gazebo du robot MiniPock installée via le
package [minipock_gz](https://github.com/catie-aq/minipock_gz) en utilisant le launch file suivant:

```bash
ros2 launch minipock_gz minipock.launch.py
```

puis de lancer cartographer:

```bash
ros2 launch minipock_cartographer cartographer.launch.py
```

Vous pouvez ensuite lancer le node de téléopération pour contrôler le robot:

```bash
ros2 run minipock_teleop teleop_keyboard
```

Une fois la carte construite, vous pouvez enrégistrer la carte dans un fichier `.pgm` et `.yaml` en utilisant le node

```bash
ros2 run nav2_map_server map_saver_cli -f <file_name>
```
