# MiniPock Bringup

- Maintainer status: developed
- Maintainer: Sébastien Delpeuch [s.delpeuch@catie.fr](mailto:s.delpeuch@catie.fr)
- License: Apache 2.0
- Bug / feature tracker: https://github.com/catie-aq/minipock_bringup/issues
- Source: git https://github.com/catie-aq/minipock_bringup

Ce package fournit l'implémentation du Bringup de la plateforme Minipock.

## Installation

Pour installer ce package, assurez-vous que votre workspace ROS est correctement configuré.

```bash
cd <your_ros2_workspace>
colcon build --packages-select minipock_bringup
```

## Utilisation

Pour lancer le Bringup de la plateforme Minipock, il suffit de lancer la commande suivante :

```bash
ros2 launch minipock_bringup bringup.launch.py
```

Il est nécessaire d'être sur le même réseau que la plateforme Minipock pour pouvoir la contrôler. Le package est nécessaire pour lancer d'autres packages de la plateforme Minipock (hormis le package `minipock_teleop`).

## API Documentation

### Noeuds

- `/joint_state_publisher`: Ce noeud recueille et publie l'état de tous les joints mobiles du robot.
- `/odometry_transform_publisher`: Ce noeud génère et publie les informations de transformation d'odométrie, qui comprennent la position et l'orientation du robot dans le repère du monde.
- `/robot_state_publisher`: Ce noeud publie l'intégralité de l'état (position et vitesse) du robot sur le sujet `/joint_states`.

### Topics

- `/joint_states`: Ce topic diffuse l'état de tous les joints mobiles du robot, y compris la position, la vitesse et l'effort appliqué.
- `/odom`: Ce topic publie les informations d'odométrie, y compris la position et la vitesse dans le repère du monde.
- `/robot_description`: Ce topic publie le URDF (Universal Robot Description Format) du robot, qui fournit des informations descriptives détaillées sur le modèle du robot.
- `/tf`: Le topic `/tf` publie les transformations entre les différents repères coordonnés dans le robot.
- `/tf_static`: Ce topic publie des informations de transformation statiques, qui contrairement à `/tf`, ne changent pas avec le temps.
