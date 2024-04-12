# MiniPock Keyboard Teleoperation

Ce package fournit un moyen de contrôler le robot MiniPock. Il a été créé pour être utilisé avec ROS 2. Il permet de
commander le mouvement linéaire et angulaire du robot à l'aide des touches de clavier suivantes : `z`, `q`, `s`, `d` et `x`.

## Installation

Pour installer ce package, assurez-vous que votre workspace ROS est correctement configuré.

```bash
cd <your_ros2_workspace>
colcon build --packages-select minipock_teleop
```

Ce package dépend des packages rclpy et geometry_msgs.msg. Veuillez vous assurer qu'ils sont installés dans votre
environnement.

## Utilisation

Pour utiliser ce package, vous pouvez exécuter le script Python à partir de la ligne de commande:

```bash
ros2 run minipock_teleop teleop_keyboard
```
