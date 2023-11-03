- Maintainer status: developed
- Maintainer: Sébastien Delpeuch [s.delpeuch@catie.fr](mailto:s.delpeuch@catie.fr)
- License: Apache 2.0
- Bug / feature tracker: https://github.com/catie-aq/minipock_description/issues
- Source: git https://github.com/catie-aq/minipock_description

Ce package contient les fichiers de description du robot MiniPock au format URDF utilisant `xacro` pour la génération
des modèles. Le dépôt n'est pas lié à un simulateur spécifique, mais il est conçu pour fonctionner avec Gazebo Sim
Garden et ROS 2 Humble.

## Installation

Clonez le dépôt dans votre espace de travail.

```bash
cd ~/[your colcon workspace]/src
git clone git@github.com:catie-aq/minipock_description.git
```

Compilez le package avec colcon.

```bash
cd ~/[your colcon workspace]
colcon build --merge-install
```

Modifiez votre `.bashrc` pour indiquer à Gazebo où trouver les modèles.

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH~/[your colcon workspace]/install/share
```

## Utilisation

```bash
ros2 launch minipock_gz minipock.launch.py
```

## Fichier URDF

La génération du modèle se fait via la définition d’un `URDF` dont le point d’entrée
est [minipock.urdf.xacro](https://github.com/catie-aq/minipock_description/blob/main/urdf/minipock.urdf.xacro) sa
conversion en `sdf` pour être utilisée dans Gazebo Sim Garden se fait via le
script [model.py](https://github.com/catie-aq/minipock_description/blob/main/minipock_description/model.py) et plus
particulièrement la fonction `generate` qui est un point d’entrée du
package https://github.com/catie-aq/minipock_description/blob/19d8f223b17d1fa1e83059e489daa218251e7e88/minipock_description/model.py#L84-L109

Les pièces d’origines servant à la construction sont disponibles
sur [OnShape](https://cad.onshape.com/documents/33cae3bcf76fa1a7bad5518d/w/291d81df1473dfe37dbb5dbf/e/d2e6f80356159bebf1722dda)

![Résumé de la construction de l’URDF](minipock_black.svg)

Résumé de la construction de l’URDF

## Robot différentiel

La base définie
dans [minipock.urdf.xacro](https://github.com/catie-aq/minipock_description/blob/main/urdf/minipock.urdf.xacro) possède
deux moteurs nommés `stepper_left` et `stepper_right`, ces deux moteurs permettent d’obtenir une base différentielle
utilisant le
plugin [gz-sim-diff-drive-system](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)

```xml

<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
  <left_joint>stepper_left_wheel_joint</left_joint>
  <right_joint>stepper_right_wheel_joint</right_joint>
  <wheel_separation>0.335</wheel_separation>
  <wheel_radius>0.036</wheel_radius>
  <odom_publish_frequency>1</odom_publish_frequency>
  <topic>cmd_vel</topic>
  <child_frame_id>base_link</child_frame_id>
  <frame_id>odom</frame_id>
</plugin>
```

## Lidar LDS 01

- Le capteur de distance laser 360 LDS-01 est un scanner laser 2D capable de détecter sur 360 degrés qui recueille un
  ensemble de données autour du robot à utiliser pour le SLAM (Simultaneous Localization and Mapping) et la navigation.
- Le LDS-01 est utilisé pour les modèles TurtleBot3 Burger, Waffle et Waffle Pi.
- Il prend en charge l'interface USB (USB2LDS). Il prend en charge l'interface UART pour une carte embarquée.

Sa simulation se fait à l’aide du
plugin [gpu_lidar](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1GpuLidarSensor.html) défini
dans [sensors.xacro](https://github.com/catie-aq/minipock_description/blob/main/urdf/sensors.xacro)

```xml

<sensor type="gpu_lidar" name="lds_01">
  <update_rate>5</update_rate>
  <topic>scan_raw</topic>
  <visualize>true</visualize>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>0.0</min_angle>
        <max_angle>6.280</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>3.5</max>
      <resolution>0.0150</resolution>
    </range>
  </lidar>
</sensor>
```