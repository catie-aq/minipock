# Changelog

## [2.2.1](https://github.com/catie-aq/minipock/releases/tag/2.2.1)

Version qui fige l'état du dépôt avant le refactor de la base jusqu'à Nav2 en simulation.

### Changed

- Renommage de la frame du LiDAR `lds_01_link` en `lidar_link` dans l'URDF et les configurations RViz
- Génération du modèle au bringup selon la cinématique de chaque robot (`mode` de `minipocks.yaml`)
- Ajout dans `raw_data_transformer` d'une fusion des odométries roue et optique (`odom_optc`, filtre de Kalman `filterpy`)
- Carte par défaut `catie` dans `minipocks.yaml`
- Ajout de `pynput` dans l'image Docker, `jsonschema` épinglé en 4.10.3

### Known issues

- `filterpy` n'est pas installé dans l'image Docker : `raw_data_transformer` ne démarre pas tel quel
- `use_sim_time` reste vrai en réel (`bool("False")` dans les launches de spawn)

## [2.2.0](https://github.com/catie-aq/minipock/releases/tag/2.2.0)

### Added

- Ajout de l'adaptateur de flotte minipock et de la configuration de salle vide
- Ajout de `minipock_firmware_updater` et des messages personnalisés pour MiniPock

## [2.1.0](https://github.com/catie-aq/minipock/releases/tag/2.1.0)

### Added

- Publication d'une image Docker pour MiniPock sur GitHub Container Registry
- Centralisation de la configuration dans un fichier YAML dans le package `minipock`
- Ajout de la téléopération en mode FPS

### Changed

- Déplacement de la documentation dans [un dépôt dédié](https://github.com/catie-aq/minipock_documentation)
- Modification de `minipock_description` pour supporter la mécanique holonome
- Modification de `minipock_description` pour supporter plusieurs robots (préfixe des topics, des paramètres, etc.)
- Modification de `minipock_bringup` pour supporter plusieurs robots (préfixe des topics, des paramètres, etc.)
- Modification de `minipock_navigation` pour supporter plusieurs robots (préfixe des topics, des paramètres, etc.)
- Modification de `minipock_gz` pour supporter plusieurs robots (préfixe des topics, des paramètres, etc.)
- Mise à jour sur ROS Jazzy et Gazebo Harmonic

## [2.0.0](https://github.com/catie-aq/minipock/releases/tag/2.0.0)

### Added

- Création d'un container Docker pour MiniPock (support avec devcontainer)
- Ajout du package `micro_ros_agent` pour la communication ROS2 <-> Micro-ROS au lancement du bringup
- Création du package `minipock`
- Ajout de la CI/CD avec GitHub Actions

### Changed

- Mise à jour de `minipock_description` pour conformer à la mécanique 2.0.0
- Refactorisation de `minipock_description` pour suivre les conventions de nommage des frames et ouvrir la possibilité d'utiliser un namespace par robot
- Fusion des différents dépôts en un seul `minipock`

## [1.0.0](https://github.com/catie-aq/minipock/releases/tag/1.0.0)

### Added

- Création des packages `minipock_gz`, `minipock_description`, `minipock_bringup` et `minipock_navigation`
