# Changelog

## Unreleased

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
