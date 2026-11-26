# MiniPock

Robot mobile R&D du CATIE : base sans ordinateur embarqué (STM32, micro-ROS en
Wi-Fi vers un PC hôte), deux cinématiques (différentielle et holonome), ROS 2
Jazzy, Gazebo Harmonic, Nav2, Cartographer. Ce dépôt contient toute la partie
ROS 2 côté PC hôte ; les firmwares vivent dans des dépôts séparés
(`catie-aq/minipock_zephyr-demo`, `catie-aq/minipock_mbed-rbdc`), les messages
dans le sous-module `minipock_msgs`, la documentation utilisateur dans
`catie-aq/minipock_documentation`.

## Livraison

- Code, commentaires et docstrings en anglais ; documentation utilisateur en
  français.
- Commits Conventional Commits en anglais (`type(scope): subject`), branches
  `type/short-description`.
- `pre-commit run --all-files` avant de pousser.
- Ce dépôt : branche, PR vers `main`, merge en rebase (seule méthode permise
  par le ruleset, historique linéaire) dès que la CI est verte, sans relecture
  obligatoire.
- Dépôts firmware (`minipock_zephyr-demo`, `minipock_mbed-rbdc`) : toute
  modification passe par une PR relue par un collègue avant merge.
- Les versions suivent `minipock/package.xml` ; un push sur `main` qui change
  cette version crée le tag (workflow `ros-tag`).
