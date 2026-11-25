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

## Suivi Jira (géré par l'agent, sans demande)

L'espace Jira `RD` (« RD SONU », catie-aq.atlassian.net) suit les travaux de
R&D de l'équipe. L'epic `RD-10` est celui de MiniPock. Ce suivi couvre ce dépôt
et les travaux MiniPock menés depuis lui (CI partagée, documentation, firmware).

- Structure : une tâche sous `RD-10` est un lot de travail, par exemple une
  étape de la reprise (`RD-895` refactor jusqu'à Nav2 en simulation, puis
  `RD-907` Open-RMF, `RD-908` passage au réel, `RD-910` compétences, `RD-909`
  cartographie collective). Une sous-tâche est une unité de travail, en général
  une PR. Son titre décrit le travail (« Monde de bureaux pour la
  simulation »), jamais « PR 12 : … », et le lien vers la PR va dans sa
  description.
- Le suivi est entièrement tenu par l'agent, pour chaque membre de l'équipe qui
  travaille ici avec Claude Code. Personne n'a à le faire à la main :
  - un travail qui commence sans ticket reçoit une tâche ou une sous-tâche,
    assignée à la personne qui mène la session (compte Jira connecté) ;
  - travail commencé : En cours ;
  - PR ouverte : commentaire avec le lien ;
  - PR mergée : Terminé ;
  - une tâche parente passe En cours dès qu'une de ses sous-tâches l'est, et
    Terminé quand toutes le sont ;
  - un ticket devenu sans objet passe Annulé, avec un commentaire qui dit
    pourquoi.
- Une automatisation de l'espace passe la tâche parente en Terminé dès que
  toutes ses sous-tâches le sont : une tâche dont le lot continue est rouverte
  (En cours) quand on lui ajoute une sous-tâche.
- Chaque changement s'accompagne d'un commentaire court et factuel, sans code
  ni logs.
- Sur un ticket assigné à un autre membre de l'équipe : ni changement de
  statut, ni changement d'assignation, ni mention @. Seulement un commentaire
  factuel quand du travail mergé le concerne.
