# MiniPock

Robot mobile R&D du CATIE : base sans ordinateur embarqué (STM32, micro-ROS en
Wi-Fi vers un PC hôte), deux cinématiques (différentielle et holonome), ROS 2
Jazzy, Gazebo Harmonic, Nav2, Cartographer. Ce dépôt contient toute la partie
ROS 2 côté PC hôte ; les firmwares vivent dans des dépôts séparés
(`catie-aq/minipock_zephyr-demo`, `catie-aq/minipock_mbed-rbdc`), les messages
dans le sous-module `minipock_msgs`.

## Paquets

| Paquet | Rôle |
|--------|------|
| `minipock` | Métapaquet pur (aucun fichier installé) |
| `minipock_description` | URDF/xacro des deux cinématiques |
| `minipock_gz` | Mondes Gazebo, lancement de la sim, ponts ROS-Gazebo |
| `minipock_bringup` | Fichiers de flotte (`config/minipocks.yaml` par défaut en sim, `config/minipocks_real.yaml` pour le robot réel, passé avec `config:=` à partir de la tâche 6), chargement et validation de la config, rendu des paramètres Nav2 et localisation de la flotte ; robot réel : agent micro-ROS, mise en forme odométrie et scan |
| `minipock_navigation/*` | Nav2 (`minipock_navigation2`), Cartographer, téléopération |
| `minipock_fleet_adapter` | Adaptateur Open-RMF (chantier séparé, ne pas toucher hors de ce chantier) |
| `minipock_firmware_updater` | Mise à jour du firmware par ROS |

## Environnement

Tout se fait dans les images Docker du dépôt (`.devcontainer/Dockerfile`,
cibles `base`, `simulation`, `real`), publiées sur
`ghcr.io/catie-aq/minipock:<version>-<cible>`.

- Compiler avec `--merge-install` : les chemins Gazebo de l'image
  (`GZ_SIM_RESOURCE_PATH`) supposent un `install/share` unique.
- Sans écran ni GPU (CI, conteneur sans `/dev/dri`) : `LIBGL_ALWAYS_SOFTWARE=1`
  et `gz sim -s --headless-rendering`. Le LiDAR simulé (`gpu_lidar`) tourne
  alors en rendu logiciel, plus lentement que le temps réel.
- `ROS_DOMAIN_ID` : 10 sur le réseau du robot. Pour un test local, prendre un
  autre domaine et `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` pour ne pas
  parler aux robots réels.

```bash
colcon build --symlink-install --merge-install
```

```bash
ros2 launch minipock_gz spawn.launch.py
```

Nav2 d'un robot (localisation lancée à part, voir « Navigation ») :

```bash
ros2 launch minipock_navigation2 robot.launch.py namespace:=minipock_0 params_file:=<fichier rendu>
```

Cartographer pour un robot (carte sur `/minipock_0/map`) :

```bash
ros2 launch minipock_cartographer cartographer.launch.py namespace:=minipock_0
```

## Conventions ROS

- Un robot = un namespace `minipock_<id>` (`<id>` = clé de `fleet` dans la config).
- Toutes les frames d'un robot sont préfixées par son namespace
  (`minipock_0/base_footprint`, `minipock_0/odom`, `minipock_0/lidar_link`) et
  publiées sur le `/tf` global ; la frame `map` est commune à la flotte. Ce choix
  garde un seul arbre TF pour RViz, Open-RMF et la cartographie multi-robot.
- Arbre par robot : `map → minipock_<id>/odom → minipock_<id>/base_footprint →
  minipock_<id>/base_link → …`, avec un seul publieur par transformation.
- Topics relatifs au namespace : `cmd_vel`, `odom`, `scan`, `joint_states`.
- Sim et réel exposent les mêmes topics et frames : un nœud ne doit pas savoir
  s'il tourne en simulation, hors `use_sim_time`.

## Navigation

- Une seule carte pour la flotte : un `map_server` global publie `/map` (frame
  `map`) ; chaque robot a son AMCL (`/<ns>/amcl`, `map_topic: /map`) ; un seul
  `lifecycle_manager_localization` gère `map_server` et tous les AMCL. Ces
  actions viennent de `minipock_bringup.localization.localization_actions`.
  Ce gestionnaire tourne sans bond (`bond_timeout: 0.0`) : un serveur crée son
  bond dans son namespace sous son nom court (`amcl`), le gestionnaire l'attend
  sur `/bond` sous `minipock_0/amcl`, ils ne se trouvent jamais.
- Le reste de Nav2 vient de `nav2_bringup/launch/navigation_launch.py`, inclus
  par `minipock_navigation2/launch/robot.launch.py` sous le namespace du robot.
  `SetRemap('/tf', '/tf')` et `SetRemap('/tf_static', '/tf_static')` avant
  l'include annulent ses remaps vers `tf` : tout reste sur le `/tf` global.
  Même chose pour AMCL et Cartographer.
- Paramètres Nav2 en couches, fusionnés au lancement par
  `minipock_bringup.nav2_params.render_params_file` :
  `minipock_navigation2/param/common.yaml`, puis la surcouche de la cinématique
  (`differential.yaml` ou `holonomic.yaml`), puis le champ optionnel
  `nav2_params` du robot dans le fichier de flotte (chemin, ou nom d'un fichier
  de `minipock_navigation2/param`). Les dictionnaires sont fusionnés, une liste
  de la surcouche remplace celle du dessous. Marqueurs remplacés :
  `<robot_namespace>`, `<x>`, `<y>`, `<yaw>` (pose initiale AMCL).
- Clés racine sans namespace (`amcl:`, `controller_server:`…) : le launch les
  range sous le namespace du robot. Topics relatifs (`scan`, `odom`), sauf ceux
  des costmaps, absolus (`/<robot_namespace>/scan`), et la carte `/map`.
- Aucune vitesse ni accélération de Nav2 ne dépasse les limites du firmware
  (2 m/s, 0,7 m/s², décélération 3,4 m/s², 3π rad/s, π rad/s²) ; les tests de
  `minipock_bringup` le vérifient. MPPI, velocity smoother et behavior server
  partagent les mêmes limites.
- `docking_server` (géré par `navigation_launch.py`) a un plugin
  `SimpleNonChargingDock` et aucun dock : il s'active sans servir.
- Cartographer : `cartographer.launch.py namespace:=<ns>` génère le lua (et la
  config RViz) avec le namespace à la place de `__NS__` ; il ne publie que
  `map → <ns>/odom`.

## Robot réel

- Le firmware publie `odom_raw` et `scan_raw` en micro-ROS (agent UDP, port
  8888, lancé par `minipock_bringup`) ; `minipock_bringup` les remet en forme
  vers `odom`, `scan` et le TF.
- Topics, frames et types échangés avec le firmware forment un contrat :
  un changement ici qui le touche demande la modification correspondante dans
  le dépôt firmware, sinon le robot réel casse sans que la CI le voie.
- Rien ne se teste sur le robot réel sans sonde de programmation à portée.

## Qualité et livraison

- Code, commentaires et docstrings en anglais ; doc utilisateur en français.
- Commits Conventional Commits en anglais (`type(scope): subject`), branches
  `type/short-description`.
- `pre-commit run --all-files` avant de pousser.
- Livraison : branche, PR vers `main`, merge en rebase (seule méthode permise
  par le ruleset, historique linéaire) dès que la CI est verte, sans relecture
  obligatoire. Les dépôts firmware (`minipock_zephyr-demo`,
  `minipock_mbed-rbdc`) suivent une autre règle : toute modification passe par
  une PR relue par un collègue avant merge.
- Les versions suivent `minipock/package.xml` ; un push sur `main` crée le tag.

## Suivi Jira (géré par l'agent, sans demande)

Espace Jira `RD` (R&D CATIE, catie-aq.atlassian.net), epic RD-10 « MiniPock ».

- Structure : l'epic RD-10 = le projet MiniPock ; une tâche = un chantier de la
  feuille de route (refactor jusqu'à Nav2 en sim, Open-RMF, passage au réel,
  cartographie collective…) ; une sous-tâche = une unité de travail, en général
  une PR. Le titre d'une sous-tâche décrit le travail, jamais « PR 12 : … » ; le
  lien de la PR va dans sa description.
- Ces règles valent pour chaque membre de l'équipe qui travaille ici avec Claude
  Code. Le suivi est tenu par l'agent, personne n'a à le faire à la main :
  - un chantier commence sans tâche : tâche créée sous RD-10, assignée à la
    personne qui mène la session (compte Jira connecté) ;
  - un travail commence sans sous-tâche : sous-tâche créée sous la tâche du
    chantier, même assignation ;
  - travail commencé → En cours ; PR ouverte → commentaire avec le lien ;
    PR mergée → Terminé ;
  - tâche parente En cours dès qu'une sous-tâche l'est, Terminé quand le
    chantier est livré.
  Commentaire court et factuel à chaque changement, jamais de code ni de logs.
- Tickets assignés à un autre membre de l'équipe : ni statut, ni assignation,
  ni mention @ ; seulement un commentaire factuel quand du travail mergé les
  concerne.
