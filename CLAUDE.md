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
| `minipock_bringup` | Launchs d'entrée `sim.launch.py` et `real.launch.py`, fichiers de flotte (`config/minipocks.yaml` par défaut en sim, `config/minipocks_real.yaml` pour le robot réel), chargement et validation de la config, rendu des paramètres Nav2 et localisation de la flotte ; robot réel : agent micro-ROS, mise en forme odométrie et scan (`raw_data_transformer`) |
| `minipock_navigation/*` | Nav2 (`minipock_navigation2`), Cartographer, téléopération |
| `minipock_fleet_adapter` | Adaptateur Open-RMF (chantier séparé, ne pas toucher hors de ce chantier) |
| `minipock_firmware_updater` | Mise à jour du firmware par ROS |

## Environnement

Tout se fait dans les images Docker du dépôt (`.devcontainer/Dockerfile`,
cibles `base`, `simulation`, `real`), publiées sur
`ghcr.io/catie-aq/minipock:<version>-<cible>`.

- `scripts/docker-run.sh [--gpu] [--gui] "<commande>"` lance une commande dans
  l'image `simulation` avec le dépôt monté sur `/workspaces/src/minipock` et un
  espace de travail colcon (`build/`, `install/`, `log/`) sur `/workspaces` :
  `$MINIPOCK_WS`, par défaut `../ws` à côté du dépôt (un `MINIPOCK_WS` par
  worktree). `--gpu` active le GPU NVIDIA, `--gui` l'affichage X11 (RViz).
  Image : `IMAGE`, par défaut `ghcr.io/catie-aq/minipock:<version>-simulation`
  (version de `minipock/package.xml`), `minipock:dev-simulation` après un build
  local. Chaque dépôt ou worktree reçoit son `ROS_DOMAIN_ID` et son
  `GZ_PARTITION`, découverte limitée à localhost. `scripts/gate.sh [<ref>]`
  rejoue la CI sur un commit (export propre, pre-commit sans sous-module,
  build, tests unitaires). Les scripts passent `shellcheck` (pre-commit).
- Compiler avec `--merge-install` : les chemins Gazebo de l'image
  (`GZ_SIM_RESOURCE_PATH`) supposent un `install/share` unique.
- Sans écran ni GPU (CI, conteneur sans `/dev/dri`) : `LIBGL_ALWAYS_SOFTWARE=1`
  et `gz sim -s --headless-rendering`. Le LiDAR simulé (`gpu_lidar`) tourne
  alors en rendu logiciel, plus lentement que le temps réel.
- `minipock_description/launch/description.launch.py` prend, en plus de
  `namespace`, `mode` et `use_sim_time`, l'argument propre à la sim
  `sim_index` (rang du robot dans la flotte, 0 à 17, unique par simulation).
  L'origine du `gpu_lidar` est dans le maillage du
  LD19 : son visuel porte le bit de visibilité Gazebo `1 << (2 + sim_index)`,
  que le masque du LiDAR du même robot exclut. Chaque LiDAR ignore ainsi son
  propre boîtier mais voit celui des autres robots ; deux robots au même
  `sim_index` ne voient pas leurs LD19 respectifs. Le launch d'entrée
  `sim.launch.py` fournit un `sim_index` unique par robot (son rang dans
  `robots` du fichier de flotte) et refuse une flotte de plus de 18 robots ;
  en réel, `sim_index` reste à son défaut, sans effet. Aucune pièce du châssis
  ne coupe le plan du LiDAR (`test_scan_plane_clear_of_robot_body`) : l'absence
  de filtre d'empreinte repose sur la hauteur de l'URDF, à confirmer sur le
  robot réel.
- Garde au sol (`minipock_description/test/test_urdf.py`) : bas des roues à
  0,0975 m sous `base_link` (plan de contact), collision du châssis 1 cm
  au-dessus. Le différentiel a deux patins sans frottement, avant et arrière,
  1,5 mm au-dessus du plan comme les patins PLA du robot réel : il penche de
  0,7° au plus et le châssis ne frotte jamais le sol. `minipock_gz` pose le
  robot 2,5 mm au-dessus du sol (`SPAWN_HEIGHT`, `test_spawn_height.py`) : plus
  bas, roues et châssis démarrent dans le maillage du sol et y restent
  enfoncés, et le frottement du châssis absorbe les petites consignes de
  `VelocityControl` (holonome à l'arrêt).
- Écart connu de la sim : l'odométrie holonome (`OdometryPublisher` de gz-sim)
  part de l'origine du monde, alors que le firmware part de zéro (les options de
  décalage n'existent qu'après gz-sim 8.11.0). Sans effet sur Nav2, qui corrige
  par `map -> odom`.
- `ROS_DOMAIN_ID` : 10 sur le réseau du robot. Pour un test local, prendre un
  autre domaine et `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` pour ne pas
  parler aux robots réels.

```bash
colcon build --symlink-install --merge-install
```

Simulation de la flotte (monde, robots, localisation, Nav2, RViz) :

```bash
ros2 launch minipock_bringup sim.launch.py [config:=<fichier>] [headless:=true] [rviz:=false]
```

Flotte réelle (agent micro-ROS, mise en forme des données, localisation, Nav2) :

```bash
ros2 launch minipock_bringup real.launch.py [config:=<fichier>] [rviz:=true]
```

- `config:=` : chemin, ou nom d'un fichier de `minipock_bringup/config`
  (défaut `minipocks.yaml` en sim, `minipocks_real.yaml` en réel). Le fichier
  est lu une fois par le launch d'entrée, qui inclut pour chaque robot
  `minipock_description/launch/description.launch.py`, puis
  `minipock_gz/launch/robot.launch.py` (sim) ou
  `minipock_bringup/launch/robot.launch.py` (réel), puis
  `minipock_navigation2/launch/robot.launch.py`.
- `use_sim_time` n'est pas dans le fichier de flotte : le launch d'entrée le
  fixe (`true` en sim, `false` en réel) par `SetParameter` pour tous les nœuds,
  et le passe aussi aux launchs par robot.
- RViz : `minipock_navigation2/rviz/<rviz>.rviz`, `<rviz>` étant le champ du
  fichier de flotte (`fleet` dans les deux fichiers livrés).

Nav2 d'un robot (localisation lancée à part, voir « Navigation ») :

```bash
ros2 launch minipock_navigation2 robot.launch.py namespace:=minipock_0 params_file:=<fichier rendu> use_sim_time:=true
```

Cartographer pour un robot (carte sur `/minipock_0/map`) :

```bash
ros2 launch minipock_cartographer cartographer.launch.py namespace:=minipock_0 use_sim_time:=true
```

## Mondes de simulation

- Mondes dans `minipock_gz/worlds/` (argument `world` de `world.launch.py`, sans
  `.sdf`) : `office` (défaut de `minipocks.yaml`, bureaux meublés tirés de
  `husarion_office` de Husarion), `empty_room_world`, `minipock_world`. Carte
  Nav2 de même nom dans `minipock_navigation/minipock_navigation2/map/`.
- Le monde `office` ne charge rien par le réseau : ses modèles sont copiés dans
  `minipock_gz/models/` (ajouté à `GZ_SIM_RESOURCE_PATH` par `world.launch.py`).
  Origine et licence de chaque modèle dans `minipock_gz/worlds/ATTRIBUTION.md`,
  vérifiées par `minipock_gz/test/test_office_world.py` : tout modèle ajouté
  doit y figurer. Pas d'URI `https://fuel...` dans un monde.
- Positions de départ de `minipocks.yaml` : dans l'open space, cellules libres
  dans un rayon de 0,25 m (testé sur la carte).
- `world_to_map.py` : la cellule i de la carte couvre
  `[origin + i·res, origin + (i+1)·res)` et une cellule est occupée dès qu'un
  obstacle la touche (les vitres de 1 cm restent dans la carte). Pas de
  décalage d'une demi-cellule (`test_world_to_map.py`) : avec, chaque obstacle
  glissait vers -x et -y et l'AMCL se recalait à plusieurs centimètres de la
  position réelle.
- Après toute modification du monde, régénérer la carte (sur l'hôte, depuis la
  racine du dépôt) ; `--z-max` = hauteur du LiDAR au-dessus du sol (0,1475 m :
  `lidar_link` à 0,05 m au-dessus de `base_link`, bas des roues à 0,0975 m en
  dessous, d'après l'URDF) + 0,05 m :

```bash
uv run minipock_gz/scripts/world_to_map.py --world minipock_gz/worlds/office.sdf \
  --models minipock_gz/models --z-min 0.02 --z-max 0.1975 --resolution 0.05 \
  --out minipock_navigation/minipock_navigation2/map/office
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
  `map`) ; chaque robot a son AMCL (`/<ns>/amcl`, `map_topic: /map`). Ces
  actions viennent de `minipock_bringup.localization.localization_actions`.
- Un gestionnaire de cycle de vie par namespace, bond actif :
  `lifecycle_manager_map` (racine) pour `map_server`,
  `<ns>/lifecycle_manager_localization` pour l'AMCL du robot. Un gestionnaire
  ne crée de bond qu'avec les serveurs de son namespace (le serveur publie sur
  `<ns>/bond` sous son nom court).
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
- MPPI : `ax_min = -ax_max` et `ay_min = -ay_max`
  (`test_mppi_acceleration_symmetric`). MPPI borne chaque commande tirée à
  `[v + ax_min·dt, v + ax_max·dt]` autour de la vitesse mesurée `v` ; avec une
  décélération plus forte que l'accélération, la moyenne retombe sous `v` à
  chaque cycle et le robot reste à l'arrêt (« Failed to make progress »).
- Velocity smoother en boucle ouverte (`test_velocity_smoother_open_loop`) :
  en boucle fermée, sa sortie est la vitesse mesurée plus une variation
  bornée, jamais ramenée sous `max_velocity`. L'`OdometryPublisher` de gz-sim
  8.11 (holonome) publie environ 62 rad/s pendant 0,1 s chaque fois que le cap
  passe -2π/3 (changement de signe du quaternion de la physique) : en boucle
  fermée le robot recevait jusqu'à 50 rad/s.
- `docking_server` (géré par `navigation_launch.py`) a un plugin
  `SimpleNonChargingDock` et aucun dock : il s'active sans servir.
- `minipock_navigation2/rviz/fleet.rviz` montre `minipock_0` et `minipock_1`
  (un groupe d'affichages par robot) : à compléter pour une flotte plus grande.
- Cartographer : `cartographer.launch.py namespace:=<ns>` génère le lua (et la
  config RViz) avec le namespace à la place de `__NS__` ; il ne publie que
  `map → <ns>/odom`.

## Robot réel

- Le firmware publie `odom_raw` (`PoseStamped`) et `scan_raw` (`LaserScan`) en
  micro-ROS (un seul agent UDP, port 8888, lancé par `real.launch.py`).
  `raw_data_transformer` (`minipock_bringup/launch/robot.launch.py`, un par
  namespace) les remet en forme : `odom` (`Odometry`, vitesse calculée dans le
  repère du robot à partir de deux poses successives, nulle si les horodatages
  sont égaux ou à rebours), `scan` (horodatage du firmware gardé, frame
  remplacée par le paramètre `scan_frame_id`, défaut `<ns>/lidar_link`, car le
  firmware envoie encore `lds_01_link`, Jira RD-711) et le seul TF
  `<ns>/odom → <ns>/base_footprint`. `odom` et `scan` sont publiés en QoS
  capteur (`qos_profile_sensor_data`, best effort). Le même launch lance
  `joint_state_publisher` : le firmware ne publie pas l'état des roues.
- Topics, frames et types échangés avec le firmware forment un contrat :
  un changement ici qui le touche demande la modification correspondante dans
  le dépôt firmware, sinon le robot réel casse sans que la CI le voie.
- Rien ne se teste sur le robot réel sans sonde de programmation à portée.

## Tests

- Tests unitaires de tous les paquets, sans la simulation :

```bash
colcon test --merge-install --pytest-args -m "not integration" && colcon test-result --verbose
```

- Test d'intégration `minipock_bringup/test/test_sim_nav2.py` (marqueur pytest
  `integration`, `launch_testing`) : lance `sim.launch.py` sans interface avec
  `minipock_bringup/test/data/two_robots.yaml` (monde `office`, `minipock_0`
  holonome, `minipock_1` différentiel), vérifie `scan` et `odom`, l'arbre TF
  (`map -> <ns>/odom` publié par chaque AMCL), le `map_server` unique, puis
  envoie en parallèle un but `NavigateToPose` par robot (constantes `GOALS`,
  choisies sur `office.pgm`), attendu `SUCCEEDED` sous 300 s simulées. Les
  attentes comptent en temps simulé (`/clock`), avec un garde-fou en temps
  réel de 20 fois l'attente. Le marqueur est posé sur
  `generate_test_description` : le plugin pytest de `launch_testing` recopie
  les marqueurs de cette fonction, pas un `pytestmark` de module. Sans `-m`,
  `colcon test` lance aussi ce test.
- Lancer le test d'intégration en local, dans le conteneur de simulation
  (workspace compilé). Avec GPU, le conteneur reçoit `--gpus all
  -e NVIDIA_DRIVER_CAPABILITIES=all --device /dev/dri` (rendu matériel, temps
  réel, 1 à 3 min) ; sans GPU, `-e LIBGL_ALWAYS_SOFTWARE=1` (rendu logiciel du
  LiDAR, simulation bien plus lente que le temps réel, attentes inchangées en
  temps simulé). La commande est la même dans les deux cas ; `-s` dans
  `--pytest-args` affiche les logs de la simulation et le temps mis par chaque
  robot pour atteindre son but :

```bash
colcon test --merge-install --packages-select minipock_bringup \
  --pytest-args -m integration --event-handlers console_direct+
colcon test-result --verbose
```

- État connu : l'estimation AMCL s'écarte de 0,1 m environ de la position
  réelle pendant un déplacement, de l'ordre de la tolérance de but (correctif
  en cours).

## Skills

Les gestes répétitifs ont un skill de projet dans `.claude/skills/` :

- `minipock-test` : compiler et tester dans le conteneur (unitaires,
  `-m integration`, gate complet), lire `colcon test-result`, pièges connus
  (liens symboliques obsolètes après suppression d'un fichier, `--merge-install`,
  pre-commit sans le sous-module).
- `minipock-sim` : lancer la simulation (sans écran, GPU ou rendu logiciel, GUI),
  sonder topics, TF et états de cycle de vie, envoyer un `NavigateToPose`,
  arrêter proprement le groupe de processus.
- `minipock-release` : figer une version (versions des paquets, CHANGELOG, PR en
  rebase, tag posé par `ros-tag`, dossier de version dans
  `minipock_documentation`).

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
