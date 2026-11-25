# MiniPock : refactor de la base jusqu'à Nav2 en simulation multi-robot (design)

Chantier 1 de la reprise, taille L. Jira RD-895 (epic RD-10). Source : `revue-base-minipock.md`, sections 1 à 5 et 7. Branche locale `refactor/nav2-sim-base`, worktree `~/CATIE/RD/minipock/minipock-refactor`.

## Point de départ (vérifié le 25/11/2026)

- Image `ghcr.io/catie-aq/minipock:2.2.0-simulation` : les 11 paquets compilent, à condition d'utiliser `--merge-install` (les chemins Gazebo de l'image supposent un `install/share` unique).
- Sim sans écran ni GPU (`LIBGL_ALWAYS_SOFTWARE=1`, `gz sim -s --headless-rendering`) : 2 robots, scan à 7 Hz pour 10 Hz demandés, odométrie et TF présents.
- Nav2 (config passée en `use_sim_time: true`, carte `empty_room_map`) : les nœuds passent actifs et AMCL localise, mais un but envoyé à `minipock_1` échoue (« Failed to make progress » quatre fois, puis le behavior server cherche une frame `odom` sans préfixe).
- `holonomic.yaml` est identique à `differential.yaml` : le robot holonome tourne avec le modèle de mouvement différentiel et `max_vel_y: 0`. La costmap locale déclare deux fois `global_frame` (`odom` puis `map`), la seconde gagne.

## Périmètre

Dans le chantier : sections 1 à 5 et 7 de la revue. Hors chantier : `minipock_fleet_adapter` (Open-RMF), firmwares Zephyr et Mbed, `minipock_teleop` et `minipock_firmware_updater` (seulement leurs dépendances).

Livrables : une PR sur `catie-aq/minipock`, en commits séparés par lot, CI verte, test sim 2 robots + Nav2 qui passe, CLAUDE.md à jour ; une PR compatible sur `catie-aq/ros_workflows` pour les tests en CI ; une PR sur `catie-aq/minipock_documentation` pour `latest`.

## 1. Launches et configuration

Aujourd'hui chaque launch relit `minipocks.yaml` dans le dossier d'installation. Cible : un launch d'entrée par mode lit la config une fois, puis passe des valeurs simples à des launches par robot réutilisables seuls.

```
minipock_bringup/launch/sim.launch.py   config:=<yaml> rviz:=true headless:=false
├── minipock_gz/launch/world.launch.py                    world, headless
└── pour chaque robot de la flotte :
    ├── minipock_description/launch/description.launch.py namespace, mode, use_sim_time
    ├── minipock_gz/launch/robot.launch.py                namespace, mode, x, y, yaw
    └── minipock_navigation2/launch/robot.launch.py       namespace, mode, map, x, y, yaw, use_sim_time

minipock_bringup/launch/real.launch.py  config:=<yaml> rviz:=false
├── agent micro-ROS (une fois, UDP 8888)
└── pour chaque robot :
    ├── description.launch.py (use_sim_time:=false)
    ├── minipock_bringup/launch/robot.launch.py           namespace (raw_data_transformer)
    └── minipock_navigation2/launch/robot.launch.py
```

- Config : `minipock/minipocks.yaml` reste le fichier de flotte, au même endroit. Il devient la valeur par défaut de l'argument `config:=` des launches d'entrée, qui le lisent une seule fois ; on peut passer un autre fichier sans recompiler. `map:=` et `world:=` en argument surchargent la valeur du fichier (par exemple `map:=catie` en réel). Champs gardés : `namespace`, `world` (sim), `map`, `rviz`, `fleet.<id>.mode`, `fleet.<id>.position` (`[x, y]` ou `[x, y, yaw]` dans le repère `map`, `null` en sim pour le placement en spirale). Champs retirés : `bringup` et `use_sim_time`, que le launch d'entrée fixe selon le mode. Valeurs par défaut cohérentes pour la sim (`empty_room_world` et `empty_room_map`, au lieu de `catie` avec la salle vide aujourd'hui). Validation jsonschema complète dans `minipock_bringup.config`, avec message d'erreur lisible.
- `robot_description` vient de `Command(xacro)` dans `description.launch.py`. Le spawn Gazebo lit ce topic (`ros_gz_sim create -topic`), Gazebo convertit lui-même l'URDF : plus de `gz sdf -p`, plus d'écriture dans `install/share`, plus d'erreur avalée en réel.
- `world.launch.py` ajoute lui-même les dossiers des paquets à `GZ_SIM_RESOURCE_PATH` : le lancement ne dépend plus du `.bashrc` de l'image ni de `--merge-install`.
- Le bug `bool("False")` disparaît par construction : `use_sim_time` circule comme argument de launch booléen, sim à `true`, réel à `false`. Un test le vérifie.
- Supprimés : `minipock_description/config.py` (déplacé dans `minipock_bringup`) et `model.py`, `minipock_gz/launch/spawn.launch.py`, `minipock_gz/lidar_process.py`, `minipock_navigation2/launch/navigation2.launch.py` (450 lignes), `minipock_bringup/launch/bringup.launch.py`, relais `topic_tools` de Cartographer. Le métapaquet `minipock` reste.

## 2. TF et namespaces (décision A)

Nav2 Jazzy se lance sous namespace via `nav2_bringup`, mais `navigation_launch.py` et `localization_launch.py` remappent `/tf` vers `tf`, donc vers `/minipock_0/tf`. Deux façons de faire :

| | A1. Frames préfixées, `/tf` global (recommandé) | A2. TF par namespace, frames sans préfixe |
|---|---|---|
| Principe | Ce que fait le dépôt aujourd'hui. `SetRemap('/tf', '/tf')` avant l'include de `nav2_bringup` annule son remap (la règle globale passe en premier, la première qui correspond gagne). | Convention des exemples multi-robot de Nav2. |
| Arbre TF | Un seul : `map → minipock_<id>/odom → …` pour tous les robots. | Un arbre par robot, `map` dupliqué dans chacun. |
| RViz, Open-RMF, carto collective | Une seule vue de la flotte ; Cartographer multi-trajectoires (v2) lit un seul arbre. | Relais ou une instance par robot. |
| Robot réel | Rien à changer côté firmware. | `raw_data_transformer` et contrat firmware à changer. |
| Risque | Repose sur l'ordre des remaps de `launch_ros`. Vérifié dans l'image (nœud classique et nœud composable) ; le test d'intégration le couvre. | Aucun côté Nav2. |

## 3. Nav2

- Un fichier de paramètres par cinématique, refait à partir du `nav2_params.yaml` de Nav2 1.3.4 (version de l'image), plutôt que corriger l'existant. Clés sans namespace (`nav2_bringup` les range sous `root_key`), frames avec un marqueur `<robot_namespace>` remplacé par `ReplaceString`, pose initiale AMCL complète (x, y, yaw) par les marqueurs `<x>`, `<y>`, `<yaw>`.
- Différentiel : AMCL `DifferentialMotionModel`, DWB sans `vy`. Holonome : AMCL `OmniMotionModel`, DWB avec échantillons en `y`, velocity smoother sur les trois axes.
- Chaîne de vitesse de `nav2_bringup` : contrôleur → `cmd_vel_nav` → velocity smoother → `cmd_vel_smoothed` → collision monitor → `cmd_vel`. Le velocity smoother passe en boucle fermée (`CLOSED_LOOP`) puisque l'odométrie porte désormais la vitesse.
- Un `map_server` par robot (`/minipock_<id>/map`, frame `map` commune), ce que fait `localization_launch.py`. Une seule carte chargée plusieurs fois, sans conséquence en sim.
- `docking_server` fait partie des nœuds gérés par `navigation_launch.py` en Jazzy : configuration minimale pour qu'il s'active, sans dock déclaré.
- Cartographer : `cartographer.launch.py namespace:=minipock_<id>`, fichier lua généré au lancement avec les frames du robot, topics `scan` et `odom` relatifs. Pas de slam_toolbox.
- RViz : une config qui montre toute la flotte (arbre TF unique), lancée par `rviz:=true`.

## 4. Nœuds

- `raw_data_transformer` (réel seulement), réécrit :
  - entrée `odom_raw` (pose du firmware) ; sortie `odom` (`nav_msgs/Odometry`, pose et vitesse) et le TF `odom → base_footprint`, seul publieur de cette transformation ;
  - vitesse calculée par différence finie entre deux poses, exprimée dans `base_footprint`, nulle si l'intervalle de temps est nul ou négatif ;
  - retrait de `odom_optc`, de la fusion et de `filterpy` ;
  - scan : republié sans changer l'horodatage, `frame_id` réécrit en `<namespace>/lidar_link` tant que le firmware envoie `lds_01_link` (ticket existant RD-711) ;
  - calculs en fonctions pures, testées sans ROS.
- `lidar_process` supprimé, remplacé par `<gz_frame_id>` dans le capteur.
- Joint du LiDAR en `fixed`.

## 5. Fidélité de la simulation

- LiDAR simulé calé sur le firmware : 132 points par tour, 0,02 à 12 m, 10 Hz, bruit gaussien de 1 cm. L'orientation montée à l'envers (roll π) est gardée telle quelle, à vérifier sur le robot au chantier 3.
- Monde et carte cohérents dans chaque config (`sim.yaml` : `empty_room_world` + `empty_room_map` ; `real.yaml` : `catie`).

## 6. Dépendances, image, CI

- `package.xml` complets (ros_gz, nav2_bringup, xacro, robot_state_publisher, cartographer_ros, python3-jsonschema, python3-numpy, etc.) ; `rosdep install` dans l'image, sources montées en bind mount pendant le build. Le contexte Docker passe de `.devcontainer` à la racine (entrée `file:` du workflow partagé `docker-ghcr`). Le hack `EXTERNALLY-MANAGED` part ; un paquet pip sans clé rosdep passe par `pip install --break-system-packages` explicite.
- Image `real` : `micro_ros_setup` épinglé sur un commit.
- Workflow `docker.yaml` déclenché aussi par les `package.xml`.
- CI : faire évoluer le workflow partagé `catie-aq/ros_workflows/.github/workflows/ros2.yml` plutôt que d'en écrire un dans le dépôt. Aujourd'hui il force `skip-tests` et `skip-rosdep-install`, ne prend aucune entrée et compile Jazzy et Rolling. Il sert aussi à six dépôts C-Rob et à `stelia_aoa-ros-driver` : chaque nouvelle entrée garde par défaut le comportement actuel. Entrées ajoutées : `run-tests` (défaut `false`), `rosdep-install` (défaut `false`), `distributions` (défaut Jazzy et Rolling), `docker-image` (image de conteneur, défaut `osrf/ros:<distro>-desktop`), `packages` (défaut : nom du dépôt), `timeout-minutes`. PR séparée sur `ros_workflows`, avec sa doc `docs/ros2.md`, mergée avant celle de MiniPock. Côté MiniPock, deux appels : tests unitaires (Jazzy, `rosdep-install`, `run-tests`) et test d'intégration sim + Nav2 (`osrf/ros:jazzy-desktop-full`, timeout 30 min). Le job pre-commit partagé reste.
- pre-commit : `ruff` et `ruff format` (ligne 99) à la place de black ; `test_flake8.py` de `minipock_firmware_updater` supprimé.
- Dependabot : retrait de l'écosystème pip (aucun fichier de dépendances), ajout de `github-actions`, sous-module conservé.
- Hygiène : sous-module `minipock_msgs` en HTTPS ; les 25 Mo de PNG de la racine sont retirés (la doc a déjà ses propres images, le README pointe vers elle ; l'historique git les garde).

## 7. Tests

| Test | Type | Vérifie |
|------|------|---------|
| Config | pytest | Exemples valides, champs manquants ou faux refusés avec un message clair |
| Description | pytest | xacro → URDF valide pour les deux cinématiques, frames préfixées, joint LiDAR fixe, capteur à 132 points |
| `raw_data_transformer` | pytest | Vitesse par différence finie (translation, rotation, dt nul), frame du scan |
| Launches | pytest | `sim.launch.py` et `real.launch.py` se construisent depuis une config, `use_sim_time` correct dans chaque mode |
| Sim + Nav2 | `launch_testing`, sans écran | 2 robots (un de chaque cinématique), topics présents, TF `map → minipock_<id>/base_footprint`, but `NavigateToPose` atteint par chaque robot |

Risque principal : le rendu logiciel du `gpu_lidar` ralentit la sim en CI (facteur temps réel inférieur à 1). Parade : buts courts (environ 1,5 m), timeouts larges, temps simulé plutôt que temps mur dans les attentes.

## 8. Doc

La doc utilisateur reste dans `catie-aq/minipock_documentation`, publiée automatiquement (Docusaurus, un dossier par version, `latest` pour la version en cours). Une PR sur ce dépôt met à jour `latest` : pages Démarrage rapide, Simulation, Navigation, SLAM et Bringup réécrites pour les nouveaux launches, page multi-robot de la 2.1.0 remise (elle a disparu de `latest`), schéma d'archi (skill `stack-diagram`) et arbre TF multi-robot. Les dossiers de versions publiées ne bougent pas. Les deux PR (code et doc) se référencent.

Dans `minipock`, `docs/` ne contient que les documents de travail : design et plan dans `docs/superpowers/`.

## 9. Ordre des commits

1. Socle : dépendances, Dockerfile, ruff, hygiène.
2. Config et launches (description, gz, nav2, bringup, cartographer).
3. Nœuds : `raw_data_transformer`, LiDAR.
4. Sim calée sur le réel.
5. Tests et CI (après merge de la PR `ros_workflows`).

Puis PR de doc sur `minipock_documentation`.

Exécution par sous-agents, un par lot, sur la même branche et dans l'ordre (les lots 2 à 5 touchent les mêmes paquets). La PR `ros_workflows` se prépare en parallèle, dans son propre clone. Revue de code avant chaque PR.

## Décisions

- A. TF : frames préfixées sur `/tf` global (recommandé) ou TF par namespace. En attente.
- B. Config : `minipock/minipocks.yaml` gardé, lu une fois par le launch d'entrée (validé le 25/11).
- C. Doc : reste dans `minipock_documentation`, mise à jour de `latest` (validé le 25/11).
- D. ruff à la place de black et flake8. En attente.
- E. CI : évolution compatible de `ros_workflows` (validé le 25/11).
