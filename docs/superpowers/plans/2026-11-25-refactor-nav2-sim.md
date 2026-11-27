# Refactor MiniPock jusqu'à Nav2 en simulation : plan d'implémentation

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** une base ROS 2 propre où `ros2 launch minipock_bringup sim.launch.py` lance N robots dans Gazebo avec Nav2 par robot, et où un test sans écran prouve que deux robots (un de chaque cinématique) atteignent un but.

**Architecture:** la config de flotte (`minipock/minipocks.yaml`) est lue une seule fois par un launch d'entrée par mode (`sim`, `real`) de `minipock_bringup`, qui inclut des launches par robot à arguments simples (description, gz, nav2). Frames préfixées par robot sur le `/tf` global ; Nav2 via `nav2_bringup` sous namespace avec `SetRemap('/tf', '/tf')`.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic (ros_gz 1.0.7), Nav2 1.3.4, Cartographer, Python 3.12, pytest, launch_testing, ruff, Docker.

**Spec:** `docs/superpowers/specs/2026-11-25-refactor-nav2-sim-design.md` (à lire avant chaque tâche).

## Global Constraints

- Branche `refactor/nav2-sim-base`, dans un worktree dédié, rebasée sur `main` après le merge de la 2.2.1.
- Commits Conventional Commits en anglais, sans ligne `Co-Authored-By`. Code, commentaires et docstrings en anglais.
- `minipock_fleet_adapter` n'est pas modifié (ni code ni formatage). Firmware hors périmètre.
- Toutes les commandes ROS tournent dans le conteneur : `../ws/run.sh "<cmd>"` (script local, monte le worktree dans `/workspaces/src/minipock`, domaine 42, découverte localhost). Image : variable `IMAGE`, `ghcr.io/catie-aq/minipock:2.2.0-simulation` jusqu'à la fin de la tâche 1, puis `minipock:dev-simulation`.
- Build : `colcon build --symlink-install --merge-install` ; tests : `colcon test --merge-install --packages-select <pkg> --event-handlers console_direct+ && colcon test-result --verbose`.
- Sans écran ni GPU : `DOCKER_EXTRA="-e LIBGL_ALWAYS_SOFTWARE=1"`, `gz sim -s --headless-rendering`.
- Namespace par robot `<namespace><id>` (défaut `minipock_<id>`) ; frames `<ns>/odom`, `<ns>/base_footprint`, `<ns>/base_link`, `<ns>/lidar_link` ; frame `map` commune ; topics relatifs `cmd_vel`, `odom`, `scan`, `joint_states`, `robot_description`.
- LiDAR simulé : 132 points, 0,02 à 12 m, 10 Hz, bruit gaussien σ = 0,01 m.
- Limites réelles du robot (firmware `minipock_mbed-rbdc/src/robot_sixtron.cpp` et `robot_sixtron.h`) : vitesse linéaire max 2,0 m/s, accélération 0,7 m/s², décélération 3,4 m/s² ; vitesse angulaire max 3π rad/s, accélération angulaire π rad/s² ; entraxe différentiel 0,328 m ; rayon de roue 0,0355 m ; holonome : roues à 0,172 m du centre. Aucun paramètre Nav2 ni de simulation ne dépasse ces limites.
- Standard Python CATIE : ruff seul (remplace black, isort, flake8), `line-length = 99`, `target-version = "py312"`, `[tool.ruff.lint] select = ["ALL"]`, `ignore = ["T20", "S101", "D104", "COM812", "CPY001"]` (`COM812` : conflit documenté avec `ruff format` ; `CPY001` : en-tête de licence, incluse dans `ALL` depuis ruff 0.17, pas de politique d'en-tête dans le dépôt), `pydocstyle.convention = "google"`. Ignores par fichier seulement là où ROS l'impose : `INP001` pour `**/launch/*.py`, `**/setup.py`, `**/test/**` ; `PLR2004` et `D` dans `**/test/**`. Tests pytest avec couverture (`--cov`, paquet `python3-pytest-cov`). Outils lancés par `uvx` (`uvx ruff`, `uvx pre-commit`). Les paquets ROS gardent setuptools/colcon et rosdep pour les dépendances (pas de `uv` ni de `uv.lock` par paquet).
- CLAUDE.md tenu à jour au fil de l'eau : toute tâche qui change une commande, un launch, un argument, un emplacement de fichier ou une convention met à jour `CLAUDE.md` dans le même commit (consigne de l'équipe du 25/11).
- Licence : Apache-2.0 partout (identifiant SPDX `Apache-2.0`), validé par l'équipe le 25/11.
- Push et merge sur `minipock` autorisés sans relecture, CI verte avant merge. Push sur `catie-aq/ros_workflows` : demander l'accord de l'équipe.

## Review Focus

- Flotte mixte (une position donnée, une `null`) : le placement en spirale ne doit pas poser un robot sur un autre → test `test_spiral_avoids_explicit_positions` (tâche 2).
- Namespace avec `/` final ou majuscules : frames en `minipock_//odom` → refus à la validation, `test_namespace_pattern_rejected` (tâche 2).
- Horodatages d'odométrie égaux ou à rebours (Wi-Fi) : vitesse nulle, pas d'exception → `test_twist_zero_dt`, `test_twist_negative_dt` (tâche 6).
- Rotation qui traverse ±π : `wz` calculé sur la différence d'angle la plus courte → `test_twist_wraps_angle` (tâche 6).
- Holonome bridé à vy = 0 par un paramètre oublié : `test_holonomic_allows_lateral_velocity` (tâche 5).

---

### Task 1: Socle (dépendances, image, lint, hygiène)

**Files:**
- Modify: tous les `package.xml` sauf `minipock_fleet_adapter/package.xml`
- Modify: `.devcontainer/Dockerfile`, `.github/workflows/docker.yaml`, `.github/dependabot.yml`, `.pre-commit-config.yaml`, `.gitmodules`, `README.md`
- Create: `pyproject.toml` (config ruff seulement), `.dockerignore`
- Delete: `differnetiel.png`, `holonome.png`, `image.png`, `minipock_firmware_updater/test/test_flake8.py`

**Interfaces:**
- Produces: image locale `minipock:dev-simulation` (cible `simulation`) avec toutes les dépendances rosdep des paquets ; config ruff à la racine.

- [ ] **Step 1:** compléter les `package.xml` (`exec_depend`) avec ce que le code importe ou lance après refactor : `minipock_description` (xacro, robot_state_publisher, launch_ros), `minipock_gz` (ros_gz_sim, ros_gz_bridge, minipock_description), `minipock_navigation2` (nav2_bringup, navigation2, nav2_common), `minipock_cartographer` (cartographer_ros), `minipock_bringup` (python3-jsonschema, python3-yaml, python3-numpy, tf2_ros, nav_msgs, sensor_msgs, micro_ros_agent seulement en commentaire car absent de rosdep), `minipock_teleop` (python3-pynput), `minipock_firmware_updater` (python3-jsonschema, python3-requests). `test_depend` : python3-pytest, python3-pytest-cov, launch_testing, launch_testing_ros. `crc` n'a pas de clé rosdep : il reste en pip.
- [ ] **Step 2:** Dockerfile. Stage `base` : `RUN --mount=type=bind,source=.,target=/tmp/src rosdep update && apt-get update && rosdep install --from-paths /tmp/src --ignore-src -y --skip-keys micro_ros_agent && rm -rf /var/lib/apt/lists/*` ; `pip install --break-system-packages --no-cache-dir crc==7.1.0` ; suppression de la ligne `EXTERNALLY-MANAGED` et de la liste apt redondante (garder `python3-colcon-common-extensions`, `ros-jazzy-rqt-tf-tree`, `ros-jazzy-rmf-dev`). Stage `real` : `micro_ros_setup` cloné puis `git checkout` d'un SHA fixe de la branche `jazzy` (noter le SHA en commentaire). Stage `simulation` : `gz-harmonic` vient des paquets ROS (`ros-jazzy-ros-gz` via rosdep), retirer le dépôt apt OSRF ; garder les `GZ_SIM_SYSTEM_PLUGIN_PATH` RMF. `.dockerignore` : `.git`, `build`, `install`, `log`, `**/*.png`.
- [ ] **Step 3:** `docker.yaml` : entrées `Dockerfile: .` et `file: .devcontainer/Dockerfile` pour les trois cibles ; déclencheur `paths` élargi à `**/package.xml`.
- [ ] **Step 4:** construire l'image : `docker build -f .devcontainer/Dockerfile --target simulation -t minipock:dev-simulation .` (en arrière-plan). Attendu : succès. Puis `IMAGE=minipock:dev-simulation ws/run.sh "rosdep check --from-paths src --ignore-src --skip-keys micro_ros_agent"` → `All system dependencies have been satisfied`.
- [ ] **Step 5:** lint. `pyproject.toml` `[tool.ruff]` selon les contraintes globales, `extend-exclude = ["minipock_fleet_adapter"]`. `.pre-commit-config.yaml` : retirer black, ajouter `astral-sh/ruff-pre-commit` (`ruff` avec `--fix`, `ruff-format`). Lancer `uvx pre-commit run --all-files` jusqu'à vert. Le code qui sera réécrit ou supprimé aux tâches 2 à 6 (`model.py`, `config.py` de description, `lidar_process.py`, `navigation2.launch.py`, `bringup.launch.py`, `spawn.launch.py`, `raw_data_transformer.py`) peut recevoir un `# noqa` de fichier temporaire (`# ruff: noqa`) plutôt qu'une mise en conformité ; le reste (teleop, firmware_updater, bridges, cartographer) est mis en conformité (docstrings, annotations).
- [ ] **Step 6:** hygiène. `.gitmodules` en `https://github.com/catie-aq/minipock_msgs` ; dependabot sans `pip`, avec `github-actions` ; PNG supprimés ; README : une phrase, lien vers la doc, lien vers `CLAUDE.md` pour les contributeurs.
- [ ] **Step 7:** build complet dans la nouvelle image : `IMAGE=minipock:dev-simulation ws/run.sh "colcon build --symlink-install --merge-install"` → 11 paquets OK.
- [ ] **Step 8:** commit `build: install dependencies with rosdep and switch to ruff`.

### Task 2: Config de flotte

**Files:**
- Create: `minipock_bringup/minipock_bringup/config.py`, `minipock_bringup/test/test_config.py`, `minipock_bringup/test/data/` (YAML de test)
- Modify: `minipock/minipocks.yaml`, `minipock_bringup/setup.py`
- Delete: `minipock_description/minipock_description/config.py`

**Interfaces:**
- Produces (dans `minipock_bringup.config`) :
  - `class ConfigError(ValueError)`
  - `@dataclass(frozen=True) class Robot: name: str; mode: str; x: float; y: float; yaw: float; nav2_params: str` (`name` = `<namespace><id>`, `mode` ∈ `{"differential", "holonomic"}`, `nav2_params` = champ optionnel `fleet.<id>.nav2_params` du YAML, chaîne vide si absent : chemin ou nom de fichier de `minipock_navigation2/param`, résolu par la tâche 5)
  - `@dataclass(frozen=True) class FleetConfig: namespace: str; world: str; map: str; rviz: str; robots: tuple[Robot, ...]`
  - `def load_config(path: str) -> FleetConfig`
  - `def resolve_config_path(value: str) -> str` : chemin existant → tel quel ; sinon nom de fichier cherché dans le share de `minipock` (`minipocks.yaml`, `minipocks_real.yaml`) ; sinon `ConfigError` qui liste les fichiers installés
  - `def default_config_path() -> str` (share de `minipock`, `minipocks.yaml`)

- [ ] **Step 1: tests qui échouent** (`test_config.py`) :
  - `test_load_default_file` : `load_config(default_config_path())` → 2 robots, `robots[0].name == "minipock_0"`, `world == "empty_room_world"`, `map == "empty_room_map"`.
  - `test_position_with_yaw` : `position: [1.0, 2.0, 1.57]` → `yaw == 1.57` ; `[1.0, 2.0]` → `yaw == 0.0`.
  - `test_missing_mode_rejected`, `test_unknown_mode_rejected`, `test_position_wrong_length_rejected` → `ConfigError` dont le message nomme le robot fautif.
  - `test_namespace_pattern_rejected` : `minipock/` et `Minipock_` refusés (motif `^[a-z][a-z0-9_]*$`).
  - `test_removed_fields_rejected` : `bringup` ou `use_sim_time` présents → `ConfigError` qui dit qu'ils sont fixés par le launch d'entrée.
  - `test_resolve_installed_name` : `resolve_config_path("minipocks_real.yaml")` → chemin existant ; `load_config` dessus → `map == "catie"`.
  - `test_resolve_unknown_name` : `resolve_config_path("nope.yaml")` → `ConfigError` qui cite `minipocks.yaml`.
  - `test_spiral_for_null_positions` : 3 robots `position: null` → positions distinctes, premier à `(0, 0)`.
  - `test_nav2_params_optional` : sans le champ → `nav2_params == ""` ; `nav2_params: holonomic_slow.yaml` → valeur gardée telle quelle.
  - `test_spiral_avoids_explicit_positions` : un robot explicite en `(0, 0)` et un `null` → le second n'est pas à moins de 0,5 m du premier.
- [ ] **Step 2:** lancer, attendu : échec à l'import.
- [ ] **Step 3:** implémenter. Schéma jsonschema complet (`additionalProperties: false` au niveau racine et robot ; `world`, `map`, `rviz` requis ; `fleet` non vide). Spirale reprise de `minipock_gz/launch/spawn.launch.py::generate_spiral_positions`, en sautant les positions à moins de 0,5 m d'une position explicite. `minipocks.yaml` : retirer `bringup` et `use_sim_time`, `map: "empty_room_map"`, commentaires en anglais. Créer `minipock/minipocks_real.yaml` (même flotte, `map: "catie"`, `world` gardé pour le schéma, positions `[0.0, 0.0, 0.0]` avec un commentaire « start pose of the robot in the map ») et l'installer dans `minipock/CMakeLists.txt`. `setup.py` : installer `minipock_bringup/test/data` n'est pas nécessaire (tests lisent depuis le source).
- [ ] **Step 4:** tests au vert (`colcon test --packages-select minipock_bringup`).
- [ ] **Step 5:** commit `feat(bringup): load and validate the fleet configuration once`.

### Task 3: Description du robot

**Files:**
- Create: `minipock_description/launch/description.launch.py`, `minipock_description/test/test_urdf.py`
- Modify: `minipock_description/urdf/sensors.xacro`, `minipock_description/urdf/minipock.urdf.xacro`, `minipock_description/urdf/motor.xacro`, `minipock_description/setup.py`
- Delete: `minipock_description/launch/spawn.launch.py`, `minipock_description/minipock_description/model.py`

**Interfaces:**
- Produces: `description.launch.py` avec arguments `namespace` (sans `/`), `mode`, `use_sim_time` (`true`/`false`) ; lance `robot_state_publisher` dans `<namespace>`, `robot_description` = `Command(["xacro ", urdf, " namespace:=", namespace, "/ mode:=", mode])` enveloppé dans `ParameterValue(..., value_type=str)`. Publie `/<namespace>/robot_description`.

- [ ] **Step 1: tests qui échouent** (`test_urdf.py`, xacro appelé en sous-processus sur le fichier source, XML lu par `xml.etree`) pour `mode` ∈ {differential, holonomic} :
  - `test_links_prefixed` : tous les liens commencent par `minipock_0/`.
  - `test_lidar_joint_fixed` : `lidar_joint` de type `fixed`.
  - `test_lidar_matches_firmware` : `samples == 132`, `min == 0.02`, `max == 12.0`, `update_rate == 10`, bruit `gaussian` avec `stddev == 0.01`.
  - `test_lidar_frame_id` : `gz_frame_id == "minipock_0/lidar_link"`, topic `minipock_0/scan`.
- [ ] **Step 2:** lancer, attendu : échecs sur le joint et le capteur.
- [ ] **Step 2b: test qui échoue** `test_diff_drive_matches_firmware` (mode differential) : plugin `DiffDrive` `wheel_separation == 0.328`, `wheel_radius == 0.0355`, et distance en y entre les axes des deux roues (origines des joints `*_wheel_joint` composées avec celles des `motor_*_joint`) égale à 0,328 m à 1 mm près ; rayon des cylindres de collision des roues 0,0355.
- [ ] **Step 3:** aligner la géométrie différentielle sur le firmware dans `minipock.urdf.xacro` et `motor.xacro` (position des moteurs, rayon des roues, paramètres `DiffDrive`) ; modifier `sensors.xacro` (joint `fixed` sans `limit` ni `axis`, capteur selon les contraintes, `<gz_frame_id>${namespace}${name}_link</gz_frame_id>`, topic `${namespace}scan`) ; écrire `description.launch.py` ; nettoyer `setup.py` (entry points `minipock_generate` et `minipock_config` retirés, dépendance `jsonschema` retirée).
- [ ] **Step 4:** tests au vert ; `ws/run.sh "ros2 launch minipock_description description.launch.py namespace:=minipock_0 mode:=holonomic use_sim_time:=false & sleep 5; ros2 param get /minipock_0/robot_state_publisher use_sim_time"` → `False`.
- [ ] **Step 5:** commit `refactor(description): build robot_description with xacro at launch`.

### Task 4: Gazebo

**Files:**
- Create: `minipock_gz/launch/world.launch.py`, `minipock_gz/launch/robot.launch.py`, `minipock_gz/test/test_bridges.py`
- Modify: `minipock_gz/minipock_gz/bridges.py`, `minipock_gz/setup.py`, `minipock_description/urdf/minipock.urdf.xacro` (si un topic de plugin change)
- Delete: `minipock_gz/launch/spawn.launch.py`, `minipock_gz/minipock_gz/lidar_process.py`

**Interfaces:**
- Consumes: `/<namespace>/robot_description` (tâche 3).
- Produces:
  - `world.launch.py` : arguments `world` (nom sans `.sdf`), `headless` (`true` → `-s --headless-rendering`), lance `gz sim -r` via `ros_gz_sim/gz_sim.launch.py`, pont `/clock`, ajoute `<share>/minipock_gz/worlds` et le dossier parent des shares de `minipock_description` à `GZ_SIM_RESOURCE_PATH` (`AppendEnvironmentVariable`).
  - `robot.launch.py` : arguments `namespace`, `x`, `y`, `yaw` ; `ros_gz_sim create -topic /<ns>/robot_description -name <ns> -x -y -z 0.05 -Y <yaw>` ; un `parameter_bridge` par robot.
  - `minipock_gz.bridges` : `robot_bridges(namespace: str) -> list[Bridge]` (odom, cmd_vel, scan, joint_states, tf).

- [ ] **Step 1: test qui échoue** `test_robot_bridges` : pour `"minipock_1"`, les arguments contiennent `/model/minipock_1/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V` remappé vers `/tf`, `minipock_1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan`, `minipock_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist`, et aucune entrée `scan_raw`.
- [ ] **Step 2:** lancer, attendu : échec.
- [ ] **Step 3:** implémenter `robot_bridges`, les deux launches, supprimer `lidar_process` et son entry point.
- [ ] **Step 4:** tests au vert, puis contrôle manuel sans écran (`DOCKER_EXTRA="-e LIBGL_ALWAYS_SOFTWARE=1"`) : world + description + robot pour `minipock_0` holonome et `minipock_1` différentiel ; `ros2 topic hz /minipock_0/scan` > 0, `frame_id` du scan `minipock_0/lidar_link`, `tf2_echo minipock_1/odom minipock_1/lidar_link` répond. Sans `--merge-install` aussi (build dans un dossier séparé `ws_iso`) : le monde se charge.
- [ ] **Step 5:** commit `refactor(gz): spawn robots from robot_description and drop lidar_process`.

### Task 4c: Monde de bureaux (amendement validé par l'équipe le 25/11)

**Files:**
- Create: `minipock_gz/worlds/office.sdf`, `minipock_gz/models/` (modèles du monde, copiés), `minipock_gz/worlds/ATTRIBUTION.md`, `minipock_gz/scripts/world_to_map.py` (script `uv run` avec dépendances inline), `minipock_navigation/minipock_navigation2/map/office.pgm` et `office.yaml`, `minipock_gz/test/test_office_world.py`
- Modify: `minipock_gz/setup.py` (installer `models/` et le monde), `minipock_gz/launch/world.launch.py` (ajouter `share/minipock_gz/models` à `GZ_SIM_RESOURCE_PATH`), `minipock_bringup/config/minipocks.yaml` (défaut : monde `office`, carte `office`, deux robots placés dans l'open space), `CLAUDE.md`

**Interfaces:**
- Base : monde `husarion_office.sdf` de [husarion/husarion_gz_worlds](https://github.com/husarion/husarion_gz_worlds) (Apache-2.0, Gazebo Harmonic), commit noté dans `ATTRIBUTION.md`. Nom du monde dans le SDF : `office`.
- Aucune ressource chargée par le réseau : chaque URI `https://fuel.gazebosim.org/...` remplacée par un `model://` local ; modèles Fuel copiés avec leur licence (vérifiée modèle par modèle dans leur `metadata.pbtxt` ou leur page Fuel) ; modèles propres à Husarion copiés (Laptop, Surfaces, Kitchen, Tv, WaterDispenser, ConferenceTable, Door) ; retirés : robots ROSbot posés dans le monde et logo Husarion.
- `world_to_map.py --world <sdf> --models <dir> --z-min 0.02 --z-max <hauteur LiDAR + 0.05> --resolution 0.05 --out <prefix>` : coupe les géométries de collision (boîtes, cylindres, maillages via trimesh) dans la bande de hauteur, écrit `.pgm` (trinaire) et `.yaml` (origine en repère monde, donc repère `map` aligné sur la sim). La hauteur du LiDAR se lit dans l'URDF (composition des origines jusqu'à `lidar_link`).

- [ ] **Step 1: tests qui échouent** (`test_office_world.py`) : `test_world_has_no_network_uri` (aucune chaîne `http` dans `office.sdf` ni dans `models/**/*.sdf`) ; `test_every_model_uri_resolves` (chaque `model://X` correspond à un dossier `models/X` avec `model.config`) ; `test_every_model_has_license` (chaque modèle copié figure dans `ATTRIBUTION.md` avec sa licence) ; `test_map_matches_world` (`office.yaml` existe, résolution 0,05, origine cohérente avec l'emprise du monde) ; `test_default_spawns_are_free` (pour chaque robot de `minipocks.yaml`, toutes les cellules de `office.pgm` dans un rayon de 0,25 m autour de sa position sont libres).
- [ ] **Step 2:** importer et nettoyer le monde, copier les modèles, écrire `ATTRIBUTION.md`.
- [ ] **Step 3:** écrire `world_to_map.py`, générer la carte, choisir deux positions de départ dans l'open space.
- [ ] **Step 4:** tests au vert ; contrôle manuel sans écran : le monde se charge sans réseau (`--network none` sur le conteneur), deux robots apparaissent, facteur temps réel mesuré en rendu logiciel (noté dans le rapport) ; une capture d'écran du monde rendu (rendu hors écran ou GUI Gazebo avec `DISPLAY`) jointe au rapport, pour la doc.
- [ ] **Step 5:** commit `feat(gz): add an office world with its generated map`.

### Task 1b: Métadonnées des paquets (amendement du 25/11)

**Files:**
- Modify: tous les `package.xml` et `setup.py` sauf `minipock_fleet_adapter`, `CMakeLists.txt` des paquets ament_cmake

**Interfaces:**
- Produces: version unique pour tous les paquets du dépôt (celle de `minipock/package.xml`, 2.2.1), mainteneur `s.delpeuch@catie.fr`, licence `Apache-2.0` partout (décision de l'équipe du 25/11 ; `minipock_description` et `minipock_gz` déclaraient GPL-3.0-only dans `package.xml`), `setup.py` qui lisent version et licence de façon cohérente avec le `package.xml`.

- [ ] **Step 1: test qui échoue** `minipock_bringup/test/test_package_metadata.py` : pour chaque `package.xml` du dépôt hors `minipock_fleet_adapter` (lus depuis le source), même `<version>` que `minipock/package.xml`, `<license>` égal à `Apache-2.0` partout, `license="Apache-2.0"` dans chaque `setup.py`, e-mail mainteneur `s.delpeuch@catie.fr`. Lancer, attendu : échec.
- [ ] **Step 2:** aligner les métadonnées ; ajouter `ament_lint_auto` avec `ament_xmllint` en `test_depend` des paquets ament_cmake.
- [ ] **Step 3:** tests au vert, `colcon test` des paquets ament_cmake vert (xmllint).
- [ ] **Step 4:** commit `chore: align package metadata across the repository`.

### Task 2b: Config déplacée dans `minipock_bringup` (amendement validé le 25/11)

**Files:**
- Move: `minipock/minipocks.yaml` et `minipock/minipocks_real.yaml` → `minipock_bringup/config/`
- Modify: `minipock_bringup/setup.py` (installer `config/*.yaml` dans `share/minipock_bringup/config`), `minipock_bringup/minipock_bringup/config.py`, `minipock_bringup/test/test_config.py`, `minipock/CMakeLists.txt` (plus d'installation de YAML), `minipock/package.xml` si besoin

**Interfaces:**
- Produces: mêmes signatures qu'à la tâche 2 ; `default_config_path()` et `resolve_config_path()` cherchent dans `share/minipock_bringup/config`. `minipock` redevient un pur métapaquet.

- [ ] **Step 1:** adapter `test_resolve_installed_name`, `test_load_default_file` et `test_resolve_unknown_name` au nouvel emplacement (le message d'erreur liste les fichiers de `share/minipock_bringup/config`) ; ajouter `test_minipock_is_pure_metapackage` (le `CMakeLists.txt` de `minipock` n'installe aucun fichier YAML, lu comme texte depuis le source). Lancer, attendu : échecs.
- [ ] **Step 2:** déplacer les fichiers avec `git mv`, adapter `config.py`, `setup.py`, `CMakeLists.txt`.
- [ ] **Step 3:** tests au vert en ne construisant que `minipock_bringup` (`colcon build --packages-select minipock_bringup` puis tests) : plus besoin du paquet `minipock` pour les tests.
- [ ] **Step 4:** commit `refactor(bringup): own the fleet configuration files`.

### Task 3b: Roues holonomes sans galets articulés (amendement validé le 25/11)

**Files:**
- Modify: `minipock_description/urdf/motor.xacro`, `minipock_description/test/test_urdf.py`

**Interfaces:**
- Produces: modèle holonome avec une seule articulation continue par roue (`<ns><motor>_wheel_joint`), aucun lien ni joint `*_roller*`. Les galets restent visibles comme géométries `visual` fixes du lien de roue (mêmes maillages `roller.stl`, poses = composition de l'origine de l'ancien joint de galet et de l'origine visuelle de l'ancien lien) ; collision de la roue : un cylindre de rayon 0,0355 m (rayon firmware).

- [ ] **Step 1: tests qui échouent** (mode holonomic) : `test_holonomic_has_no_roller_joints` (aucun joint dont le nom contient `roller`, exactement 3 joints `continuous`) ; `test_holonomic_keeps_roller_visuals` (chaque lien de roue a 1 + 9 éléments `visual`) ; `test_holonomic_wheel_collision` (un seul cylindre de rayon 0,0355 par roue). Lancer, attendu : échecs.
- [ ] **Step 2:** réécrire la branche holonome de `motor.xacro` (une macro de galet visuel appelée 9 fois si elle évite la répétition), sans toucher la branche différentielle.
- [ ] **Step 3:** tests au vert (y compris ceux de la tâche 3) ; contrôle manuel : un robot holonome apparaît dans Gazebo sans erreur et se déplace latéralement sur `cmd_vel` (vy = 0,2 pendant 3 s, `odom` en y augmente d'au moins 0,3 m).
- [ ] **Step 4:** commit `refactor(description): model holonomic wheels without articulated rollers`.

### Task 5: Nav2 et Cartographer (amendée le 25/11 : carte unique, paramètres fusionnés)

**Files:**
- Create: `minipock_navigation2/launch/robot.launch.py`, `minipock_navigation2/param/common.yaml`, `minipock_navigation2/rviz/fleet.rviz`, `minipock_bringup/minipock_bringup/nav2_params.py`, `minipock_bringup/minipock_bringup/localization.py`, `minipock_bringup/test/test_nav2_params.py`
- Modify: `minipock_navigation2/param/differential.yaml` et `holonomic.yaml` (deviennent des surcouches), `minipock_cartographer/launch/cartographer.launch.py`, `minipock_cartographer/config/minipock_2d.lua`, `minipock_bringup/package.xml` (exec_depend `nav2_map_server`, `nav2_amcl`, `nav2_lifecycle_manager`, `minipock_navigation2`)
- Delete: `minipock_navigation2/launch/navigation2.launch.py`

**Interfaces:**
- Produces:
  - `minipock_bringup.nav2_params` : `merge_params(base: dict, overlay: dict) -> dict` (fusion récursive des dictionnaires, la surcouche écrase, les listes sont remplacées en bloc) ; `render_params_file(mode: str, namespace: str, x: float, y: float, yaw: float, override: str = "") -> str` : charge `param/common.yaml`, applique la surcouche `param/<mode>.yaml`, puis `override` si non vide (chemin existant ou nom de fichier de `minipock_navigation2/param`, sinon `ValueError`), remplace `<robot_namespace>`, `<x>`, `<y>`, `<yaw>`, écrit le résultat dans un fichier temporaire et renvoie son chemin.
  - `minipock_bringup.localization` : `localization_actions(robots: list[tuple[str, str]], map_yaml: str, *, use_sim_time: bool) -> list[launch.Action]`, où chaque tuple est `(namespace, params_file)` : un `map_server` global (nom `map_server`, topic `/map`, frame `map`), un nœud `nav2_amcl` par robot dans son namespace avec son fichier de paramètres et `map_topic: /map`, un `lifecycle_manager_map` global qui gère `map_server`, et par robot un `<ns>/lifecycle_manager_localization` qui gère son `amcl`, bonds actifs (décision du 25/11), et les `SetRemap` TF (`/tf`, `/tf_static`) autour des AMCL.
  - `minipock_navigation2/launch/robot.launch.py` : arguments `namespace`, `params_file` (chemin déjà rendu), `use_sim_time`, `autostart` (`true`), `use_composition` (`true`) ; dans un `GroupAction` : `SetRemap('/tf', '/tf')`, `SetRemap('/tf_static', '/tf_static')`, include de `nav2_bringup/launch/navigation_launch.py` avec `namespace:=<ns>`. Plus de `localization_launch.py`.
- Params : `common.yaml` porte tout ce qui est commun, clés racine sans namespace (`amcl:`, `controller_server:`…), frames `<robot_namespace>/base_footprint`, `<robot_namespace>/odom`, `global_frame: map`, `scan_topic: scan`, `odom_topic: odom` relatifs, AMCL `map_topic: /map`. Base : `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` (1.3.4), contrôleur MPPI, planificateur NavFn, `robot_radius: 0.17`, `laser_min_range: 0.02`, AMCL `max_beams: 66`, `docking_server` sans dock (valeur minimale qui configure, à vérifier), `velocity_smoother.feedback: CLOSED_LOOP`. Surcouches : différentiel `nav2_amcl::DifferentialMotionModel`, MPPI `motion_model: DiffDrive`, vy max 0 ; holonome `nav2_amcl::OmniMotionModel`, MPPI `motion_model: Omni`, vy max 0,5 m/s. Valeurs de départ communes, cohérentes entre MPPI, velocity smoother et behavior server : vx max 0,5 m/s (marche arrière 0,25), wz max 1,5 rad/s, accélérations 0,7 m/s² et 3,0 rad/s², décélérations 1,5 m/s² et 3,0 rad/s², tolérance de but 0,10 m et 0,15 rad, inflation 0,45 m. La tâche 7b les règle en simulation.
- Cartographer : argument `namespace` ; lua avec `__NS__` remplacé dans un `OpaqueFunction` qui écrit le fichier dans un dossier temporaire ; nœuds dans le namespace, mêmes `SetRemap` TF ; relais supprimés.

- [ ] **Step 1: tests qui échouent** (`test_nav2_params.py`, sur le résultat de `render_params_file` relu en YAML, pour `mode` ∈ {differential, holonomic}) :
  - `test_merge_overlay_wins` : `merge_params({"a": {"b": 1, "c": 2}}, {"a": {"b": 3}})` → `{"a": {"b": 3, "c": 2}}` ; `test_merge_keeps_common_keys` : une clé absente de la surcouche est conservée ; `test_merge_replaces_lists` : une liste de la surcouche remplace celle de la base.
  - `test_frames_are_prefixed` : `amcl.base_frame_id == "minipock_0/base_footprint"`, `local_costmap.global_frame == "minipock_0/odom"`, `global_costmap.global_frame == "map"`, `amcl.map_topic == "/map"`.
  - `test_motion_models` : modèle AMCL et `motion_model` MPPI selon la cinématique.
  - `test_holonomic_allows_lateral_velocity` : holonome `velocity_smoother.max_velocity[1] > 0` et MPPI `vy_max > 0` ; différentiel `max_velocity[1] == 0` et `vy_max == 0`.
  - `test_initial_pose_rendered` : `amcl.initial_pose` vaut les `x`, `y`, `yaw` passés, `set_initial_pose: true`.
  - `test_limits_within_robot_capabilities` : vitesses et accélérations du velocity smoother, de MPPI (`vx_max`, `vy_max`, `wz_max`, `ax_max`, `az_max`) et du behavior server ≤ limites réelles (contraintes globales).
  - `test_controller_and_smoother_agree` : `vx_max`, `vy_max`, `wz_max` de MPPI égaux à `velocity_smoother.max_velocity`.
  - `test_override_applied` : un fichier `override` qui change `controller_server.FollowPath.vx_max` l'emporte sur commun et cinématique ; nom inconnu → `ValueError`.
  - `test_localization_actions` : pour deux robots, exactement un `map_server`, deux nœuds `amcl` (namespaces `minipock_0` et `minipock_1`), un `lifecycle_manager` dont `node_names == ["map_server"]` pour le manager global et `["amcl"]` pour chaque manager de robot (inspection des actions `Node` renvoyées).
- [ ] **Step 2:** lancer, attendu : échecs.
- [ ] **Step 3:** écrire `common.yaml`, réduire les deux fichiers de cinématique à leurs surcouches, écrire `nav2_params.py`, `localization.py`, `robot.launch.py`, Cartographer.
- [ ] **Step 4:** tests au vert ; contrôle manuel : sim tâche 4 pour les deux robots + un petit launch de test (ou script `ros2 launch` en Python) qui appelle `localization_actions` et `robot.launch.py` ; `ros2 lifecycle get /minipock_1/bt_navigator` → `active`, `ros2 lifecycle get /map_server` → `active`, `tf2_echo map minipock_0/odom` et `map minipock_1/odom` répondent, `ros2 topic info -v /tf` ne montre aucun abonné sous `/minipock_*/tf`, `ros2 topic list | grep -c "/minipock_0/minipock_0"` → 0.
- [ ] **Step 5:** commit `refactor(navigation): share one map and merge Nav2 parameters per kinematics`.

### Task 6: Bringup (entrée sim et réel, `raw_data_transformer`)

**Files:**
- Create: `minipock_bringup/minipock_bringup/odometry.py`, `minipock_bringup/minipock_bringup/launch_args.py`, `minipock_bringup/launch/sim.launch.py`, `minipock_bringup/launch/real.launch.py`, `minipock_bringup/launch/robot.launch.py`, `minipock_bringup/test/test_odometry.py`, `minipock_bringup/test/test_launch_args.py`
- Modify: `minipock_bringup/minipock_bringup/raw_data_transformer.py`
- Delete: `minipock_bringup/launch/bringup.launch.py`

**Interfaces:**
- Consumes: `load_config`, `FleetConfig`, `Robot` (tâche 2) ; launches des tâches 3, 4, 5.
- Produces:
  - `odometry.py` : `@dataclass(frozen=True) class Pose2D: x: float; y: float; yaw: float` ; `yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float` ; `body_twist(prev: Pose2D, curr: Pose2D, dt: float) -> tuple[float, float, float]` (vx, vy dans le repère du robot à `prev.yaw`, wz ; `(0, 0, 0)` si `dt <= 0`).
  - `launch_args.py` : `robot_launch_arguments(robot: Robot, cfg: FleetConfig, use_sim_time: bool) -> dict[str, str]` (clés `namespace`, `mode`, `x`, `y`, `yaw`, `params_file` = `render_params_file(robot.mode, robot.name, robot.x, robot.y, robot.yaw, override=robot.nav2_params)` (tâche 5, `override` nommé), `use_sim_time` en `"true"`/`"false"`).
  - `sim.launch.py` : arguments `config` (défaut `minipocks.yaml`, résolu par `resolve_config_path`), `rviz` (`true`), `headless` (`false`) ; world, `localization_actions` (tâche 5) pour toute la flotte avec la carte du YAML, puis pour chaque robot description, gz robot, nav2 robot, avec `use_sim_time:=true` ; RViz `fleet.rviz` si `rviz`.
  - `real.launch.py` : `config` (défaut `minipocks_real.yaml`), `rviz` (`false`) ; agent micro-ROS `udp4 --port 8888` une fois ; `localization_actions` pour la flotte ; par robot description, `robot.launch.py`, nav2, `use_sim_time:=false`.
  - Launchs d'entrée : `SetParameter(name="use_sim_time", value=...)` en tête, pour tous les nœuds inclus.
  - Nœud `raw_data_transformer` : paramètres déclarés avec `ParameterDescriptor` (description, type) ; `scan` et `odom` publiés en `qos_profile_sensor_data` ; paramètres `scan_frame_id` (défaut `<namespace>/lidar_link`) ; entrées `odom_raw` (`PoseStamped`), `scan_raw` ; sorties `odom` (`Odometry` avec twist), `scan`, TF `<ns>/odom → <ns>/base_footprint`.

- [ ] **Step 1: tests qui échouent** :
  - `test_odometry.py` : `test_twist_straight_line` (prev (0,0,0), curr (0.1,0,0), dt 0.1 → (1.0, 0.0, 0.0)) ; `test_twist_in_body_frame` (yaw π/2, déplacement +y monde → vx > 0, vy ≈ 0) ; `test_twist_zero_dt` et `test_twist_negative_dt` → (0,0,0) ; `test_twist_wraps_angle` (yaw 3.1 → -3.1 en 0.1 s → wz ≈ +0.83, pas -61) ; `test_yaw_from_quaternion` (quaternion de π/2 → π/2).
  - `test_launch_args.py` : `test_use_sim_time_sim` → `"true"`, `test_use_sim_time_real` → `"false"`, `test_pose_forwarded` (x, y, yaw en chaînes).
- [ ] **Step 2:** lancer, attendu : échec à l'import.
- [ ] **Step 3:** implémenter `odometry.py`, `launch_args.py`, réécrire `raw_data_transformer` (sans `filterpy`, sans `odom_optc` ni fusion, sans réhorodatage du scan, un seul `TransformBroadcaster`), écrire les trois launches.
- [ ] **Step 4:** tests au vert ; `ws/run.sh "ros2 launch minipock_bringup real.launch.py --print"` sans erreur ; lancement manuel sans écran de `sim.launch.py headless:=true rviz:=false` : deux robots, Nav2 actif.
- [ ] **Step 5:** commit `refactor(bringup): add sim and real entry launches and rewrite raw_data_transformer`.

### Task 7: Test d'intégration sim + Nav2

**Files:**
- Create: `minipock_bringup/test/test_sim_nav2.py`, `minipock_bringup/test/data/two_robots.yaml`
- Modify: `minipock_bringup/package.xml` (test_depend `nav2_msgs`, `tf2_ros`), `minipock_bringup/setup.cfg` (marqueur pytest `integration`)

**Interfaces:**
- Consumes: `sim.launch.py` (tâche 6).
- Produces: test marqué `@pytest.mark.integration`, exclu des tests unitaires par `-m "not integration"`.

- [ ] **Step 1:** `two_robots.yaml` : `minipock_0` holonome et `minipock_1` différentiel aux positions actuelles de `minipocks.yaml`, monde `empty_room_world`, carte `empty_room_map`.
- [ ] **Step 2:** `test_sim_nav2.py` (`launch_testing`) : `generate_test_description` inclut `sim.launch.py config:=<two_robots.yaml> headless:=true rviz:=false`. Tests, attentes en temps simulé (`/clock`) :
  - `test_topics` : `/minipock_{0,1}/{scan,odom}` publiés sous 120 s.
  - `test_tf_tree` : `map → minipock_{0,1}/odom` publié par chaque AMCL (vérifier via `ros2 topic info -v /tf` ou l'`authority` du `tf2` que les deux AMCL publient) et `map → minipock_{0,1}/base_footprint` disponible sous 180 s.
  - `test_single_map_server` : un seul nœud `map_server` dans le graphe, `/map` publié.
  - `test_reach_goal` : un `NavigateToPose` par robot vers une cellule libre de `empty_room_map` à 1 à 1,5 m de sa position de départ (choisie en lisant la carte, notée en constante avec un commentaire), statut `SUCCEEDED` sous 300 s simulées, envoyés en parallèle.
- [ ] **Step 3:** lancer (`DOCKER_EXTRA="-e LIBGL_ALWAYS_SOFTWARE=1"`, `colcon test --packages-select minipock_bringup --pytest-args -m integration`) jusqu'au vert, trois passages de suite. Échec : `superpowers:systematic-debugging`, pas d'augmentation de timeout sans cause trouvée.
- [ ] **Step 4:** commit `test(bringup): navigate two robots in headless simulation`.

### Task 7b: Réglage de Nav2 en simulation

**Files:**
- Create: `minipock_bringup/minipock_bringup/nav_benchmark.py` (nœud), `minipock_bringup/test/data/benchmark_goals.yaml`
- Modify: `minipock_navigation2/param/differential.yaml`, `minipock_navigation2/param/holonomic.yaml`, `minipock_bringup/setup.py` (entry point `nav_benchmark`)

**Interfaces:**
- Consumes: `sim.launch.py` et le test d'intégration (tâche 7).
- Produces: `ros2 run minipock_bringup nav_benchmark --ros-args -p robots:="[minipock_0, minipock_1]" -p goals_file:=<yaml> -p repeat:=3` : envoie les buts du fichier à chaque robot, mesure pour chaque but succès, durée (temps simulé), rapport longueur du chemin / ligne droite, distance minimale aux obstacles (depuis `scan`), oscillations (changements de signe de `wz` par mètre), et écrit un tableau Markdown sur la sortie standard.

- [ ] **Step 1:** `benchmark_goals.yaml` pour `empty_room_map`, par robot : ligne droite 3 m, demi-tour sur place (même position, yaw + π), but derrière le robot, but à 0,3 m d'un mur, et pour l'holonome un déplacement latéral de 1 m sans rotation (yaw constant).
- [ ] **Step 2:** écrire `nav_benchmark`, le lancer sur les paramètres de la tâche 5, garder le tableau de référence.
- [ ] **Step 3:** régler les paramètres (MPPI : critics et poids, `time_steps`, `model_dt`, `batch_size` réduit si le facteur temps réel de la sim logicielle tombe sous 0,5 ; costmaps ; AMCL ; goal checker) jusqu'à : 100 % de succès sur 3 répétitions, aucun contact (distance min > 0,05 m), holonome qui fait le déplacement latéral sans tourner de plus de 0,2 rad, pas d'oscillation en fin de but. Ne jamais dépasser les limites réelles (contraintes globales) ; les tests de `test_nav2_params.py` restent verts.
- [ ] **Step 4:** consigner le tableau avant/après et les paramètres changés avec leur raison dans `docs/superpowers/specs/2026-11-25-nav2-tuning.md` (repris dans la doc utilisateur à la tâche 9).
- [ ] **Step 5:** le test d'intégration de la tâche 7 passe encore trois fois de suite.
- [ ] **Step 6:** commit `perf(navigation): tune Nav2 parameters against the real robot limits`.

### Task 8: CI (ros_workflows puis minipock)

**Files:**
- Dans un clone de `catie-aq/ros_workflows` (branche `feat/ros2-tests-and-rosdep`) : `.github/workflows/ros2.yml`, `docs/ros2.md`
- Dans `minipock` : `.github/workflows/ros2-build-test.yaml`

**Interfaces:**
- Produces (entrées de `ros2.yml`, défauts = comportement actuel) : `run-tests` (boolean, `false`), `rosdep-install` (boolean, `false`), `distributions` (string JSON, `'["jazzy","rolling"]'`), `docker-image` (string, vide = `osrf/ros:<distro>-desktop`), `packages` (string, vide = nom du dépôt), `pytest-args` (string, vide), `timeout-minutes` (number, `60`).

- [ ] **Step 1:** modifier `ros2.yml` : matrice construite depuis `distributions` (`fromJSON`), image, `action-ros-ci` avec `skip-tests: ${{ !inputs.run-tests }}`, `skip-rosdep-install: ${{ !inputs.rosdep-install }}`, `package-name: ${{ inputs.packages || github.event.repository.name }}`, `colcon-defaults` qui passe `pytest-args` à `colcon test`, `timeout-minutes` sur le job. Mettre à jour `docs/ros2.md` (tableau des entrées, point d'attention sur les tests retiré).
- [ ] **Step 2:** vérifier la compatibilité : un appel sans entrée produit la même matrice et les mêmes options qu'avant (lecture du YAML résolu avec `actionlint`).
- [ ] **Step 3:** STOP : demander l'accord de l'équipe pour pousser la branche et ouvrir la PR sur `ros_workflows`.
- [ ] **Step 4:** dans `minipock`, `ros2-build-test.yaml` avec deux jobs appelant `ros_workflows/.github/workflows/ros2.yml@<branche ou main>` : `unit` (`distributions: '["jazzy"]'`, `rosdep-install: true`, `run-tests: true`, `packages: "minipock minipock_bringup minipock_description minipock_gz minipock_navigation2 minipock_cartographer"`, `pytest-args: '-m "not integration"'`) et `integration` (`docker-image: osrf/ros:jazzy-desktop-full`, même paquets, `pytest-args: '-m integration'`, `timeout-minutes: 45`, variable `LIBGL_ALWAYS_SOFTWARE=1` via l'environnement du job si l'action le permet, sinon entrée `env` supplémentaire dans `ros2.yml`).
- [ ] **Step 5:** commit `ci: run unit and simulation tests through ros_workflows`.

### Task 9: Doc utilisateur complète et CLAUDE.md

**Files:**
- Modify: `CLAUDE.md` (relecture finale, il a été tenu à jour par chaque tâche), `CHANGELOG.md` (section `Unreleased`)
- Dans le dépôt `minipock_documentation`, dossier `latest/` seulement (les dossiers de version publiés ne bougent pas ; la 2.2.1 est figée) : toutes les pages existantes réécrites ou complétées, structure et numérotation gardées, plus une page « Conventions ROS » et la page multi-robot de la 2.1.0 reprise et mise à jour (numérotation décalée en conséquence) ; images dans `img/`.

Constat de départ (équipe, 25/11) : la structure est bonne, le contenu manque (4 850 mots en tout, `10-Mécanique.md` vide, `1-Vue-d'ensemble.md` à 131 mots).

Publication : la doc est publiée depuis la branche par défaut du dépôt `minipock_documentation`, dont toute l'arborescence est reprise. Barres latérales autogénérées par dossier : les sous-dossiers avec `_category_.json` sont pris en charge. Le contrôle qualité du site signale les titres génériques ou en `snake_case` sans espace, les dossiers à une seule page, les pages vides et les doublons de texte : à éviter.

Structure de `latest/` (validée par l'équipe le 25/11, deux publics) :
- `utilisateur/` (« je viens de recevoir mon MiniPock ») : présentation et caractéristiques ; premier démarrage (alimentation, Wi-Fi, IP, namespace par le shell ou l'interface web, image sur le PC, vérification) ; piloter le robot (téléopération clavier et FPS) ; cartographier son espace ; naviguer (carte, position initiale, buts dans RViz) ; plusieurs robots ; simuler sans robot ; mettre à jour le firmware ; dépannage.
- `developpeur/` : architecture (schéma, paquets, nœuds, flux, arbre TF) ; environnement de dev (Docker, devcontainer, colcon, tests, skills) ; conventions ROS ; configuration de flotte (schéma du YAML) ; simulation (mondes, ponts, sans écran) ; navigation (paramètres, surcouches, limites, tableau de réglage) ; système embarqué et contrat firmware (micro-ROS, topics, endpoints HTTP) ; mission Open-RMF ; contribuer et publier une version.
- Les sections « API Documentation » des pages utilisateur actuelles passent côté développeur ; rien n'est supprimé sans être redistribué.

- [ ] **Step 1:** inventaire page par page de `latest/` : ce qui est faux après le refactor, ce qui manque. Écrit dans le rapport de tâche avant toute rédaction.
- [ ] **Step 2:** schéma d'archi global (skill `stack-diagram`, PNG) : PC hôte (launch d'entrée, nœuds par robot, Nav2, localisation partagée, Gazebo ou agent micro-ROS), robot (firmware), arbre TF multi-robot.
- [ ] **Step 3:** rédaction en français, dans le format Docusaurus existant (front matter, `DocCardList`), chaque commande vérifiée dans l'image `minipock:dev-simulation` avant d'être écrite :
  - Vue d'ensemble : ce qu'est MiniPock, les deux cinématiques, l'architecture (schéma), les dépôts.
  - Installation : image Docker, build colcon, devcontainer.
  - Démarrage rapide : sim deux robots en trois commandes, envoi d'un but.
  - Bringup : `real.launch.py`, agent micro-ROS, YAML réel, poses de départ.
  - Simulation : `sim.launch.py`, mondes, sans écran, ajout d'un robot.
  - Navigation : architecture Nav2 par robot, carte partagée, paramètres (commun, surcouches, `nav2_params` par robot), limites du robot réel, tableau de réglage de la tâche 7b, envoi de buts.
  - SLAM : Cartographer par robot, sauvegarde de carte.
  - Multi-robot : namespaces, YAML de flotte, carte unique, TF.
  - Conventions ROS (nouvelle page) : renvoie au standard ROS de l'équipe pour les règles communes et ne garde que ce qui est propre à MiniPock : YAML de flotte, contrat avec le firmware, arbre TF de la flotte, choix de QoS, launches et leurs arguments.
  - Système embarqué, Endpoints, Mise à jour, Logiciel de mission : relus et corrigés là où le refactor les touche ; Mécanique : contenu minimal (dimensions, entraxe, rayon de roue, limites, lien vers les fichiers de conception), à partir des valeurs du firmware.
  - Changelog dans `index.md` : entrée de la prochaine version.
- [ ] **Step 4:** passage au skill `humanizer`, `uvx pre-commit run --files` sur les fichiers touchés (corriger aussi les erreurs markdownlint préexistantes de `latest/`).
- [ ] **Step 5:** commits `docs: update CLAUDE.md and changelog` (minipock) et `docs: rewrite latest documentation for the refactored stack` (doc, poussé sur `main` du dépôt de doc, modification autorisée par l'équipe).

### Task 9b: Skills de projet pour les gestes répétitifs

**Files:**
- Create: `scripts/docker-run.sh` (remplace le script local `ws/run.sh`, hors dépôt aujourd'hui : workspace configurable par variable, image par défaut `ghcr.io/catie-aq/minipock:<version>-simulation`, `minipock:dev-simulation` en local), `.claude/skills/minipock-test/SKILL.md`, `.claude/skills/minipock-sim/SKILL.md`, `.claude/skills/minipock-release/SKILL.md`
- Modify: `CLAUDE.md` (section qui liste les skills)

**Interfaces:**
- `minipock-test` : build et tests dans le conteneur (unitaires, ou `-m integration`), lecture des résultats `colcon test-result`, pièges connus (liens symboliques obsolètes après suppression de launch, `--merge-install`).
- `minipock-sim` : lancer la sim sans écran (N robots, YAML au choix), sonder topics, TF et Nav2, envoyer un but, arrêter proprement le groupe de processus.
- `minipock-release` : figer une version (bump de `minipock/package.xml` et des autres paquets, CHANGELOG, PR rebase, tag posé par `ros-tag`, dossier de version et tag dans `minipock_documentation`), comme fait pour la 2.2.1.

- [ ] **Step 1:** écrire les skills avec le skill `skill-creator` (description qui déclenche, étapes, commandes exactes, pièges) ; chaque commande citée est exécutée une fois pour vérification.
- [ ] **Step 2:** `CLAUDE.md` liste les skills et quand les utiliser.
- [ ] **Step 3:** commit `docs: add project skills for tests, simulation and releases`.

## Après les tâches

Revue de code de toute la branche (`code-review`), « À relire », contrôle du code par l'équipe, rebase sur `main`, push, PR, CI verte, merge.
