#!/usr/bin/env bash
# Run a command in the MiniPock simulation image with this repository mounted.
#
# Usage: scripts/docker-run.sh [--gpu] [--gui] [--print] "<command>"
#
#   --gpu    NVIDIA GPU (hardware rendering in Gazebo); needs the NVIDIA container toolkit.
#   --gui    X11 forwarding (RViz, Gazebo GUI); run `xhost +local:` once on the host if needed.
#   --print  print the docker command instead of running it.
#
# The ROS environment and, once built, the workspace (install/setup.bash) are sourced first.
#
# Environment:
#   IMAGE         image to use (default ghcr.io/catie-aq/minipock:<version>-simulation, with
#                 <version> read from minipock/package.xml; `minipock:dev-simulation` after a
#                 local `docker build --target simulation -f .devcontainer/Dockerfile`).
#   MINIPOCK_WS   colcon workspace (build/install/log) mounted on /workspaces
#                 (default ../ws, next to the repository).
#   MINIPOCK_SRC  source tree mounted on /workspaces/src/minipock (default: this repository).
#   ROS_DOMAIN_ID, GZ_PARTITION  default to a value derived from the source path, so that
#                 simulations started from different worktrees do not see each other.
#   DOCKER_EXTRA  extra `docker run` options (for example `-e LIBGL_ALWAYS_SOFTWARE=1`).
set -euo pipefail

usage() {
  sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//'
  exit "${1:-2}"
}

gpu=0
gui=0
print_only=0
while [ $# -gt 0 ]; do
  case $1 in
    --gpu) gpu=1 ;;
    --gui) gui=1 ;;
    --print) print_only=1 ;;
    -h | --help) usage 0 ;;
    --) shift; break ;;
    -*) echo "unknown option: $1" >&2; usage ;;
    *) break ;;
  esac
  shift
done
[ $# -gt 0 ] || usage

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
src=$(realpath "${MINIPOCK_SRC:-$repo}")
ws=$(realpath -m "${MINIPOCK_WS:-$repo/../ws}")

if [ -z "${IMAGE:-}" ]; then
  version=$(sed -n 's|.*<version>\(.*\)</version>.*|\1|p' "$src/minipock/package.xml" 2>/dev/null | head -n1)
  if [ -z "$version" ]; then
    echo "cannot read the version from $src/minipock/package.xml: set IMAGE" >&2
    exit 2
  fi
  IMAGE=ghcr.io/catie-aq/minipock:${version}-simulation
fi
if [ -z "${ROS_DOMAIN_ID:-}" ]; then
  ROS_DOMAIN_ID=$(($(printf '%s' "$src" | cksum | cut -d' ' -f1) % 80 + 20))
fi
GZ_PARTITION=${GZ_PARTITION:-$(basename "$src")}

# DOCKER_EXTRA holds several docker options: split it on spaces.
read -r -a extra <<<"${DOCKER_EXTRA:-}"
opts=(--rm --net=host --ipc=host
  -u "$(id -u):$(id -g)" -e HOME=/tmp/home
  -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" -e "GZ_PARTITION=$GZ_PARTITION"
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
if [ "$gpu" = 1 ]; then
  opts+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi
if [ "$gpu" = 1 ] || [ "$gui" = 1 ]; then
  [ -e /dev/dri ] && opts+=(--device /dev/dri)
  for group in render video; do
    gid=$(getent group "$group" | cut -d: -f3 || true)
    [ -n "$gid" ] && opts+=(--group-add "$gid")
  done
fi
if [ "$gui" = 1 ]; then
  opts+=(-e "DISPLAY=${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix)
fi
opts+=(-v "$ws:/workspaces" -v "$src:/workspaces/src/minipock" -w /workspaces)

cmd="mkdir -p /tmp/home && source /opt/ros/jazzy/setup.bash \
  && { [ ! -f install/setup.bash ] || source install/setup.bash; } && $*"
if [ "$print_only" = 1 ]; then
  printf '%q ' docker run "${extra[@]}" "${opts[@]}" "$IMAGE" bash -c "$cmd"
  echo
  exit 0
fi
mkdir -p "$ws"
exec docker run "${extra[@]}" "${opts[@]}" "$IMAGE" bash -c "$cmd"
