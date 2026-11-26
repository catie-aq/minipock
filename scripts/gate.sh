#!/usr/bin/env bash
# Full gate on a commit: clean export, full build, unit tests and pre-commit, the way CI runs them
# (no submodule checkout for pre-commit; the minipock_msgs sources are copied for the build).
#
# Usage: scripts/gate.sh [<git-ref>]      (default HEAD)
# Environment: IMAGE (default minipock:dev-simulation), MINIPOCK_GATE_DIR (default ../ws-gate).
set -euo pipefail

ref=${1:-HEAD}
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
gate=$(realpath -m "${MINIPOCK_GATE_DIR:-$repo/../ws-gate}")
log=${TMPDIR:-/tmp}/minipock-gate-precommit.log

if [ -z "$(ls -A "$repo/minipock_msgs" 2>/dev/null)" ]; then
  echo "minipock_msgs is empty: run 'git submodule update --init' first" >&2
  exit 1
fi

git -C "$repo" worktree prune
rm -rf "$gate"
mkdir -p "$gate/src/minipock"
git -C "$repo" archive "$ref" | tar -x -C "$gate/src/minipock"
cp -r "$repo/minipock_msgs/." "$gate/src/minipock/minipock_msgs/"
rm -rf "$gate/src/minipock/minipock_msgs/.git"

# Pre-commit on a clean checkout without the submodule, as CI does.
checkout=$gate/pc-worktree
git -C "$repo" worktree add -q --detach "$checkout" "$ref"
if (cd "$checkout" && uvx pre-commit run --all-files >"$log" 2>&1); then
  echo "PRE-COMMIT OK (no submodule, like CI)"
else
  echo "PRE-COMMIT FAILED"
  tail -20 "$log"
fi
git -C "$repo" worktree remove --force "$checkout"

MINIPOCK_SRC="$gate/src/minipock" MINIPOCK_WS="$gate" IMAGE=${IMAGE:-minipock:dev-simulation} \
  "$repo/scripts/docker-run.sh" \
  "colcon build --merge-install --event-handlers console_cohesion- 2>&1 | tail -3 \
   && colcon test --merge-install --pytest-args -m 'not integration' \
        --event-handlers console_cohesion- >/dev/null 2>&1; \
   colcon test-result --all | tail -5; colcon test-result"
