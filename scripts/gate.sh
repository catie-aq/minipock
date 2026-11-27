#!/usr/bin/env bash
# Full gate on a commit: clean export, pre-commit, full build and unit tests, the way CI runs them
# (pre-commit on a checkout without the submodule; the minipock_msgs sources are exported from the
# revision the commit pins). Exits non-zero if pre-commit fails, the build fails, a test fails, no
# test ran or colcon test itself exits non-zero (a package with no collected test fails like in CI).
#
# Usage: scripts/gate.sh [<git-ref>]      (default HEAD)
# Environment: IMAGE (default minipock:dev-simulation, the image built from .devcontainer/Dockerfile;
#              set IMAGE=ghcr.io/catie-aq/minipock:<version>-simulation to use the published one),
#              MINIPOCK_GATE_DIR (default ../ws-gate).
set -euo pipefail

ref=${1:-HEAD}
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
gate=$(realpath -m "${MINIPOCK_GATE_DIR:-$repo/../ws-gate}")
log=$(mktemp "${TMPDIR:-/tmp}/minipock-gate-precommit.XXXXXX")
checkout=$gate-pc  # outside the workspace: colcon would see its packages twice

cleanup() {
  git -C "$repo" worktree remove --force "$checkout" 2>/dev/null || true
  rm -rf "$checkout"
  git -C "$repo" worktree prune
  rm -f "$log"
}
trap cleanup EXIT

commit=$(git -C "$repo" rev-parse --verify "$ref^{commit}")
msgs_sha=$(git -C "$repo" ls-tree "$commit" minipock_msgs | awk '$2 == "commit" {print $3}')
if [ -z "$msgs_sha" ]; then
  echo "commit $commit does not pin a minipock_msgs submodule revision" >&2
  exit 2
fi
if ! git -C "$repo/minipock_msgs" cat-file -e "$msgs_sha^{commit}" 2>/dev/null; then
  echo "minipock_msgs revision $msgs_sha not found locally: run 'git submodule update --init'" >&2
  exit 2
fi

# Stale registrations from an interrupted run are cleared once their directory is gone.
rm -rf "$gate" "$checkout"
git -C "$repo" worktree prune
mkdir -p "$gate/src/minipock/minipock_msgs"
git -C "$repo" archive "$commit" | tar -x -C "$gate/src/minipock"
git -C "$repo/minipock_msgs" archive "$msgs_sha" | tar -x -C "$gate/src/minipock/minipock_msgs"

# Pre-commit on a clean checkout without the submodule, as CI does.
precommit=ok
git -C "$repo" worktree add -q --detach "$checkout" "$commit"
if ! (cd "$checkout" && uvx pre-commit run --all-files >"$log" 2>&1); then
  precommit=FAILED
  tail -20 "$log"
fi

# Build and tests in the image. Fails on a build error, a failed test or zero tests collected.
inner=$(cat <<'INNER'
set -euo pipefail
colcon build --merge-install --event-handlers console_cohesion- > build.log 2>&1 \
  || { tail -30 build.log; echo "GATE build: FAILED"; exit 1; }
echo "GATE build: ok"
test_rc=0
colcon test --merge-install --packages-select $ci_packages --pytest-args -m 'not integration' --event-handlers console_cohesion- \
  > test.log 2>&1 || test_rc=$?
result=$(colcon test-result --all 2>&1 || true)
echo "$result" | tail -8
summary=$(echo "$result" | grep -E '^Summary: [0-9]+ tests' | tail -1)
tests=$(echo "$summary" | sed -E 's/^Summary: ([0-9]+) tests.*/\1/')
errors=$(echo "$summary" | grep -oE '[0-9]+ errors?' | cut -d' ' -f1)
failures=$(echo "$summary" | grep -oE '[0-9]+ failures?' | cut -d' ' -f1)
bad=$(( ${errors:-0} + ${failures:-0} ))
if [ -z "$tests" ] || [ "$tests" -eq 0 ]; then echo "GATE tests: FAILED (no test ran)"; exit 1; fi
if [ "$bad" -gt 0 ]; then echo "GATE tests: FAILED ($bad errors or failures of $tests)"; exit 1; fi
# colcon test fails on a package without a collected test (pytest exit code 5), which writes no
# failing case: the summary alone would miss it, CI would not.
if [ "$test_rc" -ne 0 ]; then
  grep -E 'exited with code|Failed' test.log | tail -10
  echo "GATE tests: FAILED (colcon test exit $test_rc, for instance a package without a collected test)"
  exit 1
fi
echo "GATE tests: ok ($tests run)"
INNER
)
# Test the packages CI tests (its `packages` list), not every package of the tree: a package CI
# leaves out, without tests, would otherwise fail the gate on pytest exit code 5.
ci_packages=$(git -C "$repo" show "$commit:.github/workflows/ros2-build-test.yaml" \
  | awk '/^ *packages: >-/ {f=1; next} f && /^ *[a-z-]+:/ {f=0} f {printf "%s ", $1}')
if [ -z "$ci_packages" ]; then
  echo "no package list found in .github/workflows/ros2-build-test.yaml" >&2
  exit 2
fi
inner="ci_packages='$ci_packages'"$'\n'"$inner"
status=0
MINIPOCK_SRC="$gate/src/minipock" MINIPOCK_WS="$gate" IMAGE=${IMAGE:-minipock:dev-simulation} \
  "$repo/scripts/docker-run.sh" "$inner" || status=$?

echo "== gate summary for $commit: pre-commit $precommit; build and tests exit $status"
if [ "$precommit" != ok ] || [ "$status" -ne 0 ]; then
  exit 1
fi
