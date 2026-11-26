---
name: minipock-release
description: Freeze and publish a new MiniPock version (version bump in every package.xml and setup.py, CHANGELOG, release PR, tag, release, documentation version folder in minipock_documentation). Use when asked to release, cut or freeze a version, bump the version, tag a release, or publish a documentation version of MiniPock.
---

# Release a MiniPock version

Reference: version 2.2.1 (`chore(release): bump version to 2.2.1`, then `docs: freeze documentation as version 2.2.1`). Versions follow SemVer; the reference is `minipock/package.xml`. Pushes and merges on `minipock` need no approval once CI is green; pushing to `catie-aq/ros_workflows` or any other shared repo does (ask first).

## 1. Bump on a branch

```bash
git switch -c chore/release-X.Y.Z origin/main
cur=$(sed -n 's|.*<version>\(.*\)</version>.*|\1|p' minipock/package.xml | head -1)
grep -rn "$cur" --include=package.xml --include=setup.py . | grep -v "minipock_msgs\|minipock_fleet_adapter\|minipock_firmware"
```

All packages carry the same version, enforced by `minipock_bringup/test/test_package_metadata.py`. Edit `<version>` in every `package.xml` and `version=` in every `setup.py` (`minipock_fleet_adapter`, `minipock_msgs` and `minipock_firmware` are excluded by the test and keep their own version). Check:

```bash
grep -rhn "<version>\|version=" --include=package.xml --include=setup.py . | sort | uniq -c
scripts/docker-run.sh "colcon build --symlink-install --merge-install --packages-select minipock_bringup && colcon test --merge-install --packages-select minipock_bringup --event-handlers console_direct+ --pytest-args -k package_metadata && colcon test-result --verbose"
```

## 2. CHANGELOG

In `CHANGELOG.md`, rename the `Unreleased` section (create it from the merged PRs if missing) to `## [X.Y.Z](https://github.com/catie-aq/minipock/releases/tag/X.Y.Z)`, with a one-line summary, then `### Added`, `### Changed`, `### Fixed`, `### Known issues` as needed. Keep `CLAUDE.md` accurate (version mentions, commands).

## 3. PR and merge

```bash
git commit -am "chore(release): bump version to X.Y.Z"
git push -u origin chore/release-X.Y.Z
gh pr create --fill
gh run list --branch chore/release-X.Y.Z --limit 1      # note the run id
gh run watch <run-id> --exit-status
gh pr merge --rebase
```

Wait on the run id with `gh run watch --exit-status`: `gh pr checks --watch` can return before the checks exist, and the ruleset has no required check, so a merge would go through on a red CI. Merge in rebase only (linear history).

## 4. Tag and release

The merge to `main` triggers `.github/workflows/ros2-tag.yaml` (`catie-aq/ros_workflows` `ros-tag.yml`, reads `minipock/package.xml`). If the version is new it creates the tag `X.Y.Z` and a draft release.

```bash
gh run list --workflow ros2-tag.yaml --limit 1
gh release list --limit 3
gh release view X.Y.Z
```

Review the draft and publish it (`gh release edit X.Y.Z --draft=false`) once the Docker workflow (`docker.yaml`, triggered by the tag and by `package.xml` changes) has pushed `ghcr.io/catie-aq/minipock:X.Y.Z-{base,simulation,real}`:

```bash
gh run list --workflow docker.yaml --limit 3
```

No tag after the merge: the version was not changed in `minipock/package.xml`, or the tag already exists.

## 5. Documentation version (repo `minipock_documentation`)

Do it once the mechanics, firmware and ROS blocks are published; `latest/` is the working copy, `X.Y.Z/` is frozen. In the `minipock_documentation` clone, on a branch:

```bash
cp -r latest X.Y.Z
sed -i 's/^# latest$/# X.Y.Z/' X.Y.Z/index.md      # heading of the frozen index
```

Check `head X.Y.Z/index.md` afterwards (the heading must read `# X.Y.Z`). Then, in the root `index.md`, add a `### X.Y.Z` section at the top of the changelog (newest first): a table of the versions of the three blocks (mechanics, firmware, ROS packages) with their links, then the list of changes (copy the shape of the 2.2.1 entry). Commit `docs: freeze documentation as version X.Y.Z`, merge, then tag that commit:

```bash
git tag X.Y.Z <commit> && git push origin X.Y.Z
```

The site is published from the default branch a few minutes after the push. Never edit a frozen folder afterwards.

## Pitfalls

- Frozen docs: edits go to `latest/` only.
- The Zephyr firmware has its own release (`vX.Y.Z` tag, draft release to publish, signed `.bin`); do not mix it with this one.
