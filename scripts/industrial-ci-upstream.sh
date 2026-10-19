#!/bin/bash

# industrial_ci hook after upstream setup and before upstream rosdep and builds.
# An empty manifest activates the upstream phase; this hook imports the actual
# dependencies. Discover .repos files in the target and imported repositories
# recursively, or import only VCS_IMPORT_FILE when recursion is disabled.
# When enabled, the shared pruning helper removes upstream packages outside the
# target packages' recursive dependency closure. A target without ROS packages
# keeps all imported packages. Use the original target checkout because
# industrial_ci creates its target workspace only after building upstream.

set -euo pipefail

target_path="$(realpath "${TARGET_WORKSPACE:-${TARGET_REPO_PATH:?}}")"
upstream_path="${BASEDIR:?}/${PREFIX:-}upstream_ws/src"
docker_ros_path="${target_path}/docker/docker-ros"

if [[ "${ENABLE_RECURSIVE_VCS_IMPORT:-true}" == "true" ]]; then
    python3 "${docker_ros_path}/docker/recursive_vcs_import.py" "$target_path" "$upstream_path"
elif [[ -f "${target_path}/${VCS_IMPORT_FILE:-.repos}" ]]; then
    vcs import --recursive "$upstream_path" < "${target_path}/${VCS_IMPORT_FILE:-.repos}"
fi

if [[ "${ENABLE_BUILD_REQUIRED_PACKAGES_ONLY:-true}" == "true" ]]; then
    python3 "${docker_ros_path}/docker/prune_packages.py" "$target_path" "$upstream_path"
fi
