#!/bin/bash

set -euo pipefail

# The target workspace is only created after upstream dependencies are built.
target_path="$(realpath "${TARGET_WORKSPACE:-${TARGET_REPO_PATH:?}}")"
upstream_path="${BASEDIR:?}/${PREFIX:-}upstream_ws/src"
docker_ros_path="${target_path}/docker/docker-ros"

if [[ "${ENABLE_RECURSIVE_VCS_IMPORT:-true}" == "true" ]]; then
    python3 "${docker_ros_path}/docker/recursive_vcs_import.py" "$target_path" "$upstream_path"
elif [[ -f "${target_path}/${VCS_IMPORT_FILE:-.repos}" ]]; then
    vcs import --recursive "$upstream_path" < "${target_path}/${VCS_IMPORT_FILE:-.repos}"
fi

if [[ "${BUILD_REQUIRED_PACKAGES_ONLY:-true}" == "true" ]]; then
    python3 "${docker_ros_path}/docker/prune_packages.py" "$target_path" "$upstream_path"
fi
