#!/bin/bash

set -e

ROOT_PATH="$(realpath "$(cd -P "$(dirname "${0}")" && pwd)"/..)"
source "${ROOT_PATH}/scripts/build.sh"
source "${ROOT_PATH}/scripts/utils.sh"


# check for required variables and set defaults for optional variables
TARGET="${TARGET:-run}"
PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
require_var "BASE_IMAGE"
require_var "IMAGE_NAME"
IMAGE_TAG="${IMAGE_TAG:-latest}"
[[ "${TARGET}" == *"run"* ]] && require_var "COMMAND"
DEV_IMAGE_NAME="${DEV_IMAGE_NAME:-${IMAGE_NAME}}"
DEV_IMAGE_TAG="${DEV_IMAGE_TAG:-${IMAGE_TAG}-dev}"

ADDITIONAL_DEBS_FILE="${ADDITIONAL_DEBS_FILE:-}"
ADDITIONAL_FILES_DIR="${ADDITIONAL_FILES_DIR:-}"
ADDITIONAL_PIP_FILE="${ADDITIONAL_PIP_FILE:-}"
BLACKLISTED_PACKAGES_FILE="${BLACKLISTED_PACKAGES_FILE:-}"
CUSTOM_SCRIPT_FILE="${CUSTOM_SCRIPT_FILE:-}"
DEV_IMAGE="${DEV_IMAGE_NAME}:${DEV_IMAGE_TAG}"
ENABLE_CONTINUE_BUILD_DESPITE_ERRORS="${ENABLE_CONTINUE_BUILD_DESPITE_ERRORS:-}"
ENABLE_CONTINUE_ROSDEP_INSTALL_DESPITE_ERRORS="${ENABLE_CONTINUE_ROSDEP_INSTALL_DESPITE_ERRORS:-}"
ENABLE_RECURSIVE_ADDITIONAL_DEBS="${ENABLE_RECURSIVE_ADDITIONAL_DEBS:-}"
ENABLE_RECURSIVE_ADDITIONAL_PIP="${ENABLE_RECURSIVE_ADDITIONAL_PIP:-}"
ENABLE_RECURSIVE_BLACKLISTED_PACKAGES="${ENABLE_RECURSIVE_BLACKLISTED_PACKAGES:-}"
ENABLE_RECURSIVE_CUSTOM_SCRIPT="${ENABLE_RECURSIVE_CUSTOM_SCRIPT:-}"
ENABLE_RECURSIVE_VCS_IMPORT="${ENABLE_RECURSIVE_VCS_IMPORT:-}"
ENABLE_ROS1_DEVEL_SPACE="${ENABLE_ROS1_DEVEL_SPACE:-}"
ENABLE_SINGLEARCH_PUSH="${ENABLE_SINGLEARCH_PUSH:-false}"
GIT_HTTPS_PASSWORD="${GIT_HTTPS_PASSWORD:-}"
GIT_HTTPS_SERVER="${GIT_HTTPS_SERVER:-}"
GIT_HTTPS_USER="${GIT_HTTPS_USER:-}"
GIT_SSH_KNOWN_HOST_KEYS="${GIT_SSH_KNOWN_HOST_KEYS:-}"
GIT_SSH_PRIVATE_KEY="${GIT_SSH_PRIVATE_KEY:-}"
IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-}"
ROS_DISTRO="${ROS_DISTRO:-}"
VCS_IMPORT_FILE="${VCS_IMPORT_FILE:-}"
_ENABLE_IMAGE_PUSH="${_ENABLE_IMAGE_PUSH:-false}"
_IMAGE_POSTFIX="${_IMAGE_POSTFIX:-""}"

# write image name for industrial_ci to output (GitHub-only)
if [[ -n "${GITHUB_ACTIONS}" ]]; then
    industrial_ci_image="${IMAGE}"
    [[ "${TARGET}" == *"dev"* ]] && industrial_ci_image="${DEV_IMAGE}"
    [[ -n "${_IMAGE_POSTFIX}" ]] && industrial_ci_image="${industrial_ci_image}${_IMAGE_POSTFIX}"
    if [[ "${PLATFORM}" != *","* ]]; then
        industrial_ci_image="${industrial_ci_image}-${PLATFORM}"
    else
        industrial_ci_image="${industrial_ci_image}-$(dpkg --print-architecture)"
    fi
    echo "INDUSTRIAL_CI_IMAGE=${industrial_ci_image}" >> "${GITHUB_OUTPUT}"
fi

# parse (potentially) comma-separated lists to arrays
IFS="," read -ra TARGETS <<< "${TARGET}"
if [[ "${_ENABLE_IMAGE_PUSH}" != "true" || "${ENABLE_SINGLEARCH_PUSH}" == "true" ]]; then
    IFS="," read -ra PLATFORMS <<< "${PLATFORM}"
else
    PLATFORMS=( "${PLATFORM}" )
fi
unset TARGET
unset PLATFORM

# loop over targets and platforms to build images
for PLATFORM in "${PLATFORMS[@]}"; do
    for TARGET in "${TARGETS[@]}"; do
        open_log_group "Build ${TARGET} image (${PLATFORM})"
        image="${IMAGE}"
        [[ "${TARGET}" == "dev" ]] && image="${DEV_IMAGE}"
        [[ -n "${_IMAGE_POSTFIX}" ]] && image="${image}${_IMAGE_POSTFIX}"
        [[ "${_ENABLE_IMAGE_PUSH}" != "true" || "${ENABLE_SINGLEARCH_PUSH}" == "true" ]] && image="${image}-${PLATFORM}"
        IMAGE="${image}" build_image
        close_log_group
    done
done
