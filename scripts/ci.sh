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
SLIM_IMAGE_NAME="${SLIM_IMAGE_NAME:-${IMAGE_NAME}}"
SLIM_IMAGE_TAG="${SLIM_IMAGE_TAG:-${IMAGE_TAG}-slim}"

IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"
DEV_IMAGE="${DEV_IMAGE_NAME}:${DEV_IMAGE_TAG}"
SLIM_IMAGE="${SLIM_IMAGE_NAME}:${SLIM_IMAGE_TAG}"
ENABLE_SLIM="${ENABLE_SLIM:-true}"
SLIM_BUILD_ARGS="${SLIM_BUILD_ARGS:-'--sensor-ipc-mode proxy --continue-after=10 --show-clogs --http-probe=false'}"
ENABLE_SINGLEARCH_PUSH="${ENABLE_SINGLEARCH_PUSH:-false}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-}"
ROS_DISTRO="${ROS_DISTRO:-}"
GIT_HTTPS_SERVER="${GIT_HTTPS_SERVER:-}"
GIT_HTTPS_USER="${GIT_HTTPS_USER:-}"
GIT_HTTPS_PASSWORD="${GIT_HTTPS_PASSWORD:-}"
GIT_SSH_PRIVATE_KEY="${GIT_SSH_PRIVATE_KEY:-}"
GIT_SSH_KNOWN_HOST_KEYS="${GIT_SSH_KNOWN_HOST_KEYS:-}"
ADDITIONAL_DEBS_FILE="${ADDITIONAL_DEBS_FILE:-}"
ENABLE_RECURSIVE_ADDITIONAL_DEBS="${ENABLE_RECURSIVE_ADDITIONAL_DEBS:-}"
ADDITIONAL_FILES_DIR="${ADDITIONAL_FILES_DIR:-}"
ADDITIONAL_PIP_FILE="${ADDITIONAL_PIP_FILE:-}"
ENABLE_RECURSIVE_ADDITIONAL_PIP="${ENABLE_RECURSIVE_ADDITIONAL_PIP:-}"
BLACKLISTED_PACKAGES_FILE="${BLACKLISTED_PACKAGES_FILE:-}"
ENABLE_RECURSIVE_BLACKLISTED_PACKAGES="${ENABLE_RECURSIVE_BLACKLISTED_PACKAGES:-}"
CUSTOM_SCRIPT_FILE="${CUSTOM_SCRIPT_FILE:-}"
ENABLE_RECURSIVE_CUSTOM_SCRIPT="${ENABLE_RECURSIVE_CUSTOM_SCRIPT:-}"
VCS_IMPORT_FILE="${VCS_IMPORT_FILE:-}"
ENABLE_RECURSIVE_VCS_IMPORT="${ENABLE_RECURSIVE_VCS_IMPORT:-}"
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

# prepare slim
curl -L -o ds.tar.gz https://github.com/slimtoolkit/slim/releases/download/1.40.11/dist_linux.tar.gz
tar -xvf ds.tar.gz

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

    # slim image
    if [[ "${ENABLE_SLIM}" == "true" && "${TARGET}" == "run" ]]; then
        open_log_group "Slim image (${PLATFORM})"
        image="${IMAGE}"
        slim_image="${SLIM_IMAGE}"
        [[ -n "${_IMAGE_POSTFIX}" ]] && image="${image}${_IMAGE_POSTFIX}"
        [[ -n "${_IMAGE_POSTFIX}" ]] && slim_image="${slim_image}${_IMAGE_POSTFIX}"
        [[ "${_ENABLE_IMAGE_PUSH}" != "true" || "${ENABLE_SINGLEARCH_PUSH}" == "true" ]] && image="${image}-${PLATFORM}"
        # TODO: not yet working; builds image with suffix, pushed without? [[ "${_ENABLE_IMAGE_PUSH}" != "true" || "${ENABLE_SINGLEARCH_PUSH}" == "true" ]] && slim_image="${slim_image}-${PLATFORM}"
        if [[ "${_ENABLE_IMAGE_PUSH}" == "true" || "${ENABLE_SINGLEARCH_PUSH}" == "true" ]]; then
            docker push "${slim_image}"
        else
            cd dist_linux*
            ./slim build --target "${image}" --tag "${slim_image}" ${SLIM_BUILD_ARGS}
            cd -
        fi
        close_log_group
    fi
done
