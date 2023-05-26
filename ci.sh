#!/bin/bash

set -e

DOCKER_ROS_PATH="$(cd -P "$(dirname "${0}")" && pwd)"
source "${DOCKER_ROS_PATH}/build.sh"

require_var() {
    if [[ -z "${!1}" ]]; then
        echo "Environment variable '${1}' is required"
        exit 1
    fi
}

open_log_group() {
    echo "::group::[docker-ros] ${1}"
}

close_log_group() {
    echo "::endgroup::"
}

# check for required variables and set defaults for optional variables
TARGET="${TARGET:-run}"
PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
require_var "BASE_IMAGE"
require_var "IMAGE"
[[ "${TARGET}" == *"run"* ]] && require_var "COMMAND"
DEV_IMAGE="${DEV_IMAGE:-${IMAGE}-dev}" # TODO: what if IMAGE has no TAG?
ENABLE_IMAGE_PUSH="${ENABLE_IMAGE_PUSH:-false}"
ENABLE_MULTIARCH_BUILD="${ENABLE_MULTIARCH_BUILD:-false}"
IMAGE_POSTFIX="${IMAGE_POSTFIX:-""}"

# write image name for industrial_ci to output
# TODO: GitHub-only
industrial_ci_image=${IMAGE}
[[ "${TARGET}" == *"dev"* ]] && industrial_ci_image=${DEV_IMAGE}
echo "INDUSTRIAL_CI_IMAGE=${industrial_ci_image}_${CI_POSTFIX}-$(dpkg --print-architecture)" >> "${GITHUB_OUTPUT}"

# parse (potentially) comma-separated lists to arrays
IFS="," read -ra TARGETS <<< "${TARGET}"
if [[ "${ENABLE_MULTIARCH_BUILD}" == "true" ]]; then
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
        [[ -n "${IMAGE_POSTFIX}" ]] && image="${image}${!IMAGE_POSTFIX}"
        IMAGE="${image}" build_image
        close_log_group
    done
done
