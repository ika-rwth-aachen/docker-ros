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

# check for required variables set defaults for optional variables
TARGET="${TARGET:-run}"
PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
require_var "BASE_IMAGE"
require_var "IMAGE"
[[ "${TARGET}" == *"run"* ]] && require_var "COMMAND"
DEV_IMAGE="${DEV_IMAGE:-${IMAGE}-dev}" # TODO: what if IMAGE has no TAG?

# set constant variables
CI_POSTFIX="ci"
CI_ARCH_POSTFIX="PLATFORM"

# parse (potentially) comma-separated lists to arrays
IFS="," read -ra TARGETS <<< "${TARGET}"
IFS="," read -ra PLATFORMS <<< "${PLATFORM}"
unset TARGET
unset PLATFORM

# loop over targets and platforms to build images
for PLATFORM in "${PLATFORMS[@]}"; do
    for TARGET in "${TARGETS[@]}"; do
        open_log_group "Build ${TARGET} image (${TARGET})"
        image="${IMAGE}"
        [[ "${TARGET}" == "dev" ]] && image="${DEV_IMAGE}"
        ci_image="${image}_${CI_POSTFIX}-${!CI_ARCH_POSTFIX}"
        IMAGE="${ci_image}" build_image
        close_log_group
    done
done
