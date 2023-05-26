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

# parse (potentially) comma-separated lists to arrays
IFS="," read -ra TARGETS <<< "${TARGET}"
unset TARGET

# loop over targets to push multi-arch images
for TARGET in "${TARGETS[@]}"; do
    open_log_group "Push ${TARGET} image (${PLATFORM[*]})"
    image="${IMAGE}"
    [[ "${TARGET}" == "dev" ]] && image="${DEV_IMAGE}"
    ENABLE_IMAGE_PUSH="true" IMAGE="${image}" build_image
    close_log_group
done
