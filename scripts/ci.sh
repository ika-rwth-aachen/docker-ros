#!/bin/bash

set -e

ROOT_PATH="$(realpath "$(cd -P "$(dirname "${0}")" && pwd)"/..)"
source "${ROOT_PATH}/scripts/build.sh"
source "${ROOT_PATH}/scripts/utils.sh"


# check for required variables and set defaults for optional variables
TARGET="${TARGET:-run}"
PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
require_var "BASE_IMAGE"
require_var "IMAGE"
[[ "${TARGET}" == *"run"* ]] && require_var "COMMAND"
DEV_IMAGE="${DEV_IMAGE:-${IMAGE}-dev}" # TODO: what if IMAGE has no TAG?
ENABLE_IMAGE_PUSH="${ENABLE_IMAGE_PUSH:-false}"
ENABLE_MULTIARCH_BUILD="${ENABLE_MULTIARCH_BUILD:-false}"
IMAGE_POSTFIX="${IMAGE_POSTFIX:-"_ci"}"

# write image name for industrial_ci to output
# TODO: GitHub-only
industrial_ci_image="${IMAGE}"
[[ "${TARGET}" == *"dev"* ]] && industrial_ci_image="${DEV_IMAGE}"
industrial_ci_image="${industrial_ci_image}${IMAGE_POSTFIX}"
[[ "${ENABLE_MULTIARCH_BUILD}" != "true" ]] && industrial_ci_image="${industrial_ci_image}-$(dpkg --print-architecture)"
echo "INDUSTRIAL_CI_IMAGE=${industrial_ci_image}" >> "${GITHUB_OUTPUT}"

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
        [[ -n "${IMAGE_POSTFIX}" ]] && image="${image}${IMAGE_POSTFIX}"
        [[ "${ENABLE_MULTIARCH_BUILD}" != "true" ]] && image="${image}-${TARGET}"
        IMAGE="${image}" build_image
        close_log_group
    done
done
