#!/bin/bash

set -e

ROOT_PATH="$(realpath "$(cd -P "$(dirname "${0}")" && pwd)"/..)"
source "${ROOT_PATH}/scripts/utils.sh"


build_image() {

    echo "Building stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}' ..."
    docker buildx build \
        --file $(dirname $0)/../docker/Dockerfile \
        --target "${TARGET}" \
        --platform "${PLATFORM}" \
        --tag "${IMAGE}" \
        $(if [[ "${ENABLE_IMAGE_PUSH}" == "true" ]]; then echo "--push"; else echo "--load"; fi) \
        --build-arg BASE_IMAGE="${BASE_IMAGE}" \
        --build-arg COMMAND="${COMMAND}" \
        --build-arg GIT_HTTPS_PASSWORD="${GIT_HTTPS_PASSWORD}" \
        --build-arg GIT_HTTPS_USER="${GIT_HTTPS_USER}" \
        .
    echo "Successfully built stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}'"
    # TODO: GIT_HTTPS
}


# check if script is executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # check for required variables and set defaults for optional variables
    TARGET="${TARGET:-run}"
    PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
    require_var "BASE_IMAGE"
    require_var "IMAGE"
    [[ "${TARGET}" == *"run"* ]] && require_var "COMMAND"
    ENABLE_IMAGE_PUSH="${ENABLE_IMAGE_PUSH:-false}"
    build_image
fi
