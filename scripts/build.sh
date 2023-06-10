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
        $(if [[ "${_ENABLE_IMAGE_PUSH}" == "true" ]]; then echo "--push"; else echo "--load"; fi) \
        --build-arg BASE_IMAGE="${BASE_IMAGE}" \
        --build-arg COMMAND="${COMMAND}" \
        $(if [[ -n "${ROS_DISTRO}" ]]; then echo "--build-arg ROS_DISTRO=${ROS_DISTRO}"; fi) \
        --build-arg GIT_HTTPS_SERVER="${GIT_HTTPS_SERVER}" \
        --build-arg GIT_HTTPS_USER="${GIT_HTTPS_USER}" \
        --build-arg GIT_HTTPS_PASSWORD="${GIT_HTTPS_PASSWORD}" \
        --build-arg GIT_SSH_PRIVATE_KEY="${GIT_SSH_PRIVATE_KEY}" \
        --build-arg GIT_SSH_KNOWN_HOST_KEYS="${GIT_SSH_KNOWN_HOST_KEYS}" \
        $(if [[ -n "${ADDITIONAL_DEBS_FILE}" ]]; then echo "--build-arg ADDITIONAL_DEBS_FILE=${ADDITIONAL_DEBS_FILE}"; fi) \
        $(if [[ -n "${ADDITIONAL_DEBS_RECURSIVE}" ]]; then echo "--build-arg ADDITIONAL_DEBS_RECURSIVE=${ADDITIONAL_DEBS_RECURSIVE}"; fi) \
        $(if [[ -n "${ADDITIONAL_FILES_DIR}" ]]; then echo "--build-arg ADDITIONAL_FILES_DIR=${ADDITIONAL_FILES_DIR}"; fi) \
        $(if [[ -n "${ADDITIONAL_PIP_FILE}" ]]; then echo "--build-arg ADDITIONAL_PIP_FILE=${ADDITIONAL_PIP_FILE}"; fi) \
        $(if [[ -n "${ADDITIONAL_PIP_RECURSIVE}" ]]; then echo "--build-arg ADDITIONAL_PIP_RECURSIVE=${ADDITIONAL_PIP_RECURSIVE}"; fi) \
        $(if [[ -n "${CUSTOM_SCRIPT_FILE}" ]]; then echo "--build-arg CUSTOM_SCRIPT_FILE=${CUSTOM_SCRIPT_FILE}"; fi) \
        $(if [[ -n "${CUSTOM_SCRIPT_RECURSIVE}" ]]; then echo "--build-arg CUSTOM_SCRIPT_RECURSIVE=${CUSTOM_SCRIPT_RECURSIVE}"; fi) \
        .
    echo "Successfully built stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}'"
}


# check if script is executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # check for required variables and set defaults for optional variables
    TARGET="${TARGET:-run}"
    PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
    require_var "BASE_IMAGE"
    require_var "IMAGE"
    [[ "${TARGET}" == *"run"* ]] && require_var "COMMAND"
    _ENABLE_IMAGE_PUSH="${_ENABLE_IMAGE_PUSH:-false}"
    build_image
fi
