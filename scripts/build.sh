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
        $(if [[ -n "${ADDITIONAL_DEBS_FILE}" ]]; then echo "--build-arg ADDITIONAL_DEBS_FILE=${ADDITIONAL_DEBS_FILE}"; fi) \
        $(if [[ -n "${ADDITIONAL_FILES_DIR}" ]]; then echo "--build-arg ADDITIONAL_FILES_DIR=${ADDITIONAL_FILES_DIR}"; fi) \
        $(if [[ -n "${ADDITIONAL_PIP_FILE}" ]]; then echo "--build-arg ADDITIONAL_PIP_FILE=${ADDITIONAL_PIP_FILE}"; fi) \
        $(if [[ -n "${BLACKLISTED_PACKAGES_FILE}" ]]; then echo "--build-arg BLACKLISTED_PACKAGES_FILE=${BLACKLISTED_PACKAGES_FILE}"; fi) \
        $(if [[ -n "${CUSTOM_SCRIPT_FILE}" ]]; then echo "--build-arg CUSTOM_SCRIPT_FILE=${CUSTOM_SCRIPT_FILE}"; fi) \
        $(if [[ -n "${DISABLE_ROS_INSTALLATION}" ]]; then echo "--build-arg DISABLE_ROS_INSTALLATION=${DISABLE_ROS_INSTALLATION}"; fi) \
        $(if [[ -n "${ENABLE_RECURSIVE_ADDITIONAL_DEBS}" ]]; then echo "--build-arg ENABLE_RECURSIVE_ADDITIONAL_DEBS=${ENABLE_RECURSIVE_ADDITIONAL_DEBS}"; fi) \
        $(if [[ -n "${ENABLE_RECURSIVE_ADDITIONAL_PIP}" ]]; then echo "--build-arg ENABLE_RECURSIVE_ADDITIONAL_PIP=${ENABLE_RECURSIVE_ADDITIONAL_PIP}"; fi) \
        $(if [[ -n "${ENABLE_RECURSIVE_BLACKLISTED_PACKAGES}" ]]; then echo "--build-arg ENABLE_RECURSIVE_BLACKLISTED_PACKAGES=${ENABLE_RECURSIVE_BLACKLISTED_PACKAGES}"; fi) \
        $(if [[ -n "${ENABLE_RECURSIVE_CUSTOM_SCRIPT}" ]]; then echo "--build-arg ENABLE_RECURSIVE_CUSTOM_SCRIPT=${ENABLE_RECURSIVE_CUSTOM_SCRIPT}"; fi) \
        $(if [[ -n "${ENABLE_RECURSIVE_VCS_IMPORT}" ]]; then echo "--build-arg ENABLE_RECURSIVE_VCS_IMPORT=${ENABLE_RECURSIVE_VCS_IMPORT}"; fi) \
        $(if [[ -n "${GIT_HTTPS_PASSWORD}" ]]; then echo "--build-arg GIT_HTTPS_PASSWORD=${GIT_HTTPS_PASSWORD}"; fi) \
        $(if [[ -n "${GIT_HTTPS_SERVER}" ]]; then echo "--build-arg GIT_HTTPS_SERVER=${GIT_HTTPS_SERVER}"; fi) \
        $(if [[ -n "${GIT_HTTPS_USER}" ]]; then echo "--build-arg GIT_HTTPS_USER=${GIT_HTTPS_USER}"; fi) \
        $(if [[ -n "${GIT_SSH_KNOWN_HOST_KEYS}" ]]; then echo "--build-arg GIT_SSH_KNOWN_HOST_KEYS=${GIT_SSH_KNOWN_HOST_KEYS}"; fi) \
        $(if [[ -n "${GIT_SSH_PRIVATE_KEY}" ]]; then echo "--build-arg GIT_SSH_PRIVATE_KEY=${GIT_SSH_PRIVATE_KEY}"; fi) \
        $(if [[ -n "${RMW_IMPLEMENTATION}" ]]; then echo "--build-arg RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"; fi) \
        $(if [[ -n "${RMW_ZENOH_GIT_REF}" ]]; then echo "--build-arg RMW_ZENOH_GIT_REF=${RMW_ZENOH_GIT_REF}"; fi) \
        $(if [[ -n "${ROS_DISTRO}" ]]; then echo "--build-arg ROS_DISTRO=${ROS_DISTRO}"; fi) \
        $(if [[ -n "${VCS_IMPORT_FILE}" ]]; then echo "--build-arg VCS_IMPORT_FILE=${VCS_IMPORT_FILE}"; fi) \
        .
    echo "Successfully built stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}'"
}


# check if script is executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # load variables from .env
    [[ -f "$(pwd)/.env" ]] && source "$(pwd)/.env"
    # check for required variables and set defaults for optional variables
    TARGET="${TARGET:-run}"
    PLATFORM="${PLATFORM:-$(dpkg --print-architecture)}"
    require_var "BASE_IMAGE"
    require_var "IMAGE"
    [[ "${TARGET}" == *"run"* ]] && require_var "COMMAND"
    _ENABLE_IMAGE_PUSH="${_ENABLE_IMAGE_PUSH:-false}"
    build_image
fi
