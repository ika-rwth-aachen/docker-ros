#!/bin/bash

set -e

ROOT_PATH="$(realpath "$(cd -P "$(dirname "${0}")" && pwd)"/..)"
source "${ROOT_PATH}/scripts/utils.sh"


build_image() {
    echo "Building stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}' ..."

    DOCKER_ARGS=(
      --file "$(dirname "$0")/../docker/Dockerfile"
      --target "${TARGET}"
      --platform "${PLATFORM}"
      --tag "${IMAGE}"
    )

    if [[ "${_ENABLE_IMAGE_PUSH}" == "true" ]]; then
      DOCKER_ARGS+=( "--push" )
    else
      DOCKER_ARGS+=( "--load" )
    fi

    # required build args
    DOCKER_ARGS+=( --build-arg "BASE_IMAGE=${BASE_IMAGE}" )
    DOCKER_ARGS+=( --build-arg "COMMAND=${COMMAND}" )

    # function to add "--build-arg NAME=VALUE" only if VALUE is non-empty
    add_arg_if_set() {
      local var_name="$1"
      local var_value="${!var_name}"
      if [[ -n "${var_value}" ]]; then
        DOCKER_ARGS+=( "--build-arg" "${var_name}=${var_value}" )
      fi
    }

    # optional build args
    add_arg_if_set "ADDITIONAL_DEBS_FILE"
    add_arg_if_set "ADDITIONAL_FILES_DIR"
    add_arg_if_set "ADDITIONAL_PIP_FILE"
    add_arg_if_set "AFTER_DEPENDENCY_IDENTIFICATION_SCRIPT"
    add_arg_if_set "AFTER_DEPENDENCY_INSTALLATION_SCRIPT"
    add_arg_if_set "BEFORE_DEPENDENCY_IDENTIFICATION_SCRIPT"
    add_arg_if_set "BEFORE_DEPENDENCY_INSTALLATION_SCRIPT"
    add_arg_if_set "BLACKLISTED_PACKAGES_FILE"
    add_arg_if_set "CMAKE_ARGS"
    add_arg_if_set "CUSTOM_SCRIPT_FILE"
    add_arg_if_set "DISABLE_ROS_INSTALLATION"
    add_arg_if_set "ENABLE_RECURSIVE_ADDITIONAL_DEBS"
    add_arg_if_set "ENABLE_RECURSIVE_ADDITIONAL_PIP"
    add_arg_if_set "ENABLE_RECURSIVE_AFTER_DEPENDENCY_INSTALLATION_SCRIPT"
    add_arg_if_set "ENABLE_RECURSIVE_BLACKLISTED_PACKAGES"
    add_arg_if_set "ENABLE_RECURSIVE_VCS_IMPORT"
    add_arg_if_set "GIT_HTTPS_PASSWORD"
    add_arg_if_set "GIT_HTTPS_SERVER"
    add_arg_if_set "GIT_HTTPS_USER"
    add_arg_if_set "GIT_SSH_KNOWN_HOST_KEYS"
    add_arg_if_set "GIT_SSH_PRIVATE_KEY"
    add_arg_if_set "RMW_IMPLEMENTATION"
    add_arg_if_set "RMW_ZENOH_GIT_REF"
    add_arg_if_set "ROS_DISTRO"
    add_arg_if_set "VCS_IMPORT_FILE"

    DOCKER_ARGS+=( "." )

    docker buildx build "${DOCKER_ARGS[@]}"
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
