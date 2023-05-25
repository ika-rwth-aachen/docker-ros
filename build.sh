#!/bin/bash

set -e

# load environment variables
_BASE_IMAGE="${BASE_IMAGE}"
_CACHE_FROM="${CACHE_FROM}"
_CACHE_TO="${CACHE_TO}"
_COMMAND="${COMMAND}"
_DEV_IMAGE="${DEV_IMAGE}"
_GIT_HTTPS_PASSWORD="${GIT_HTTPS_PASSWORD}"
_GIT_HTTPS_USER="${GIT_HTTPS_USER}"
_PLATFORM="${PLATFORM}"
_RUN_IMAGE="${RUN_IMAGE}"
_TARGET="${TARGET}"

# load (unset) variables from .env file
source .env
[[ -z "${_BASE_IMAGE}" ]]           && _BASE_IMAGE="${BASE_IMAGE}"
[[ -z "${_CACHE_FROM}" ]]           && _CACHE_FROM="${CACHE_FROM}"
[[ -z "${_CACHE_TO}" ]]             && _CACHE_TO="${CACHE_TO}"
[[ -z "${_COMMAND}" ]]              && _COMMAND="${COMMAND}"
[[ -z "${_DEV_IMAGE}" ]]            && _DEV_IMAGE="${DEV_IMAGE}"
[[ -z "${_GIT_HTTPS_PASSWORD}" ]]   && _TARGET="${GIT_HTTPS_PASSWORD}"
[[ -z "${_GIT_HTTPS_USER}" ]]       && _TARGET="${GIT_HTTPS_USER}"
[[ -z "${_PLATFORM}" ]]             && _RUN_IMAGE="${PLATFORM}"
[[ -z "${_RUN_IMAGE}" ]]            && _RUN_IMAGE="${RUN_IMAGE}"
[[ -z "${_TARGET}" ]]               && _TARGET="${TARGET}"

# check for required environment variables or set defaults
[[ -z "${_BASE_IMAGE}" ]]   && echo "Environment variable 'BASE_IMAGE' is required" && exit 1
[[ -z "${_COMMAND}" ]]      && echo "Environment variable 'COMMAND' is required"    && exit 1
_PLATFORM="${_PLATFORM:-$(dpkg --print-architecture)}"

# evaluate which targets to build
if [[ -z "${_TARGET}" ]]; then
    [[ -z "${_DEV_IMAGE}" && -z "${_RUN_IMAGE}" ]] && echo "One of environment variables 'DEV_IMAGE' or 'RUN_IMAGE' is required" && exit 1
    [[ -n "${_DEV_IMAGE}" && -z "${_RUN_IMAGE}" ]] && _TAGS=( "${_DEV_IMAGE}" )                 && _TARGETS=( dev )
    [[ -z "${_DEV_IMAGE}" && -n "${_RUN_IMAGE}" ]] && _TAGS=( "${_RUN_IMAGE}" )                 && _TARGETS=( run )
    [[ -n "${_DEV_IMAGE}" && -n "${_RUN_IMAGE}" ]] && _TAGS=( "${_DEV_IMAGE}" "${_RUN_IMAGE}" ) && _TARGETS=( dev run )
else
    [[ "${_TARGET}" == "dev" ]] && _TAGS=( "${_DEV_IMAGE}" ) && [[ -z "${_DEV_IMAGE}" ]] && echo "Environment variable 'DEV_IMAGE' is required" && exit 1
    [[ "${_TARGET}" == "run" ]] && _TAGS=( "${_RUN_IMAGE}" ) && [[ -z "${_RUN_IMAGE}" ]] && echo "Environment variable 'RUN_IMAGE' is required" && exit 1
    _TARGETS=( "${_TARGET}" )
fi

# build image(s)
for (( i=0; i<${#_TARGETS[*]}; ++i)); do
    echo "Building stage '${_TARGETS[$i]}' for platform '${_PLATFORM}' as '${_TAGS[$i]}' ..."
    docker build \
        --file docker-ros/Dockerfile \
        --target "${_TARGETS[$i]}" \
        --platform "${_PLATFORM}" \
        --tag "${_TAGS[$i]}" \
        $(if [[ -n "${_CACHE_FROM}" ]]; then echo "--cache-from ${_CACHE_FROM}"; fi) \
        $(if [[ -n "${_CACHE_TO}" ]]; then echo "--cache-to ${_CACHE_TO}"; fi) \
        --build-arg BASE_IMAGE="${_BASE_IMAGE}" \
        --build-arg COMMAND="${_COMMAND}" \
        --build-arg GIT_HTTPS_PASSWORD="${_GIT_HTTPS_PASSWORD}" \
        --build-arg GIT_HTTPS_USER="${_GIT_HTTPS_USER}" \
        ..
done
for (( i=0; i<${#_TARGETS[*]}; ++i)); do
    echo "Successfully built stage '${_TARGETS[$i]}' for platform ${_PLATFORM} as '${_TAGS[$i]}'"
done
