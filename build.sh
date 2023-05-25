#!/bin/bash

set -e

# load environment variables
_BASE_IMAGE="${BASE_IMAGE}"
_CACHE_FROM="${CACHE_FROM}"
_CACHE_TO="${CACHE_TO}"
_COMMAND="${COMMAND}"
_GIT_HTTPS_PASSWORD="${GIT_HTTPS_PASSWORD}"
_GIT_HTTPS_USER="${GIT_HTTPS_USER}"
_IMAGE="${IMAGE}"
_PLATFORM="${PLATFORM}"
_TARGET="${TARGET}"

# load (unset) variables from .env file
[[ -f .env ]] && source .env # TODO: configurable/flexible .env file
[[ -z "${_BASE_IMAGE}" ]]           && _BASE_IMAGE="${BASE_IMAGE}"
[[ -z "${_CACHE_FROM}" ]]           && _CACHE_FROM="${CACHE_FROM}"
[[ -z "${_CACHE_TO}" ]]             && _CACHE_TO="${CACHE_TO}"
[[ -z "${_COMMAND}" ]]              && _COMMAND="${COMMAND}"
[[ -z "${_GIT_HTTPS_PASSWORD}" ]]   && _TARGET="${GIT_HTTPS_PASSWORD}"
[[ -z "${_GIT_HTTPS_USER}" ]]       && _TARGET="${GIT_HTTPS_USER}"
[[ -z "${_IMAGE}" ]]                && _IMAGE="${IMAGE}"
[[ -z "${_PLATFORM}" ]]             && _RUN_IMAGE="${PLATFORM}"
[[ -z "${_TARGET}" ]]               && _TARGET="${TARGET}"

# check for required environment variables or set defaults
[[ -z "${_BASE_IMAGE}" ]]   && echo "Environment variable 'BASE_IMAGE' is required" && exit 1
[[ -z "${_COMMAND}" ]]      && echo "Environment variable 'COMMAND' is required"    && exit 1
[[ -z "${_IMAGE}" ]]        && echo "Environment variable 'IMAGE' is required"      && exit 1
_PLATFORM="${_PLATFORM:-$(dpkg --print-architecture)}"
_TARGET="${_TARGET:-run}"

# build image
echo "Building stage '${_TARGET}' for platform '${_PLATFORM}' as '${_IMAGE}' ..."
docker buildx build \
    --file $(dirname $0)/Dockerfile \
    --target "${_TARGET}" \
    --platform "${_PLATFORM}" \
    --tag "${_IMAGE}" \
    --push \
    $(if [[ -n "${_CACHE_FROM}" ]]; then echo "--cache-from ${_CACHE_FROM}"; fi) \
    $(if [[ -n "${_CACHE_TO}" ]]; then echo "--cache-to ${_CACHE_TO}"; fi) \
    --build-arg BASE_IMAGE="${_BASE_IMAGE}" \
    --build-arg COMMAND="${_COMMAND}" \
    --build-arg GIT_HTTPS_PASSWORD="${_GIT_HTTPS_PASSWORD}" \
    --build-arg GIT_HTTPS_USER="${_GIT_HTTPS_USER}" \
    .
echo "Successfully built stage '${_TARGET}' for platform '${_PLATFORM}' as '${_IMAGE}'"
