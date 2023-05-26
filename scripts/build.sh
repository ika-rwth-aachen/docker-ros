#!/bin/bash

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
    # TODO: support local build
}
