#!/bin/bash

build_image() {

    echo "Building stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}' ..."
    docker buildx build \
        --file $(dirname $0)/Dockerfile \
        --target "${TARGET}" \
        --platform "${PLATFORM}" \
        --tag "${IMAGE}" \
        --load \
        --build-arg BASE_IMAGE="${BASE_IMAGE}" \
        --build-arg COMMAND="${COMMAND}" \
        --build-arg GIT_HTTPS_PASSWORD="${GIT_HTTPS_PASSWORD}" \
        --build-arg GIT_HTTPS_USER="${GIT_HTTPS_USER}" \
        .
    echo "Successfully built stage '${TARGET}' for platform '${PLATFORM}' as '${IMAGE}'"
    # TODO: GIT_HTTPS
}