#!/bin/bash

require_var() {
    if [[ -z "${!1}" ]]; then
        echo "Environment variable '${1}' is required"
        exit 1
    fi
}

open_log_group() {
    if [[ "${GITLAB_CI}" ]]; then
        echo "section_start:build_jobs:$(date +%s)"
    elif [[ "${CI}" ]]; then
        echo "::group::[docker-ros] ${1}"
    fi
}

close_log_group() {
    if [[ "${GITLAB_CI}" ]]; then
        echo "section_end:build_jobs:$(date +%s)"
    elif [[ "${CI}" ]]; then
        echo "::endgroup::"
    fi
}
