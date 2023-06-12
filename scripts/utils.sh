#!/bin/bash

require_var() {
    if [[ -z "${!1}" ]]; then
        echo "Environment variable '${1}' is required"
        exit 1
    fi
}

open_log_group() {
    if [[ -n "${GITLAB_CI}" ]]; then
        echo -e "section_start:`date +%s`:build_section[collapsed=true]\r\e[0K[docker-ros] ${1}"
    elif [[ -n "${GITHUB_ACTIONS}" ]]; then
        echo "::group::[docker-ros] ${1}"
    fi
}

close_log_group() {
    if [[ -n "${GITLAB_CI}" ]]; then
        echo -e "section_end:`date +%s`:build_section\r\e[0K"
    elif [[ -n "${GITHUB_ACTIONS}" ]]; then
        echo "::endgroup::"
    fi
}
