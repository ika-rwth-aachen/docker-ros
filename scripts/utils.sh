#!/bin/bash

require_var() {
    if [[ -z "${!1}" ]]; then
        echo "Environment variable '${1}' is required"
        exit 1
    fi
}

open_log_group() {
    if [[ "${GITLAB_CI}" ]]; then
        echo "\e[0Ksection_start:`date +%s`:my_first_section\r\e[0K${1}"
    elif [[ "${CI}" ]]; then
        echo "::group::[docker-ros] ${1}"
    fi
}

close_log_group() {
    if [[ "${GITLAB_CI}" ]]; then
        echo -e "\e[0Ksection_end:`date +%s`:my_first_section\r\e[0K"
    elif [[ "${CI}" ]]; then
        echo "::endgroup::"
    fi
}
