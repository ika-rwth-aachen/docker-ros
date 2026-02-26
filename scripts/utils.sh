#!/bin/bash

LOG_GROUP_COUNTER=0
LOG_GROUP_SECTION_STACK=()

require_var() {
    if [[ -z "${!1}" ]]; then
        echo "Environment variable '${1}' is required"
        exit 1
    fi
}

open_log_group() {
    if [[ -n "${GITLAB_CI}" ]]; then
        LOG_GROUP_COUNTER=$((LOG_GROUP_COUNTER + 1))
        local section_id="build_section_${LOG_GROUP_COUNTER}"
        LOG_GROUP_SECTION_STACK+=( "${section_id}" )
        echo -e "section_start:`date +%s`:${section_id}[collapsed=true]\r\e[0K[docker-ros] ${1}"
    elif [[ -n "${GITHUB_ACTIONS}" ]]; then
        echo "::group::[docker-ros] ${1}"
    fi
}

close_log_group() {
    if [[ -n "${GITLAB_CI}" ]]; then
        local stack_idx=$((${#LOG_GROUP_SECTION_STACK[@]} - 1))
        local section_id="build_section"
        if [[ ${stack_idx} -ge 0 ]]; then
            section_id="${LOG_GROUP_SECTION_STACK[${stack_idx}]}"
            unset 'LOG_GROUP_SECTION_STACK[${stack_idx}]'
        fi
        echo -e "section_end:`date +%s`:${section_id}\r\e[0K"
    elif [[ -n "${GITHUB_ACTIONS}" ]]; then
        echo "::endgroup::"
    fi
}
