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


render_buildx_log_stream() {
    local current_group=""
    local group_name
    local group_title
    local line
    local step_id
    declare -A step_groups=()

    # Plain BuildKit output associates subsequent progress lines with the
    # numeric step ID introduced by its first line.  Remember that mapping so
    # every line can be emitted immediately in the corresponding CI group.
    while IFS= read -r line || [[ -n "${line}" ]]; do
        group_name="BuildKit"
        if [[ "${line}" =~ ^#([0-9]+)[[:space:]]+\[([^[:space:]]+[[:space:]]+)?([[:alnum:]_-]+)[[:space:]]+[0-9]+/[0-9]+\] ]]; then
            step_id="${BASH_REMATCH[1]}"
            group_name="${BASH_REMATCH[3]}"
            step_groups["${step_id}"]="${group_name}"
        elif [[ "${line}" =~ ^#([0-9]+) ]]; then
            step_id="${BASH_REMATCH[1]}"
            group_name="${step_groups[${step_id}]:-${current_group:-BuildKit}}"
        elif [[ -n "${current_group}" ]]; then
            group_name="${current_group}"
        fi

        if [[ "${group_name}" != "${current_group}" ]]; then
            [[ -n "${current_group}" ]] && close_log_group
            group_title="BuildKit stage: ${group_name}"
            [[ "${group_name}" == "BuildKit" ]] && group_title="BuildKit setup and diagnostics"
            open_log_group "${group_title}"
            current_group="${group_name}"
        fi

        printf '%s\n' "${line}"
    done

    [[ -n "${current_group}" ]] && close_log_group
}
