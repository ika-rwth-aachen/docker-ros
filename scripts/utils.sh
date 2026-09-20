#!/bin/bash

LOG_GROUP_COUNTER=0
LOG_GROUP_SECTION_STACK=()

require_var() {
    if [[ -z "${!1}" ]]; then
        echo "Environment variable '${1}' is required"
        exit 1
    fi
}

require_command() {
    if ! command -v "$1" > /dev/null 2>&1; then
        echo "Required command '${1}' is not available"
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


render_buildx_log() {
    local log_file="$1"
    local current_group=""
    local group_name
    local group_title
    local encoded_line
    local rendered_log

    rendered_log="$(mktemp)"
    if ! render_buildx_log_records "${log_file}" "${rendered_log}"; then
        open_log_group "BuildKit diagnostics"
        echo "Unable to decode BuildKit progress; showing the original output."
        cat "${log_file}"
        close_log_group
        rm -f "${rendered_log}"
        return 1
    fi

    # BuildKit's raw JSON reports a vertex before emitting its logs.  Buffering
    # the complete stream lets us associate each log record with the named
    # Dockerfile stage and render it in a single, collapsible CI log group.
    while IFS=$'\t' read -r group_name encoded_line; do
        if [[ "${group_name}" != "${current_group}" ]]; then
            [[ -n "${current_group}" ]] && close_log_group
            group_title="BuildKit stage: ${group_name}"
            [[ "${group_name}" == "BuildKit" ]] && group_title="BuildKit setup and diagnostics"
            open_log_group "${group_title}"
            current_group="${group_name}"
        fi
        printf '%s' "${encoded_line}" | base64 -d
        printf '\n'
    done < "${rendered_log}"

    [[ -n "${current_group}" ]] && close_log_group
    rm -f "${rendered_log}"
}


render_buildx_log_records() {
    local log_file="$1"
    local rendered_log="$2"

    jq -Rrs '
          def stage_from_name($name):
            try ($name | capture("^\\[(?:[^ ]+ )?(?<stage>[A-Za-z0-9_-]+) [0-9]+/[0-9]+\\]").stage)
            catch "BuildKit";

          [ split("\n")[]
            | select(length > 0)
            | (fromjson? // {unparsed: .})
            # Buildx writes SolveStatus objects, whose vertexes and logs are
            # arrays.  Older clients may write the individual objects instead.
            | if (.vertexes? or .logs? or .warnings?) then
                (.vertexes[]? | {vertex: .}),
                (.logs[]? | {log: .}),
                (.warnings[]? | {warning: .})
              else .
              end
          ] as $events
          | (reduce $events[] as $event
              ({};
               if ($event.vertex? and $event.vertex.digest? and $event.vertex.name?) then
                 .[$event.vertex.digest] = stage_from_name($event.vertex.name)
               else .
               end)) as $vertex_stages
          | [ $events[]
              | if .unparsed? then
                  {stage: "BuildKit", text: .unparsed}
                elif .log? then
                  {stage: ($vertex_stages[.log.vertex] // "BuildKit"), text: (.log.data | @base64d)}
                elif .warning? then
                  {stage: ($vertex_stages[.warning.vertex] // "BuildKit"),
                   text: ((.warning.short | @base64d) + "\n" + ((.warning.detail // []) | map(@base64d) | join("\n")))}
                elif (.vertex? and .vertex.completed?) then
                  .vertex as $vertex
                  | {stage: ($vertex_stages[$vertex.digest] // "BuildKit"),
                     text: ($vertex.name + if $vertex.error? then " ERROR: " + $vertex.error elif $vertex.cached then " CACHED" else " DONE" end)}
                else empty
                end
            ] as $records
          | (reduce $records[] as $record
              ([]; if index($record.stage) then . else . + [$record.stage] end)) as $stage_order
          | $stage_order[] as $stage
          | $records[]
          | select(.stage == $stage)
          | [$stage, (.text | @base64)] | @tsv
        ' "${log_file}" > "${rendered_log}"
}
