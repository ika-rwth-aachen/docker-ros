#!/bin/bash

trim_whitespace() {
    local value="$*"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"
    printf '%s' "${value}"
}

join_with_comma() {
    local first_item=true
    local value
    for value in "$@"; do
        [[ -n "${value}" ]] || continue
        if [[ "${first_item}" == "true" ]]; then
            printf '%s' "${value}"
            first_item=false
        else
            printf ', %s' "${value}"
        fi
    done
}

list_package_xml_files() {
    find . -type f -name package.xml ! -path '*/build/*' ! -path '*/install/*' ! -path '*/log/*' | sort
}

collect_package_xml_values() {
    local tag="$1"
    local include_email="${2:-false}"
    local file
    local line
    local email
    local value
    local entry

    while IFS= read -r file; do
        while IFS= read -r line; do
            [[ "${line}" == *"<${tag}"*"</${tag}>"* ]] || continue

            value="$(printf '%s\n' "${line}" | sed -n "s:.*<${tag}[^>]*>\\([^<]*\\)</${tag}>.*:\\1:p")"
            value="$(trim_whitespace "${value}")"
            [[ -n "${value}" ]] || continue

            if [[ "${include_email}" == "true" ]]; then
                email="$(printf '%s\n' "${line}" | sed -n 's:.*email="\([^"]*\)".*:\1:p')"
                email="$(trim_whitespace "${email}")"
                entry="${value}"
                [[ -n "${email}" ]] && entry="${entry} <${email}>"
                printf '%s\n' "${entry}"
            else
                printf '%s\n' "${value}"
            fi
        done < "${file}"
    done < <(list_package_xml_files)
}

resolve_package_xml_label() {
    local tag="$1"
    local include_email="${2:-false}"
    local -a values=()

    mapfile -t values < <(collect_package_xml_values "${tag}" "${include_email}" | awk 'NF' | sort -u)
    if [[ ${#values[@]} -gt 0 ]]; then
        join_with_comma "${values[@]}"
    fi
}

normalize_git_url() {
    local url="$1"

    url="${url%.git}"
    case "${url}" in
        git@*:*)
            url="${url#git@}"
            printf 'https://%s\n' "${url/:/\/}"
            ;;
        ssh://*@*)
            url="${url#ssh://}"
            url="${url#*@}"
            printf 'https://%s\n' "${url}"
            ;;
        git://*)
            printf 'https://%s\n' "${url#git://}"
            ;;
        http://*|https://*)
            printf '%s\n' "${url}" | sed 's#\(https\{0,1\}://\)[^/@]*@#\1#'
            ;;
        *)
            printf '%s\n' "${url}"
            ;;
    esac
}

get_git_root() {
    git rev-parse --show-toplevel 2> /dev/null || true
}

resolve_repository_url() {
    local git_root
    local git_url

    if [[ -n "${CI_PROJECT_URL:-}" ]]; then
        printf '%s' "${CI_PROJECT_URL}"
        return 0
    fi

    if [[ -n "${GITHUB_SERVER_URL:-}" && -n "${GITHUB_REPOSITORY:-}" ]]; then
        printf '%s/%s' "${GITHUB_SERVER_URL%/}" "${GITHUB_REPOSITORY}"
        return 0
    fi

    git_root="$(get_git_root)"
    if [[ -z "${git_root}" ]]; then
        return 0
    fi

    git_url="$(git -C "${git_root}" config --get remote.origin.url 2> /dev/null || true)"
    [[ -n "${git_url}" ]] || return 0

    normalize_git_url "${git_url}"
}

resolve_repository_version() {
    local git_root
    local git_tag

    if [[ -n "${CI_COMMIT_TAG:-}" ]]; then
        printf '%s' "${CI_COMMIT_TAG}"
        return 0
    fi

    if [[ "${GITHUB_REF_TYPE:-}" == "tag" && -n "${GITHUB_REF_NAME:-}" ]]; then
        printf '%s' "${GITHUB_REF_NAME}"
        return 0
    fi

    git_root="$(get_git_root)"
    if [[ -n "${git_root}" ]]; then
        git_tag="$(git -C "${git_root}" describe --tags --exact-match HEAD 2> /dev/null || true)"
        if [[ -n "${git_tag}" ]]; then
            printf '%s' "${git_tag}"
            return 0
        fi

        git -C "${git_root}" rev-parse HEAD 2> /dev/null || true
        return 0
    fi

    if [[ -n "${GITHUB_SHA:-}" ]]; then
        printf '%s' "${GITHUB_SHA}"
        return 0
    fi

    if [[ -n "${CI_COMMIT_SHA:-}" ]]; then
        printf '%s' "${CI_COMMIT_SHA}"
    fi
}

resolve_repository_labels() {
    if [[ -z "${LABEL_MAINTAINER:-}" ]]; then
        LABEL_MAINTAINER="$(resolve_package_xml_label maintainer true)"
    fi
    LABEL_MAINTAINER="${LABEL_MAINTAINER:-TBD <TBD@TBD.xy>}"

    if [[ -z "${LABEL_AUTHORS:-}" ]]; then
        LABEL_AUTHORS="$(resolve_package_xml_label author true)"
    fi
    LABEL_AUTHORS="${LABEL_AUTHORS:-TBD}"

    if [[ -z "${LABEL_LICENSES:-}" ]]; then
        LABEL_LICENSES="$(resolve_package_xml_label license)"
    fi
    LABEL_LICENSES="${LABEL_LICENSES:-MIT}"

    if [[ -z "${LABEL_URL:-}" ]]; then
        LABEL_URL="$(resolve_repository_url)"
    fi
    LABEL_URL="${LABEL_URL:-TBD}"

    if [[ -z "${LABEL_VERSION:-}" ]]; then
        LABEL_VERSION="$(resolve_repository_version)"
    fi
    LABEL_VERSION="${LABEL_VERSION:-TBD}"

    export LABEL_MAINTAINER LABEL_AUTHORS LABEL_LICENSES LABEL_URL LABEL_VERSION
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    resolve_repository_labels
    printf 'LABEL_MAINTAINER=%s\n' "${LABEL_MAINTAINER}"
    printf 'LABEL_AUTHORS=%s\n' "${LABEL_AUTHORS}"
    printf 'LABEL_LICENSES=%s\n' "${LABEL_LICENSES}"
    printf 'LABEL_URL=%s\n' "${LABEL_URL}"
    printf 'LABEL_VERSION=%s\n' "${LABEL_VERSION}"
fi