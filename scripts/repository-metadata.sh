#!/bin/bash

trim_whitespace() {
    local value="$*"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"
    printf '%s' "${value}"
}

normalize_optional_label() {
    local var_name="$1"
    local value="${!var_name:-}"

    value="$(trim_whitespace "${value}")"
    if [[ -n "${value}" ]]; then
        printf -v "${var_name}" '%s' "${value}"
        export "${var_name}"
    else
        unset "${var_name}"
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

resolve_repository_root() {
    local git_root

    git_root="$(get_git_root)"
    if [[ -n "${git_root}" ]]; then
        printf '%s\n' "${git_root}"
    else
        pwd
    fi
}

find_repository_license_file() {
    local repo_root="$1"
    local candidate

    for candidate in LICENSE LICENSE.txt LICENSE.md LICENSE.rst COPYING COPYING.txt COPYING.md COPYING.rst; do
        if [[ -f "${repo_root}/${candidate}" ]]; then
            printf '%s\n' "${repo_root}/${candidate}"
            return 0
        fi
    done
}

resolve_license_identifier_from_file() {
    local file="$1"
    local spdx_identifier
    local first_non_empty_line

    spdx_identifier="$(sed -n 's/.*SPDX-License-Identifier:[[:space:]]*\([^[:space:]]\+\).*/\1/p' "${file}" | head -n 1)"
    if [[ -n "${spdx_identifier}" ]]; then
        printf '%s' "${spdx_identifier}"
        return 0
    fi

    if grep -qi 'Apache License' "${file}" && grep -qi 'Version 2\.0' "${file}"; then
        printf 'Apache-2.0'
        return 0
    fi

    if grep -qiE '(^|[[:space:]])MIT License([[:space:]]|$)|The MIT License' "${file}"; then
        printf 'MIT'
        return 0
    fi

    if grep -qi 'BSD 3-Clause' "${file}"; then
        printf 'BSD-3-Clause'
        return 0
    fi

    if grep -qi 'BSD 2-Clause' "${file}"; then
        printf 'BSD-2-Clause'
        return 0
    fi

    if grep -qi 'Mozilla Public License' "${file}" && grep -qi 'Version 2\.0' "${file}"; then
        printf 'MPL-2.0'
        return 0
    fi

    if grep -qi 'GNU GENERAL PUBLIC LICENSE' "${file}" && grep -qi 'Version 3' "${file}"; then
        printf 'GPL-3.0'
        return 0
    fi

    if grep -qi 'GNU GENERAL PUBLIC LICENSE' "${file}" && grep -qi 'Version 2' "${file}"; then
        printf 'GPL-2.0'
        return 0
    fi

    if grep -qi 'GNU LESSER GENERAL PUBLIC LICENSE' "${file}" && grep -qi 'Version 3' "${file}"; then
        printf 'LGPL-3.0'
        return 0
    fi

    if grep -qi 'GNU LESSER GENERAL PUBLIC LICENSE' "${file}" && grep -qi 'Version 2\.1' "${file}"; then
        printf 'LGPL-2.1'
        return 0
    fi

    first_non_empty_line="$(awk 'NF { print; exit }' "${file}")"
    first_non_empty_line="$(trim_whitespace "${first_non_empty_line}")"
    printf '%s' "${first_non_empty_line}"
}

resolve_repository_license() {
    local repo_root
    local license_file

    repo_root="$(resolve_repository_root)"
    license_file="$(find_repository_license_file "${repo_root}")"
    [[ -n "${license_file}" ]] || return 0

    resolve_license_identifier_from_file "${license_file}"
}

resolve_github_repository_description() {
    local event_path="${GITHUB_EVENT_PATH:-}"
    local python_bin

    [[ -n "${event_path}" && -f "${event_path}" ]] || return 0

    python_bin="$(command -v python3 2> /dev/null || command -v python 2> /dev/null || true)"
    [[ -n "${python_bin}" ]] || return 0

    "${python_bin}" - "${event_path}" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], encoding="utf-8") as event_file:
        payload = json.load(event_file)
except Exception:
    sys.exit(0)

description = (((payload or {}).get("repository") or {}).get("description") or "").strip()
if description:
    print(description, end="")
PY
}

resolve_repository_description() {
    if [[ -n "${CI_PROJECT_DESCRIPTION:-}" ]]; then
        # GitLab
        printf '%s' "${CI_PROJECT_DESCRIPTION}"
        return 0
    fi

    # GitHub
    resolve_github_repository_description
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
    if [[ -z "${LABEL_DESCRIPTION:-}" ]]; then
        LABEL_DESCRIPTION="$(resolve_repository_description)"
    fi

    if [[ -z "${LABEL_LICENSES:-}" ]]; then
        LABEL_LICENSES="$(resolve_repository_license)"
    fi

    if [[ -z "${LABEL_URL:-}" ]]; then
        LABEL_URL="$(resolve_repository_url)"
    fi

    if [[ -z "${LABEL_VERSION:-}" ]]; then
        LABEL_VERSION="$(resolve_repository_version)"
    fi

    normalize_optional_label "LABEL_MAINTAINER"
    normalize_optional_label "LABEL_AUTHORS"
    normalize_optional_label "LABEL_DESCRIPTION"
    normalize_optional_label "LABEL_LICENSES"
    normalize_optional_label "LABEL_URL"
    normalize_optional_label "LABEL_VERSION"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    resolve_repository_labels
    printf 'LABEL_MAINTAINER=%s\n' "${LABEL_MAINTAINER}"
    printf 'LABEL_AUTHORS=%s\n' "${LABEL_AUTHORS}"
    printf 'LABEL_DESCRIPTION=%s\n' "${LABEL_DESCRIPTION}"
    printf 'LABEL_LICENSES=%s\n' "${LABEL_LICENSES}"
    printf 'LABEL_URL=%s\n' "${LABEL_URL}"
    printf 'LABEL_VERSION=%s\n' "${LABEL_VERSION}"
fi