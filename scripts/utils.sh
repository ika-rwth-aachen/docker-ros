#!/bin/bash

require_var() {
    if [[ -z "${!1}" ]]; then
        echo "Environment variable '${1}' is required"
        exit 1
    fi
}

open_log_group() {
    echo "::group::[docker-ros] ${1}"
}

close_log_group() {
    echo "::endgroup::"
}
