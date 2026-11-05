#!/usr/bin/env bash
# Source this file in a vendor-only terminal. It never starts a ROS node or driver.
# The optional argument is the vendor workspace; defaults match robot 118.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source env_118.sh [/home/cc/snowbot_ws]' >&2
    exit 2
fi

_bindu_vendor_env_118() {
    local bindu_vendor_workspace="${1:-/home/cc/snowbot_ws}"
    local bindu_vendor_file
    if [[ -n "${VIRTUAL_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
        echo 'Use a terminal without a Python venv or Conda environment for the vendor drivers.' >&2
        return 2
    fi
    local bindu_vendor_distro=jazzy
    local bindu_vendor_pinocchio=4.1.0
    local bindu_vendor_profile="$(dirname "$bindu_vendor_workspace")/upper/deployment.json"
    if [[ -r "$bindu_vendor_profile" ]]; then
        bindu_vendor_distro="$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["ros"])' "$bindu_vendor_profile")" || return 2
        case "$bindu_vendor_distro" in
            humble) bindu_vendor_pinocchio=3.9.0 ;;
            jazzy) ;;
            *) echo "Unsupported deployment ROS distro: $bindu_vendor_distro" >&2; return 2 ;;
        esac
    fi
    if [[ "${ROS_DISTRO:-$bindu_vendor_distro}" != "$bindu_vendor_distro" ]]; then
        echo "Deployment requires ROS $bindu_vendor_distro; use a terminal without another ROS distro." >&2
        return 2
    fi
    for bindu_vendor_file in "/opt/ros/$bindu_vendor_distro/setup.bash" \
        "$bindu_vendor_workspace/install/local_setup.bash" \
        "/opt/openrobots/lib/libpinocchio_parsers.so.$bindu_vendor_pinocchio" \
        "/opt/openrobots/lib/libpinocchio_default.so.$bindu_vendor_pinocchio"; do
        if [[ ! -r "$bindu_vendor_file" ]]; then
            echo "Missing vendor dependency: $bindu_vendor_file" >&2
            return 2
        fi
    done
    source "/opt/ros/$bindu_vendor_distro/setup.bash" || return
    source "$bindu_vendor_workspace/install/local_setup.bash" || return
    export LD_LIBRARY_PATH="/opt/openrobots/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
}

if _bindu_vendor_env_118 "$@"; then
    unset -f _bindu_vendor_env_118
else
    unset -f _bindu_vendor_env_118
    return 2
fi
