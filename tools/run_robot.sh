#!/usr/bin/env bash
# One base terminal; select VR or replay in the upper terminal.
set -e

usage() {
    printf '%s\n' 'Usage: bash tools/run_robot.sh base [--no-impedance]' \
        '       bash tools/run_robot.sh vr|replay|menu|prepare' \
        '  base     Start servo, middle and topic upper; impedance is on by default.' \
        '  vr       Start VR, executor, browser and recording.' \
        '  replay   Start executor, Prepare, then the numbered menu (default).' \
        '  menu     Start executor and menu for an already mounted tool.' \
        '  prepare  Prepare only.' \
        'Upper applications read the base mode automatically.'
}
mode="${1:-replay}"
if (( $# )); then shift; fi
case "$mode" in
    -h|--help) usage; exit 0 ;;
    base)
        if (( $# > 1 )) || [[ $# == 1 && "$1" != --no-impedance ]]; then usage >&2; exit 2; fi ;;
    vr|replay|menu|prepare|executor)
        if (( $# )); then usage >&2; exit 2; fi ;;
    *) usage >&2; exit 2 ;;
esac

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
source "$root/artifacts/robot/upper/env.sh"
if [[ "$mode" == base ]]; then
    sudo -v
    exec python3 -u "$root/tools/deployment/deploy.py" services "$@"
fi
exec python3 -u "$root/tools/deployment/deploy.py" operator --frontend "$mode"
