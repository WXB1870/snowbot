#!/usr/bin/env bash
# Compatibility for the earlier preset entry; implementation lives in run_robot.sh.
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
case "${1:-}" in
    start) shift; set -- base "$@" ;;
    run) shift; set -- replay "$@" ;;
esac
exec bash "$root/tools/run_robot.sh" "$@"
