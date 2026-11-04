#!/usr/bin/env bash
# Operator entry: terminal 4 executor; terminal 5 Prepare followed by the menu.
set -e

usage() {
    printf '%s\n' 'Usage: bash tools/run_primitives.sh [executor|run|prepare|menu]' \
        '  executor  Start the impedance executor and recorder (terminal 4).' \
        '  run       Prepare, then open the numbered action menu (default).' \
        '  prepare   Prepare only; open/close grippers after operator Enter.' \
        '  menu      Open the menu for an already mounted tool.'
}
mode="${1:-run}"
if (( $# > 1 )); then usage >&2; exit 2; fi
case "$mode" in
    -h|--help) usage; exit 0 ;;
    executor|run|prepare|menu) ;;
    *) usage >&2; exit 2 ;;
esac

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
source "$root/artifacts/robot/upper/env.sh"
cfg="$(ros2 pkg prefix --share snowbot_bringup)/config/robots/snowbot"
runs="$root/artifacts/robot/primitive-runs"
console="$root/artifacts/robot/startup-logs"
run_stamp="$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$runs" "$console"
exec > >(tee -ai "$console/primitives-$mode-$run_stamp.log") 2>&1

if [[ "$mode" == executor ]]; then
    exec ros2 launch snowbot_bringup snowbot_adapter.launch.py \
        runtime_mode:=real device_backend:=snowbot_vendor \
        namespace:=bindu_snowbot vendor_namespace:=/ \
        profile:="$cfg/snowbot_real_position.json" \
        parameters_config:="$cfg/runtime_parameters.json" gripper_config:="$cfg/grippers.json" \
        position_protocol:=session_v3 execution_mode:=impedance command_enabled:=true \
        recording_mode:=compact output:="$runs"
fi

replay=(python3 -u -m snowbot_bringup.teleop.snowbot_primitive_replay \
    --library "$cfg/primitive-library/manifest.json" --execution-mode impedance --execute)
if [[ "$mode" == run || "$mode" == prepare ]]; then
    "${replay[@]}" --action Prepare --output "$runs/Prepare-$run_stamp.jsonl"
    if [[ "$mode" == prepare ]]; then exit 0; fi
fi
exec "${replay[@]}" --interactive --tool-id g099-curved --output-dir "$runs"
