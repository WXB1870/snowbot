#!/usr/bin/env bash
# Installed VR physics launcher; no framework checkout is needed.
set -euo pipefail
if [ "${1:-}" = --vr ]; then shift; fi
bindu_sdk="${ISAAC_SIM_PATH:-$HOME/isaacsim-6.2.0}"
bindu_state="${BINDU_ISAAC_STATE:-${BINDU_FRAMEWORK_SOURCE:-$HOME/.cache/snowbot}/artifacts/isaac62-state}"
bindu_contracts_prefix="$(python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("bindu_contracts"))')"
bindu_hardware_prefix="$(python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("bindu_hardware"))')"
bindu_kinematics_prefix="$(python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("bindu_kinematics"))')"
snowbot_hardware_prefix="$(python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("snowbot_hardware"))')"
test -x "$bindu_sdk/python.sh"
mkdir -p "$bindu_state"/{kit,xdg_cache,xdg_config,xdg_data,cuda_cache,tmp}
unset PYTHONPATH PYTHONHOME PYTHONEXE LD_LIBRARY_PATH LD_PRELOAD
unset CARB_APP_PATH EXP_PATH ISAAC_PATH OLD_PYTHONPATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH CUDA_HOME CUDA_PATH
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export XDG_CACHE_HOME="$bindu_state/xdg_cache" XDG_CONFIG_HOME="$bindu_state/xdg_config"
export XDG_DATA_HOME="$bindu_state/xdg_data" CUDA_CACHE_PATH="$bindu_state/cuda_cache"
export TMPDIR="$bindu_state/tmp"
bindu_nv_dirs=""
for bindu_lib in "$bindu_sdk/exts/isaacsim.pip.nv/pip_prebundle/nvidia"/*/lib; do
  test -d "$bindu_lib" || continue
  bindu_nv_dirs="${bindu_nv_dirs:+$bindu_nv_dirs:}$bindu_lib"
done
test -n "$bindu_nv_dirs"
set +u
source /opt/ros/jazzy/setup.bash
source "${BINDU_ISAAC_ROS_PREFIX:?VR physics requires the current bindu_interfaces install}/share/bindu_interfaces/local_setup.bash"
COLCON_CURRENT_PREFIX="$bindu_contracts_prefix" source "$bindu_contracts_prefix/share/bindu_contracts/package.bash"
COLCON_CURRENT_PREFIX="$bindu_hardware_prefix" source "$bindu_hardware_prefix/share/bindu_hardware/package.bash"
COLCON_CURRENT_PREFIX="$bindu_kinematics_prefix" source "$bindu_kinematics_prefix/share/bindu_kinematics/package.bash"
COLCON_CURRENT_PREFIX="$snowbot_hardware_prefix" source "$snowbot_hardware_prefix/share/snowbot_hardware/package.bash"
COLCON_CURRENT_PREFIX="${BINDU_ISAAC_APP_PREFIX:?VR physics requires the current Snowbot application install}" \
  source "$BINDU_ISAAC_APP_PREFIX/share/snowbot_bringup/package.bash"
set -u
export LD_LIBRARY_PATH="$bindu_nv_dirs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$bindu_sdk/python.sh" --no-ros-env -m snowbot_bringup.simulation.snowbot_isaac "$@" \
  --portable --portable-root "$bindu_state/kit" --/isaac/startup/ros_bridge_extension=
