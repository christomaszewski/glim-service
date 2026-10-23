#!/usr/bin/env bash
# Sources the ROS environment and the GLIM install, applies the launcher-rendered zenoh session
# overrides (rmw_zenoh only), then execs the given command (the supervisor, by default). GLIM is
# started by the supervisor and inherits this environment.
#
# NOTE: ROS setup.bash scripts reference variables that may not be set yet
# (AMENT_TRACE_SETUP_FILES, AMENT_PYTHON_EXECUTABLE, etc.) — `set -u` around the sourcing would
# abort the shell.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-lyrical}"
# shellcheck disable=SC1090
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] && source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
[[ -f /opt/glim/setup.bash ]] && source /opt/glim/setup.bash

# Opt-in zenoh session overrides (config `zenoh:` block -> GLIM_ZENOH_OVERRIDE, a launcher-rendered
# "path=json;..." pair string). rmw_zenoh applies ZENOH_CONFIG_OVERRIDE pairs on top of whatever
# session config it loads; ours are appended AFTER any injected value, so the config wins on key
# conflict. Inert under Fast DDS.
if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_zenoh_cpp" && -n "${GLIM_ZENOH_OVERRIDE:-}" ]]; then
  export ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:+${ZENOH_CONFIG_OVERRIDE};}${GLIM_ZENOH_OVERRIDE}"
  echo "ros-entrypoint: ZENOH_CONFIG_OVERRIDE=${ZENOH_CONFIG_OVERRIDE}" >&2
fi

exec "$@"
