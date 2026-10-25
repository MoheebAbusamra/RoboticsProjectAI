#!/usr/bin/env bash
# Start simulation + localization + navigation + fleet management.
set -e
cd "$(dirname "$0")"
source ./env.sh
# Software rendering is reliable with VirtualBox's virtual graphics adapter.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
# Bound Mesa's raster workers so virtual CPU scheduling stays responsive.
export LP_NUM_THREADS="${LP_NUM_THREADS:-4}"
# Reuse the logged-in Ubuntu desktop when launched over SSH.
if [[ -z "${DISPLAY:-}" ]]; then
  _factory_auth=$(find "/run/user/$(id -u)" -maxdepth 1 -name '.mutter-Xwaylandauth.*' -print -quit 2>/dev/null)
  if [[ -n "$_factory_auth" ]]; then
    export DISPLAY=:0
    export XAUTHORITY="$_factory_auth"
  fi
fi
exec ros2 launch factory_bot demo.launch.py "$@"
