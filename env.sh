#!/usr/bin/env bash
# Source this file in each terminal before using this workspace.
_factory_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$_factory_root/install/local_setup.bash" ]]; then
  echo "Build first: cd $_factory_root && colcon build --symlink-install" >&2
  return 1
fi
source "$_factory_root/install/local_setup.bash"
unset _factory_root
