#!/usr/bin/env bash
# Source this file: source scripts/use_ros.sh
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS Jazzy is not installed yet.' >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash
# Keep practice data local to this computer until multi-PC communication is intentional.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=87
