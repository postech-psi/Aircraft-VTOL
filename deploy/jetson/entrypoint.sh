#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /workspace/ros2/ws/install/setup.bash ]; then
    source /workspace/ros2/ws/install/setup.bash
fi

exec "$@"
