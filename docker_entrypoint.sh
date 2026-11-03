#!/bin/bash

set -e

# Source ROS 2 base environment
source /opt/ros/humble/setup.bash

# Activate simulator virtual environment and source workspaces
source /sim_ws/install/setup.bash
source /rmpc_ws/install/setup.bash

exec "$@"