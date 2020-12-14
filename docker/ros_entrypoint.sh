#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/melodic/setup.bash"
source "/home/developer/project11/catkin_ws/devel/setup.bash"

#mon launch project11_vorc cora_backseat.launch
sleep 5
roslaunch project11_vorc cora_backseat.launch
