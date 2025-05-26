#!/bin/bash


source /opt/ros/foxy/setup.bash

colcon build --symlink-install
if [ -d /home/mmr/global_planner_env/install ]; then
    exit 1
fi

source install/setup.bash
ros2 launch global_planner global_planner.launch.py 