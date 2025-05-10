#!/bin/bash

ssh -t root@kria "tmux new-session -s mmr_kria \; split-window -h\; select-pane -t 0 \; send-keys 'ros2 launch control_node control_node.launch.py' C-m \; select-pane -t 1 \; send-keys 'ros2 launch canopen_bridge canopen_bridge_launch.py' C-m"
