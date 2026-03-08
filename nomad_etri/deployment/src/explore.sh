#!/bin/bash

# Create a new tmux session
session_name="vint_locobot_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# add
tmux selectp -t 3    # select the new, second (3) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# Run the roslaunch command in the first pane
tmux select-pane -t 0
tmux send-keys "workon nomad" Enter # add
tmux send-keys "ros2 launch vint_locobot vint_locobot_launch.py" Enter

# Run the navigate.py script with command line args in the second pane
tmux select-pane -t 1
# Activate your Python environment if needed
# source /path/to/your/python/environment/bin/activate
tmux send-keys "workon nomad" Enter
tmux send-keys "python3 explore.py $@" Enter

# Run the teleop.py script in the third pane
tmux select-pane -t 2
# Activate your Python environment if needed
# source /path/to/your/python/environment/bin/activate
tmux send-keys "workon nomad" Enter
tmux send-keys "python3 joy_teleop.py" Enter

# Run the pd_controller.py script in the fourth pane
tmux select-pane -t 3
# Activate your Python environment if needed
# source /path/to/your/python/environment/bin/activate
tmux send-keys "workon nomad" Enter
tmux send-keys "python3 pd_controller.py" Enter

# add
tmux select-pane -t 4
tmux send-keys "workon nomad" Enter
tmux send-keys "rviz2 nomad.rviz" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
