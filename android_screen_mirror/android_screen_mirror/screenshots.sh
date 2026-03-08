# !/bin/bash

# Create a new tmux session
session_name="screenshots_$(date +%s)"
# detached mode : allows the script to continue setting up the environment
tmux new-session -d -s $session_name

# Split the window into two panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves

# Run the teleop.py script in the third pane
tmux select-pane -t 0
# tmux send-keys "conda activate vint_deployment" Enter
tmux send-keys "cd /home/wonjun/projects/phone_mirroring && source install/setup.bash" Enter
tmux send-keys "ros2 run android_screen_mirror screen_mirror_node" Enter

# Run the pd_controller.py script in the fourth pane
tmux select-pane -t 1
tmux send-keys "cd /home/wonjun/projects/phone_mirroring && source install/setup.bash" Enter
tmux send-keys "ros2 run android_screen_mirror subscribe_node" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
