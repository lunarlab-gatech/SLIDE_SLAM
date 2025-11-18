#!/bin/bash

SESSION_NAME=tmux_pipeline

## Aliases are not expanded in non-intereactive bash
shopt -s expand_aliases
source ~/.bash_aliases

## Check alias set
if [ "$(type -t runyuezhansloamdocker)" = 'alias' ]; then
    echo 'runyuezhansloamdocker is an alias'
else
    echo 'runyuezhansloamdocker is not an alias, please set it first!'
    exit
fi
if [ "$(type -t rundetectnode)" = 'alias' ]; then
    echo 'rundetectnode is an alias'
else
    echo 'rundetectnode is not an alias, please set it first!'
    exit
fi
if [ "$(type -t runprocesscloudnode)" = 'alias' ]; then
    echo 'runprocesscloudnode is an alias'
else
    echo 'runprocesscloudnode is not an alias, please set it first!'
    exit
fi

if [ -z ${TMUX} ];
then
  tmux has-session -t $SESSION_NAME 2>/dev/null
  if [ "$?" -eq 1 ] ; then
    # Set up session
    TMUX= tmux new-session -s $SESSION_NAME -d
    echo "Starting new session."
  else
    echo "Session exist, kill it first."
  fi
else
  echo "Already in tmux, leave it first."
  exit
fi

# runyuezhansloamdocker = run_sloam_node_xu.sh
tmux setw -g mouse on

tmux rename-window -t $SESSION_NAME "Core"
tmux send-keys -t $SESSION_NAME "roscore" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; rosparam set /use_sim_time True" Enter

tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "rosbag play /home/sam/bags/yuezhan-bags/first_floor_handcarry_2023-04-04-14-11-02.bag --clock -r 2"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1;  roslaunch sloam run_indoor_large_scale_exploration.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1;  roslaunch rgb_sem_segmentation rgb_segmentation_bag.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1;  roscd scan2shape_launch; python ../script/process_cloud_node.py" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Closure"
tmux send-keys -t $SESSION_NAME "sleep 1; rosrun loop_closure loop_closure_server_node" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "roscd scan2shape_launch; python ../script/pub_loop_closure_trigger.py"
tmux select-layout -t $SESSION_NAME tiled


tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t tmux_pipeline"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

clear