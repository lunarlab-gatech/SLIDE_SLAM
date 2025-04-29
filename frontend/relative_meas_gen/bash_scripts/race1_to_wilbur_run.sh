#!/bin/bash

# This script runs the backend of SlideSLAM (sloam) on simulated jackal 
# robots from the multi-UGV-gazebo-sim. In order to bypass SlideSLAM's
# frontend, we spawn a publisher to convert GT information into fake 
# frontend measurements.

# Set parameters
SESSION_NAME=race1_to_wilbur
BAG_PLAY_RATE=1
BAG_DIR='/opt/slideslam_docker_ws/src/SLIDE_SLAM/bags/CoPeD/FOREST/'
SETUP_ROS_STRING="export ROS_MASTER_URI=http://localhost:11311"

# Set Display Settings
CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ];
then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

# Start Tmux Session if not already in one
if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# Make mouse useful in copy mode
tmux setw -g mouse on

# Setup the panes for the main window
tmux new-window -t $SESSION_NAME -n "Main"
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.0
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.3
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.0
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.3
tmux split-window -h -t $SESSION_NAME

# Setup commands for main window
tmux select-pane -t $SESSION_NAME:1.0
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch relative_meas_gen race1_wilbur_RelativeMeasSync.launch" Enter
tmux select-pane -t $SESSION_NAME:1.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; cd $BAG_DIR && rosbag play FOREST_wilbur_arl_outdoor_2023_05_19_06_2023-05-19-16-27-47.bag --clock --pause -r $BAG_PLAY_RATE -s 251.4" Enter
tmux select-pane -t $SESSION_NAME:1.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; cd $BAG_DIR && rosbag play FOREST_race1_2023_05_19_04_29_PM_1.bag --clock --pause -r $BAG_PLAY_RATE /clock:=/race1_clock -s 150" Enter
tmux select-pane -t $SESSION_NAME:1.3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 12; roslaunch relative_meas_gen race1_relative_meas_gen.launch" Enter
tmux select-pane -t $SESSION_NAME:1.4
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 12; rqt_image_view" Enter
tmux select-layout -t $SESSION_NAME tiled

# Add window for roscore
tmux new-window -t $SESSION_NAME -n "roscore"
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:2.0
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:2.2
tmux split-window -v -t $SESSION_NAME

# Set commands for roscore window
tmux select-pane -t $SESSION_NAME:2.0
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
tmux select-pane -t $SESSION_NAME:2.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch sloam decentralized_sloam.launch hostRobot_ID:=0 odom_topic:=/wilbur/odom turn_off_rel_inter_robot_factor:=false param_file:=sloam_multiUGV.yaml" Enter
tmux select-pane -t $SESSION_NAME:2.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; rosparam set /use_sim_time true" Enter
tmux select-pane -t $SESSION_NAME:2.3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch sloam decentralized_sloam.launch hostRobot_ID:=2 odom_topic:=/race1/mavros/local_position/odom enable_rviz:=true turn_off_rel_inter_robot_factor:=false param_file:=sloam_multiUGV.yaml" Enter

# tmux new-window -t $SESSION_NAME -n "Sync"
# tmux send-keys -t $SESSION_NAME "sleep 30; tmux send-keys -t $SESSION_NAME:1.1 ' '; tmux send-keys -t $SESSION_NAME:1.2 ' '" Enter

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

# Go back to the first window
tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

