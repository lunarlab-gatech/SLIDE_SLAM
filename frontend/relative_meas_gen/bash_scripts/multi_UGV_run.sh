#!/bin/bash

# This script runs the backend of SlideSLAM (sloam) on simulated jackal 
# robots from the multi-UGV-gazebo-sim. In order to bypass SlideSLAM's
# frontend, we spawn a publisher to convert GT information into fake 
# frontend measurements.

# Set parameters
SESSION_NAME=multi_UGV_run
BAG_PLAY_RATE=1
BAG_DIR='/opt/slideslam_docker_ws/src/SLIDE_SLAM/bags/multi-UGV-gazebo-sim-data/'
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
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch relative_meas_gen odomRelativeMeasSync.launch" Enter
tmux select-pane -t $SESSION_NAME:1.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 18; cd $BAG_DIR && rosbag play stuff_world-two_jackal_amcl.bag --clock -r $BAG_PLAY_RATE -s 0 --topic /jackal0/odometry/local_filtered /jackal1/odometry/local_filtered /gazebo/model_states /jackal0/odometry/local_filtered:=/robot0/odometry/local_filtered /jackal1/odometry/local_filtered:=/robot1/odometry/local_filtered" Enter
tmux select-pane -t $SESSION_NAME:1.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch relative_meas_gen multiUGVToSlideSLAM.launch publish_for_rviz:=true" Enter
tmux select-pane -t $SESSION_NAME:1.3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch object_modeller sync_semantic_measurements.launch robot_name:=robot0 odom_topic:=/robot0/odometry/local_filtered" Enter
tmux select-pane -t $SESSION_NAME:1.4
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch object_modeller sync_semantic_measurements.launch robot_name:=robot1 odom_topic:=/robot1/odometry/local_filtered" Enter
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
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch sloam decentralized_sloam.launch hostRobot_ID:=0 odom_topic:=/robot0/odometry/local_filtered enable_rviz:=true turn_off_rel_inter_robot_factor:=true  param_file:=sloam_multiUGV.yaml" Enter
tmux select-pane -t $SESSION_NAME:2.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; rosparam set /use_sim_time true" Enter
tmux select-pane -t $SESSION_NAME:2.3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch sloam decentralized_sloam.launch hostRobot_ID:=1 odom_topic:=/robot1/odometry/local_filtered turn_off_rel_inter_robot_factor:=true param_file:=sloam_multiUGV.yaml" Enter

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

# Go back to the first window
tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
