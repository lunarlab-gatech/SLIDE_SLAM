#!/bin/bash

# Set parameters
SESSION_NAME=slide_slam_nodes
BAG_PLAY_RATE=0.5
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
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; roslaunch scan2shape_launch infer_node.launch param_file:=config/CoPeD/infer_node_params_wilbur.yaml" Enter
tmux select-pane -t $SESSION_NAME:1.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; roslaunch object_modeller sync_semantic_measurements.launch" Enter
tmux select-pane -t $SESSION_NAME:1.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; roslaunch sloam single_robot_sloam_test_LiDAR.launch enable_rviz:=true" Enter
tmux select-pane -t $SESSION_NAME:1.3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; roslaunch scan2shape_launch process_cloud_node_outdoor_with_ns.launch  param_file:=config/CoPeD/process_cloud_node_params_wilbur.yaml" Enter
tmux select-pane -t $SESSION_NAME:1.4
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; roslaunch faster_lio mapping_ouster64.launch param_file:=config/CoPeD/wilburOusterOS1-64.yaml" Enter
tmux select-pane -t $SESSION_NAME:1.5
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 18; cd $BAG_DIR && rosbag play FOREST_wilbur_arl_outdoor_2023_05_19_06_2023-05-19-16-27-47.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /wilbur/lidar_points /wilbur/imu/data /wilbur/stereo_left/image_rect_color/compressed /wilbur/lidar_points:=/robot0/lidar_points /wilbur/imu/data:=/robot0/imu/data" Enter
tmux select-layout -t $SESSION_NAME tiled

# Add window for roscore
tmux new-window -t $SESSION_NAME -n "roscore"
tmux split-window -h -t $SESSION_NAME
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:2.0
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
tmux select-pane -t $SESSION_NAME:2.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; rosparam set /use_sim_time true" Enter
tmux select-pane -t $SESSION_NAME:2.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; rosrun image_transport republish compressed in:=/wilbur/stereo_left/image_rect_color raw out:=/wilbur/stereo_left/image_rect_color/uncompressed _image_transport:=compressed
" Enter

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

# Go back to the first window
tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
