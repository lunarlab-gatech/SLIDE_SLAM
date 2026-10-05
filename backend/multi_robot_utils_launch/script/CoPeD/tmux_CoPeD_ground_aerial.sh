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
tmux select-pane -t $SESSION_NAME:1.1
tmux select-pane -t $SESSION_NAME:1.2
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; roslaunch faster_lio mapping_ouster64.launch param_file:=config/CoPeD/wandaOusterOS1-64.yaml odom_topic:=/robot0/Odometry" Enter
tmux select-pane -t $SESSION_NAME:1.3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; roslaunch relative_meas_gen odomGPSSync.launch gps_topic:=/wanda/gps slop:=0.1 odom_topic:=/robot0/Odometry" Enter
tmux select-pane -t $SESSION_NAME:1.4
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; cd $BAG_DIR && rosbag play FOREST_wanda_arl_outtdoor_2023_05_19_06_2023-05-19-16-27-46.bag --clock -r $BAG_PLAY_RATE -s 0 --topics /wanda/lidar_points /wanda/imu/data /wanda/gps /wanda/lidar_points:=/robot0/lidar_points /wanda/imu/data:=/robot0/imu/data" Enter
tmux select-pane -t $SESSION_NAME:1.5
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 10; cd $BAG_DIR && rosbag play FOREST_race1_2023_05_19_04_29_PM_1.bag -r $BAG_PLAY_RATE -s 0" Enter
tmux select-layout -t $SESSION_NAME tiled

# Add window for roscore
tmux new-window -t $SESSION_NAME -n "roscore"
tmux split-window -h -t $SESSION_NAME
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:2.0
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
tmux select-pane -t $SESSION_NAME:2.1
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; rosparam set /use_sim_time true" Enter
tmux select-pane -t $SESSION_NAME:2.2


# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

# Go back to the first window
tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
