#!/bin/bash


SESSION_NAME=sync_nodes

CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ];
then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

SETUP_ROS_STRING="source ~/xmas_slam_ws/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11311"

# Python args
ODOM_TOPIC="/dragonfly67/quadrotor_ukf/control_odom"
# ODOM_TOPIC="/Odometry"
# ODOM_TOPIC="/scarab41/odom_laser"
# ODOM_TOPIC="/quadrotor1/lidar_odom"
# ODOM_TOPIC="/quadrotor1/lidar_odom"
# POINT_CLOUD_NS="/quadrotor1/"
POINT_CLOUD_NS="/"


# Make mouse useful in copy mode
tmux setw -g mouse on


tmux rename-window -t $SESSION_NAME "Core"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; rosparam set /use_sim_time True" Enter 



tmux new-window -t $SESSION_NAME -n "Bag"
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; rosbag play --clock /home/sam/bags/pennovation-bags/generic_sloam_2_robots_multi_robot_MOST_IMPORTANT_2022-06-30-22-50-33.bag -s 30"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; rosbag play --clock /home/sam/bags/xmas-slam-bags/test-indoor-sloam-and-SLC-cylinder-odom-only-bag-2023-10-26-17-58-19.bag -s 30"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; rosparam get /use_sim_time; python ./merge_synced_measurements.py" Enter
tmux select-layout -t $SESSION_NAME tiled



tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; python ./cylinder_plane_modeller.py --point_cloud_ns $POINT_CLOUD_NS" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; python ./sync_cylinder_odom.py --odom_topic $ODOM_TOPIC" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; python ./sync_cuboid_odom.py --odom_topic $ODOM_TOPIC" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; python ./sync_centroid_odom.py --odom_topic $ODOM_TOPIC" Enter
tmux select-layout -t $SESSION_NAME tiled

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; rviz -d ../rviz/object_modeller.rviz"

# sloam
tmux new-window -t $SESSION_NAME -n "sloam"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 2; roslaunch sloam single_robot_sloam_test.launch"


# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"


tmux select-window -t $SESSION_NAME:5
tmux -2 attach-session -t $SESSION_NAME
