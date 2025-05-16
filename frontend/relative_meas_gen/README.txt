Current Status of Relative Measurements with sloam for CoPeD

How to run:
- catkin_make && source devel/setup.bash
- roscd relative_meas_gen/bash_scripts
- ./race1_to_wilbur_run.sh
    This loads both rosbags, the apriltag measurement node, the measurement/odom sync node
- Once both rosbags are loaded and paused, run tmux send-keys -t race1_to_wilbur:1.1 ' '; tmux send-keys -t race1_to_wilbur:1.2 ' '

Working Functionality:
- race1 and wilbur odoms are published
- apriltag measurements are published
- synched odom and relative measurements are published
- wilbur path is drawn (incorrectly) in rviz

What needs to be done:
- race1 trajectory needs to be drawn in rviz
- relative measurements need to be drawn in rviz

Bugs:
- Wilbur trajectory is super wrong: the axes move well but the trajectory is drawn in long jagged lines
- tf tree for race1 is not being published: I believe this is why it is not showing up
