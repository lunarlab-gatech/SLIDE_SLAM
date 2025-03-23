# A message containing relative inter-robot measurements 
# with synced odometry.
#
# relativePose contains the relative pose difference from the
# current robot with index "robotIdObserver" to robot with 
# index "robotIdObserved". header contains the timestamp of this 
# measurement. odometry contains the closest synced odometry 
# measurement for each robot.

std_msgs/Header header
geometry_msgs/Pose relativePose
int8 robotIdObserver
int8 robotIdObserved
nav_msgs/Odometry odometryObserver
nav_msgs/Odometry odometryObserved