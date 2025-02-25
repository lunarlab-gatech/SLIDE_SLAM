# A message containing relative inter-robot measurements 
#
# relativePose contains the relative pose difference from the
# current robot to robot with index "robotId". header contains
# the timestamp of this measurement. odometry contains the
# closest synced odometry measurement.

std_msgs/Header header
geometry_msgs/Pose relativePose
nav_msgs/Odometry odometry
int8 robotId