# A message containing relative inter-robot measurements 
#
# relativePose contains the relative pose difference from the
# current robot with index "robotIdObserver" to robot with 
# index "robotIdObserved". header contains the timestamp of this 
# measurement.

std_msgs/Header header
geometry_msgs/Pose relativePose
int8 robotIdObserver
int8 robotIdObserved