# This message combines a odometry message
# and a GPS message, syncronized with the timestamp
# in header.

std_msgs/Header header
nav_msgs/Odometry odom
sensor_msgs/NavSatFix gps