# This message is the main message for semantic SLAM, where synced measurements are recorded. 
# If a certain type of measurements is absent, we just leave it as empty
# Header should have the stamp
# std_msgs/Header header 
# # For centroid semantic landmarks, the direction of this 3D vector is bearing while the magnitude is the range
# geometry_msgs/Point[] centroid_factors
# # For cylindrical landmarks. Contains root, radius and axis. Same as original SLOAM cylinder modeling 
# ROSCylinder[] cylinder_factors
# # For cuboidal landmarks. 
# visualization_msgs/MarkerArray cuboid_factors
# # For the odometry corresponding to these measurements
# nav_msgs/Odometry odometry


# new version
# Header should have the stamp
std_msgs/Header header 
# For centroid semantic landmarks, the direction of this 3D vector is bearing while the magnitude is the range
ROSEllipsoid[] ellipsoid_factors
# For cylindrical landmarks. Contains root, radius and axis. Same as original SLOAM cylinder modeling 
ROSCylinder[] cylinder_factors
# For cuboidal landmarks. 
ROSCube[] cuboid_factors
# For the odometry corresponding to these measurements
nav_msgs/Odometry odometry