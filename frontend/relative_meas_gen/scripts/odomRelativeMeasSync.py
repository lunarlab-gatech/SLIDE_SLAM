#!/usr/bin/env python

import rospy
import sensor_msgs.msg 
import nav_msgs.msg
from sloam_msgs.msg import RelativeInterRobotMeasurement, RelativeInterRobotMeasurementOdom, StampedRvizMarkerArray
import message_filters

class OdomRelativeMeasSync:
    def __init__(self):

        # Synchronize the odometry and RelativeInterRobotMeasurement
        sub_relative_meas = message_filters.Subscriber("/relative_inter_robot_meas", RelativeInterRobotMeasurement)
        sub_odom_jackal0 = message_filters.Subscriber("/robot0/odometry/local_filtered", nav_msgs.msg.Odometry)
        sub_odom_jackal1 = message_filters.Subscriber("/robot1/odometry/local_filtered", nav_msgs.msg.Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([sub_relative_meas, sub_odom_jackal0, sub_odom_jackal1], queue_size=100, slop=0.0125)
        self.ts.registerCallback(self.publish_sync)

        # Publisher
        self.pub = rospy.Publisher('/relative_inter_robot_meas_sync', RelativeInterRobotMeasurementOdom, queue_size=10)

        # Keep track of messages sent
        self.sequence_number = 0

    def publish_sync(self, msg_relative: RelativeInterRobotMeasurement, 
                     msg_jackal0: nav_msgs.msg.Odometry, msg_jackal1: nav_msgs.msg.Odometry):
        
        # Create a new RelativeInterRobotMeasurementOdom message
        msg = RelativeInterRobotMeasurementOdom()
        msg.header.seq = self.sequence_number
        msg.header.stamp = msg_relative.header.stamp # Assume odometry at same timestamp as relative measurement
        msg.header.frame_id = "jackal0/base_link"
        msg.relativePose = msg_relative.relativePose
        msg.robotIdObserver = 0
        msg.robotIdObserved = 1
        msg.odometryObserver = msg_jackal0
        msg.odometryObserved = msg_jackal1
        self.pub.publish(msg)

        # Output the difference in timestamps
        rospy.loginfo_throttle(10, "Time difference from jackal0 Odom to Relative Measurement: {}".format(msg_jackal0.header.stamp.to_sec() - msg_relative.header.stamp.to_sec()))
        rospy.loginfo_throttle(10, "Time difference from jackal1 Odom to Relative Measurement: {}".format(msg_jackal1.header.stamp.to_sec() - msg_relative.header.stamp.to_sec()))

        # Increment the sequence number
        self.sequence_number += 1

if __name__ == '__main__':
    # Initialize the node with given name
    node_name = rospy.get_param("/odom_gps_sync_node_name", default="odom_gps_sync_node_name")
    rospy.init_node(node_name)
    sync = OdomRelativeMeasSync()

    # Run until shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))