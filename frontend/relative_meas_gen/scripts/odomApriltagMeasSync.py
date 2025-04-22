#!/usr/bin/env python

import rospy
import sensor_msgs.msg 
import nav_msgs.msg
from sloam_msgs.msg import RelativeInterRobotMeasurement, RelativeInterRobotMeasurementOdom, StampedRvizMarkerArray
import message_filters

class OdomApriltagMeasSync:
    def __init__(self):

        measurement_topic = rospy.get_param("~relative_meas_topic", default="default_relative_meas_topic")
        odom_topic_observer = rospy.get_param("~odom_topic_observer", default="default_odom_topic_observer")
        odom_topic_observed = rospy.get_param("~odom_topic_observed", default="default_odom_topic_observed")

        # Synchronize the odometry and RelativeInterRobotMeasurement
        sub_relative_meas = message_filters.Subscriber(measurement_topic, RelativeInterRobotMeasurement)
        sub_odom_observer = message_filters.Subscriber(odom_topic_observer, nav_msgs.msg.Odometry)
        sub_odom_observed = message_filters.Subscriber(odom_topic_observed, nav_msgs.msg.Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([sub_relative_meas, sub_odom_observer, sub_odom_observed], queue_size=100, slop=100.0125)
        self.ts.registerCallback(self.publish_sync)

        # Publisher
        self.pub = rospy.Publisher('/relative_inter_robot_meas_sync', RelativeInterRobotMeasurementOdom, queue_size=10)

        # Keep track of messages sent
        self.sequence_number = 0

    def publish_sync(self, msg_relative: RelativeInterRobotMeasurement, 
                     msg_observer: nav_msgs.msg.Odometry, msg_observed: nav_msgs.msg.Odometry):
        
        # Create a new RelativeInterRobotMeasurementOdom message
        msg = RelativeInterRobotMeasurementOdom()
        msg.header.seq = self.sequence_number
        msg.header.stamp = msg_relative.header.stamp # Assume odometry at same timestamp as relative measurement
        # msg.header.frame_id = "jackal0/base_link"
        msg.relativePose = msg_relative.relativePose
        msg.robotIdObserver = 0
        msg.robotIdObserved = 1
        msg.odometryObserver = msg_observer
        msg.odometryObserved = msg_observed
        self.pub.publish(msg)

        # Output the difference in timestamps
        rospy.loginfo_throttle(10, "Time difference from observer Odom to Relative Measurement: {}".format(msg_observer.header.stamp.to_sec() - msg_relative.header.stamp.to_sec()))
        rospy.loginfo_throttle(10, "Time difference from observed Odom to Relative Measurement: {}".format(msg_observed.header.stamp.to_sec() - msg_relative.header.stamp.to_sec()))

        # Increment the sequence number
        self.sequence_number += 1

if __name__ == '__main__':
    # Initialize the node with given name
    node_name = rospy.get_param("/odom_apriltag_meas_sync_node_name", default="odom_apriltag_meas_sync_node_name")
    rospy.init_node(node_name)
    sync = OdomApriltagMeasSync()

    # Run until shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))