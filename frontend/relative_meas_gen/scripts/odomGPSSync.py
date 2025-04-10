#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import numpy as np
import sensor_msgs.msg
import nav_msgs.msg
import message_filters
from sloam_msgs.msg import OdometryGPSBundle

class OdomGPSSync:
    def __init__(self):
        # Get parameters
        self.odom_topic = rospy.get_param("odom_topic", default="/Odometry")
        self.gps_topic = rospy.get_param("gps_topic", default="/gps")
        self.pub_topic = rospy.get_param("pub_topic", default="/odom_gps_sync")
        self.slop = rospy.get_param("slop", default=0.1)

        # Synchronize the odometry and gps
        sub_odom = message_filters.Subscriber(self.odom_topic, nav_msgs.msg.Odometry)
        sub_gps = message_filters.Subscriber(self.gps_topic, sensor_msgs.msg.NavSatFix)
        self.ts = message_filters.ApproximateTimeSynchronizer([sub_odom, sub_gps], queue_size=100, slop=self.slop)
        self.ts.registerCallback(self.publish_sync)

        # Publisher
        self.pub = rospy.Publisher(self.pub_topic, OdometryGPSBundle, queue_size=10)

        # Keep track of messages sent
        self.sequence_number = 0

    def publish_sync(self, msg_odom: nav_msgs.msg.Odometry, msg_gps: sensor_msgs.msg.NavSatFix):
        # Create a new OdometryGPSBundle message
        msg = OdometryGPSBundle()

        # Fill in the header
        msg.header.seq = self.sequence_number
        msg.header.stamp = msg_gps.header.stamp # Assume odometry at same timestamp as gps
        msg.header.frame_id = ""

        # Fill in the message and publish
        msg.odom = msg_odom
        msg.gps = msg_gps
        self.pub.publish(msg)

        # Increment the sequence number
        self.sequence_number += 1


if __name__ == '__main__':
    # Initialize the node with given name
    node_name = rospy.get_param("/odom_gps_sync_node_name", default="odom_gps_sync_node_name")
    rospy.init_node(node_name)
    sync = OdomGPSSync()

    # NOTE: NODE NOT FULLY IMPLEMENTED/TESTED
    raise NotImplementedError("This node is not fully implemented or tested. Please check the code and test it before use.")

    # Run until shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))