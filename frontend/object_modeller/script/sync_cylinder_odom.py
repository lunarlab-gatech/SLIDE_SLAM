#! /usr/bin/env python3

import rospy

from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import ROSCylinderArray
from nav_msgs.msg import Odometry
# message filter
from message_filters import ApproximateTimeSynchronizer, Subscriber
import argparse

# This node syncs the measurements from the following topics:
# cylinder measurements
# odometry


class SyncMeasurementsCylinderOdom:
    def __init__(self, args):
        # "/dragonfly67/quadrotor_ukf/control_odom" "/quadrotor1/lidar_odom"
        self.odom_topic = args["odom_topic"]
        self.odom_sub = Subscriber(self.odom_topic, Odometry)
        # print in green odom topic
        rospy.logdebug_once(
            "\033[92mOdom topic: {}\033[0m".format(self.odom_topic))

        self.cylinder_sub = Subscriber(
            "cylinder_measurements", ROSCylinderArray)

        # keep a list of the past timestamps that already synced
        self.synced_timestamps = []
        self.num_timestamps_to_keep = 100

        # use ApproximateTimeSynchronizer to sync the messages
        self.sync2 = ApproximateTimeSynchronizer(
            [self.cylinder_sub, self.odom_sub], queue_size=100, slop=0.01)
        self.sync2.registerCallback(self.sync_callback2)

        # create publishers
        self.sync_meas_pub = rospy.Publisher(
            "semantic_meas_sync_odom_raw", SemanticMeasSyncOdom, queue_size=10)

    def sync_callback2(self, cylinder_msg, odom_msg):
        # create SemanticMeasSyncOdom message
        sync_msg = SemanticMeasSyncOdom()
        # fill in the header
        sync_msg.header = odom_msg.header
        # fill in the odometry
        sync_msg.odometry = odom_msg
        # fill in the cylinder factors
        sync_msg.cylinder_factors = cylinder_msg.cylinders
        # put empty cuboid factors (empty MarkerArray)
        sync_msg.cuboid_factors = []
        # put point landmarks as empty
        sync_msg.ellipsoid_factors = []
        # publish the message

        # check if the timestamp is already synced
        if odom_msg.header.stamp.to_sec() in self.synced_timestamps:
            rospy.logwarn_throttle(
                2, "\033[93mWarning: timestamp already synced, skipping this to avoid duplicate measurements\033[0m")
        else:
            self.sync_meas_pub.publish(sync_msg)
            # print in green to indicate that the message is published
            rospy.loginfo_throttle(
                3, "\033[92mSynced measurements (cylinder only!) published\033[0m")


if __name__ == "__main__":
    rospy.init_node("sync_semantic_node_cylinder_odom")

    ap = argparse.ArgumentParser()

    # add indoor argument
    ap.add_argument("-o", "--odom_topic", type=str, default="odom",
                    help="odometry topic")

    args = vars(ap.parse_args(rospy.myargv()[1:]))

    sync_mes = SyncMeasurementsCylinderOdom(args)
    while not rospy.is_shutdown():
        rospy.spin()
    print("Node Killed")
