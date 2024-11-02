#! /usr/bin/env python3

import rospy
from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import StampedRvizMarkerArray
from nav_msgs.msg import Odometry
# message filter
from message_filters import ApproximateTimeSynchronizer, Subscriber
# import R
from merge_synced_measurements import rviz2cubelist

import argparse

# This node syncs the measurements from the following topics:
# cuboid measurements
# odometry


class SyncMeasurementsCuboidOdom:
    def __init__(self, args):
        # "/dragonfly67/quadrotor_ukf/control_odom" "/quadrotor1/lidar_odom"
        self.odom_topic = args["odom_topic"]
        self.odom_sub = Subscriber(self.odom_topic, Odometry)
        rospy.loginfo_once(
            "\033[92mOdom topic: {}\033[0m".format(self.odom_topic))
        self.cuboid_sub = Subscriber(
            "cuboid_measurements", StampedRvizMarkerArray)
        # keep a list of the past timestamps that already synced
        self.synced_timestamps = []
        self.num_timestamps_to_keep = 100
        # use ApproximateTimeSynchronizer to sync the messages
        self.sync1 = ApproximateTimeSynchronizer(
            [self.cuboid_sub, self.odom_sub], queue_size=200, slop=0.01)
        self.sync1.registerCallback(self.sync_callback1)
        # create publishers
        self.sync_meas_pub = rospy.Publisher(
            "semantic_meas_sync_odom_raw", SemanticMeasSyncOdom, queue_size=10)

    def sync_callback1(self, cuboid_msg, odom_msg):
        # create a delay for processing
        # create SemanticMeasSyncOdom message
        sync_msg = SemanticMeasSyncOdom()
        # fill in the header
        sync_msg.header = odom_msg.header
        # fill in the odometry
        sync_msg.odometry = odom_msg
        # fill in the cuboid factors
        sync_msg.cuboid_factors = rviz2cubelist(cuboid_msg.cuboid_rviz_markers)
        # publish the message
        self.sync_meas_pub.publish(sync_msg)
        # put empty cylinder factors
        sync_msg.cylinder_factors = []
        # print in green to indicate that the message is published
        rospy.loginfo_throttle(
            3, "\033[92mSynced measurements (cuboid only!) published\033[0m")


if __name__ == "__main__":
    rospy.init_node("sync_semantic_node_cuboid_odom")

    ap = argparse.ArgumentParser()

    # add indoor argument
    ap.add_argument("-o", "--odom_topic", type=str, default="odom",
                    help="odometry topic")

    args = vars(ap.parse_args(rospy.myargv()[1:]))

    sync_mes = SyncMeasurementsCuboidOdom(args)
    while not rospy.is_shutdown():
        rospy.spin()
    print("Node Killed")
