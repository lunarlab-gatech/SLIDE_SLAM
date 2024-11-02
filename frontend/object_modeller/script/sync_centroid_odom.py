#! /usr/bin/env python3

import rospy
from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import StampedRvizMarkerArray, ROSEllipsoid
from nav_msgs.msg import Odometry
# message filter
from message_filters import ApproximateTimeSynchronizer, Subscriber
# import R
import argparse
import yaml
import rospkg

# This node syncs the measurements from the following topics:
# centroid/ellipsoids (point landmark) measurements
# odometry


class SyncMeasurementsCentroidOdom:
    def __init__(self, args):
        # "/dragonfly67/quadrotor_ukf/control_odom" "/quadrotor1/lidar_odom"
        self.odom_topic = args["odom_topic"]
        self.odom_sub = Subscriber(self.odom_topic, Odometry)
        self.centroid_sub = Subscriber(
            "chair_cuboids_stamped", StampedRvizMarkerArray)
        rospy.loginfo_once(
            "\033[92mOdom topic: {}\033[0m".format(self.odom_topic))
        # keep a list of the past timestamps that already synced
        self.synced_timestamps = []
        self.num_timestamps_to_keep = 100
        self.robot_name = "/"+rospy.get_param("/robot_name", default="robot0")
        self.process_cloud_node_name = "/" + \
            rospy.get_param("/process_cloud_node_name",
                            default="process_cloud_node")
        self.detect_no_seg = rospy.get_param(
            self.robot_name+self.process_cloud_node_name+"/detect_no_seg", default=False)
        rospack = rospkg.RosPack()

        if self.detect_no_seg == True:
            rospy.loginfo_once("Running with open vocabulary object detector")
            with open(rospack.get_path('scan2shape_launch') + "/config/process_cloud_node_indoor_open_vocab_cls_info.yaml", 'r') as file:
                self.cls_full_data = yaml.load(file, Loader=yaml.FullLoader)
        else:
            rospy.loginfo_once("Running with closed vocabulary object detector")
            with open(rospack.get_path('scan2shape_launch') + "/config/process_cloud_node_indoor_cls_info.yaml", 'r') as file:
                self.cls_full_data = yaml.load(file, Loader=yaml.FullLoader)

        self.cls = {cls_name: self.cls_full_data[cls_name]["id"]
                    for cls_name in self.cls_full_data.keys()}
        # use ApproximateTimeSynchronizer to sync the messages
        self.sync1 = ApproximateTimeSynchronizer(
            [self.centroid_sub, self.odom_sub], queue_size=400, slop=0.01)
        self.sync1.registerCallback(self.sync_callback1)
        # create publishers
        self.sync_meas_pub = rospy.Publisher(
            "semantic_meas_sync_odom_raw", SemanticMeasSyncOdom, queue_size=10)

    def sync_callback1(self, cuboid_msg, odom_msg):
        rospy.loginfo_throttle(7, "Odom and centroid messages received!")
        # create a delay for processing
        # create SemanticMeasSyncOdom message
        sync_msg = SemanticMeasSyncOdom()
        # fill in the header
        sync_msg.header = odom_msg.header
        # fill in the odometry
        sync_msg.odometry = odom_msg
        # take the centroid as the centroid factor
        # centroid_factors is geometry_msgs/Point[]
        ellipsoid_factors = []
        # cuboid factors are MarkerArray
        for cuboid in cuboid_msg.cuboid_rviz_markers.markers:
            ellipsoid_factor = ROSEllipsoid()
            #  the label is in the ns field
            semantic_label_str = cuboid.ns
            ellipsoid_factor.semantic_label = self.cls[semantic_label_str]
            # ellipsoid_factors.scale is float 3
            # ellipsoid_factors.pose is geometry_msgs/Pose
            # ellipsoid_factors.semantic_label is int
            ellipsoid_factor.pose = cuboid.pose
            # set orientation to identity
            ellipsoid_factor.pose.orientation.x = 0
            ellipsoid_factor.pose.orientation.y = 0
            ellipsoid_factor.pose.orientation.z = 0
            ellipsoid_factor.pose.orientation.w = 1
            ellipsoid_factor.scale = [
                cuboid.scale.x, cuboid.scale.y, cuboid.scale.z]
            ellipsoid_factors.append(ellipsoid_factor)
        sync_msg.ellipsoid_factors = ellipsoid_factors
        # publish the message
        self.sync_meas_pub.publish(sync_msg)
        # put empty cylinder factors
        sync_msg.cylinder_factors = []
        # put empty cuboid factors
        sync_msg.cuboid_factors = []
        # print in green to indicate that the message is published
        rospy.loginfo_throttle(
            3, "\033[92mSynced measurements (centroid only!) published\033[0m")


if __name__ == "__main__":
    rospy.init_node("sync_semantic_node_centroid_odom")

    ap = argparse.ArgumentParser()

    # add indoor argument
    ap.add_argument("-o", "--odom_topic", type=str, default="odom",
                    help="odometry topic")

    args = vars(ap.parse_args(rospy.myargv()[1:]))

    sync_mes = SyncMeasurementsCentroidOdom(args)
    while not rospy.is_shutdown():
        rospy.spin()
    print("Node Killed")
