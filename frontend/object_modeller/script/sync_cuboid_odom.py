#! /usr/bin/env python3

import argparse
import rospy
from sloam_msgs.msg import SemanticMeasSyncOdom
from sloam_msgs.msg import ROSCube
from geometry_msgs.msg import Pose
from sloam_msgs.msg import StampedRvizMarkerArray
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
import yaml

class SyncMeasurementsCuboidOdom:
    """ This node syncs cuboid measurements and odometry """

    def __init__(self, args):

        # Subscribe to the odometry topic
        self.odom_topic = args["odom_topic"]
        self.odom_sub = Subscriber(self.odom_topic, Odometry)
        rospy.loginfo_once(
            "\033[92mOdom topic: {}\033[0m".format(self.odom_topic))
        
        # Load the mapping from semantic label to class ID
        self.cls_config_path = args["cls_config_path"]
        with open(self.cls_config_path, 'r') as file:
            self.cls_data_all: dict = yaml.load(file, Loader=yaml.FullLoader)
        self.cls_str_to_cls_id: dict[str, int] = {}
        for key, value in self.cls_data_all.items():
            self.cls_str_to_cls_id[key] = value["id"]
        
        # Subscribe to the cuboid topic
        self.cuboid_sub = Subscriber("cuboid_measurements", StampedRvizMarkerArray)

        # keep a list of the past timestamps that already synced
        self.synced_timestamps = []
        self.num_timestamps_to_keep = 100

        # Use ApproximateTimeSynchronizer to sync the messages
        self.sync1 = ApproximateTimeSynchronizer([self.cuboid_sub, self.odom_sub], queue_size=200, slop=0.01)
        self.sync1.registerCallback(self.sync_callback1)

        # Create synched publisher
        self.sync_meas_pub = rospy.Publisher("semantic_meas_sync_odom_raw", SemanticMeasSyncOdom, queue_size=10)

    def sync_callback1(self, cuboid_msg, odom_msg):
        # create a delay for processing
        # create SemanticMeasSyncOdom message
        sync_msg = SemanticMeasSyncOdom()
        # fill in the header
        sync_msg.header = odom_msg.header
        # fill in the odometry
        sync_msg.odometry = odom_msg
        # fill in the cuboid factors
        sync_msg.cuboid_factors = self.rviz2cubelist(cuboid_msg.cuboid_rviz_markers)
        # publish the message
        self.sync_meas_pub.publish(sync_msg)
        # put empty cylinder factors
        sync_msg.cylinder_factors = []
        # print in green to indicate that the message is published
        rospy.loginfo_throttle(
            3, "\033[92mSynced measurements (cuboid only!) published\033[0m")
        
    def rviz2cubelist(self, rviz_cube):
        cubelist = []
        for cube in rviz_cube.markers:
            # Extract semantic label which is stored in ns field
            semantic_label_str: str = cube.ns 
            # rospy.logwarn("Cuboid semantic label: {}".format(semantic_label_str))
            # rospy.logwarn("Cuboid class ID: {}".format(self.cls_str_to_cls_id.get(semantic_label_str, -1)))
            
            # Fill in other values
            dims = [cube.scale.x, cube.scale.y, cube.scale.z]
            pose = Pose()
            pose.position = cube.pose.position
            pose.orientation = cube.pose.orientation
            cube = ROSCube()
            cube.dim = dims
            cube.semantic_label = self.cls_str_to_cls_id.get(semantic_label_str, -1)
            cube.pose = pose
            cubelist.append(cube)
        return cubelist


if __name__ == "__main__":
    rospy.init_node("sync_semantic_node_cuboid_odom")

    ap = argparse.ArgumentParser()

    # add indoor argument
    ap.add_argument("-o", "--odom_topic", type=str, default="odom",
                    help="odometry topic")
    ap.add_argument("-c", "--cls_config_path", type=str, required=True,
                    help="path to the class info yaml file")

    args = vars(ap.parse_args(rospy.myargv()[1:]))

    sync_mes = SyncMeasurementsCuboidOdom(args)
    while not rospy.is_shutdown():
        rospy.spin()
    print("Node Killed")
