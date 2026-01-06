#!/usr/bin/env python

import cv2
import pandas as pd
import rospy
import rospkg
from cv_bridge import CvBridge
import message_filters
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
from sloam_msgs.msg import syncPcOdom
import yaml

bridge = CvBridge()

class Semantic_Detector_From_Sem_Labels:
    def __init__(self) -> None:

        rospy.init_node('semantic_detector_from_sem_labels')
        rospack = rospkg.RosPack()
        self.prev_time = rospy.Time.now()

        # Load instrinsics and depth parameters
        self.fx = rospy.get_param("~fx", 603.7166748046875)
        self.fy = rospy.get_param("~fy", 603.9064331054688)
        self.cx = rospy.get_param("~cx", 314.62518310546875)
        self.cy = rospy.get_param("~cy", 244.9166717529297) 
        self.k_depth_scaling_factor = rospy.get_param("~k_depth_scaling_factor", 1000.0)
        self.depth_scale = 1 / self.k_depth_scaling_factor

        print("Depth scale: ", self.depth_scale)
        print("fx: ", self.fx)
        print("fy: ", self.fy)
        print("cx: ", self.cx)
        print("cy: ", self.cy)

        # Load topic and other parameters
        self.desired_rate = rospy.get_param('~desired_rate')
        self.robot_name = rospy.get_param('~robot_name')
        self.seg_topic = rospy.get_param('~seg_topic')
        self.label_color_map_path = rospy.get_param('~label_color_map_path')
        self.cls_config_path = rospy.get_param('~cls_config_path')
        self.aligned_depth_topic = rospy.get_param('~aligned_depth_topic')
        self.odom_topic = rospy.get_param("~odom_topic")
        self.sync_odom_measurements = rospy.get_param('~sync_odom_measurements')
        self.sync_pc_odom_topic = rospy.get_param('~sync_pc_odom_topic')
        self.pc_topic = rospy.get_param('~pc_topic')

        # Set up subscribers
        self.seg_sub = message_filters.Subscriber(self.seg_topic, Image)
        self.aligned_depth_sub = message_filters.Subscriber(self.aligned_depth_topic, Image)
        self.odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)

        # Set up publishers
        self.pc_pub_ = rospy.Publisher(self.pc_topic, PointCloud2, queue_size=1)
        self.synced_pc_pub_ = rospy.Publisher(self.sync_pc_odom_topic, syncPcOdom, queue_size=1)

        # Synchronize the two image topics with a time delay of 0.1 seconds
        if (self.sync_odom_measurements):
            rospy.loginfo("Syncing segmentation, aligned depth and odom")
            
            # ApproximateTimeSynchronizer to allow for 0.01s time difference
            ts = message_filters.ApproximateTimeSynchronizer([self.seg_sub, self.aligned_depth_sub, self.odom_sub], 10, 0.05)
            ts.registerCallback(self.seg_aligned_depth_odom_callback)
        else:
            raise NotImplementedError("Only synced odom measurements is implemented currently")
        
        # Create the dictionary mapping RGB input arrays to labels
        df = pd.read_csv(self.label_color_map_path)
        self.rgb_to_seg_label: dict[tuple[int, int, int], str] = {(row['R'], row['G'], row['B']): row['Label'] for _, row in df.iterrows()}
        self.rgb_to_seg_id: dict[tuple[int, int, int], int] = {(row['R'], row['G'], row['B']): row['SegmentationID'] for _, row in df.iterrows()}

        # Make reusable point cloud fields for PointCloud2 messages
        self.pc_fields_ = self.make_fields()

        # Load classes we're interested in with their class IDs
        with open(self.cls_config_path, 'r') as file:
            self.cls_data_all: dict = yaml.load(file, Loader=yaml.FullLoader)
        self.cls_str_to_cls_id: dict[str, int] = {}
        for key, value in self.cls_data_all.items():
            self.cls_str_to_cls_id[key] = value["id"]
        self.not_desired_cls = []        

    def run(self):
        """ Runs this node. """
        rospy.loginfo("Semantic detection node init.")
        rospy.spin()

    def seg_aligned_depth_odom_callback(self, seg: Image, aligned_depth: Image, odom: Odometry):
        """ Callback function for synchronized RGB-D and odom messages. """

        # Skip current frame if it is too soon
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            rospy.loginfo_throttle(3, f"Time elapsed since last depth rgb callback is: {(rospy.Time.now() - self.prev_time).to_sec()}")
            rospy.loginfo_throttle(3, f"Skipping current depth image to get desired rate of {self.desired_rate} Hz")
            return
        else:
            self.prev_time = rospy.Time.now()

        try:
            # Make sure the depth image follows our data assumptions
            assert aligned_depth.encoding == "32FC1", f"Expected depth image encoding to be 32FC1, but got {aligned_depth.encoding}"

            # Load images from ROS messages
            seg_img_rgb = np.array(bridge.imgmsg_to_cv2(seg, "rgb8"))
            depth_img = np.array(bridge.imgmsg_to_cv2(aligned_depth, desired_encoding="passthrough"))

        except Exception as e:
            rospy.logerr(e)
            return

        # Convert RGB segmentation image to class labels
        cls_id_array = np.zeros([seg_img_rgb.shape[0], seg_img_rgb.shape[1]], dtype=int)
        seg_id_array = np.zeros([seg_img_rgb.shape[0], seg_img_rgb.shape[1]], dtype=int)
        conf_array = np.zeros([seg_img_rgb.shape[0], seg_img_rgb.shape[1]], dtype=float)
        
        for i, row in enumerate(seg_img_rgb):
            for j, val in enumerate(row):
                rgb_tuple = (int(val[0]), int(val[1]), int(val[2]))
                seg_label = self.rgb_to_seg_label.get(rgb_tuple, "None")
                seg_id = self.rgb_to_seg_id.get(rgb_tuple, -1)

                # Convert class str to class id
                def cls_str_to_cls_id(cls_str):
                    desired_cls_strs: list[str] = self.cls_str_to_cls_id.keys()
                    for desired_cls_str in desired_cls_strs:
                        if desired_cls_str in cls_str:
                            return self.cls_str_to_cls_id[desired_cls_str]
                    else:
                        if cls_str not in self.not_desired_cls:
                            rospy.logwarn(f"Class label {cls_str} not in desired classes, setting class id to 0")
                            self.not_desired_cls.append(cls_str)
                        return 0
                cls_id_array[i, j] = cls_str_to_cls_id(seg_label)

                # Save seg id and confidence
                seg_id_array[i, j] = seg_id
                conf_array[i, j] = 1.0

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        
        # Scale the depth image by the depth scaling factor
        d = depth_img.flatten()
        d = d * self.depth_scale

        # Back-project to 3D points
        x = (u.flatten() - self.cx) * d / self.fx
        y = (v.flatten() - self.cy) * d / self.fy
        z = d

        # Reshape back into the original image shape
        x_pt = x.reshape(-1, depth_img.shape[1])
        y_pt = y.reshape(-1, depth_img.shape[1])
        z_pt = z.reshape(-1, depth_img.shape[1])

        # Stack into a point cloud with shape (H, W, 3)
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)

        # Add the manual class ids, seg ids and confidence to the point cloud
        cls_id_array = cls_id_array[..., None]
        seg_id_array = seg_id_array[..., None]
        conf_array = conf_array[..., None]
        pc_data = np.concatenate((points, cls_id_array, seg_id_array, conf_array), axis=2).astype(np.float32)

        # Create synced point cloud odom message and set header
        sync_pc_odom_msg = syncPcOdom()
        sync_pc_odom_msg.header = Header()
        sync_pc_odom_msg.header.stamp = odom.header.stamp # PC assumed to have same timestamp as odom
        sync_pc_odom_msg.header.frame_id = self.robot_name + "/camera"

        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = Header()
        pc_msg.header.stamp = odom.header.stamp
        pc_msg.header.frame_id = self.robot_name + "/camera" 
        pc_msg.width = seg_img_rgb.shape[1]
        pc_msg.height = seg_img_rgb.shape[0]
        pc_msg.point_step = 24 # Hardcoding for now, 6 fields * 4 bytes each
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()

        # Add new point cloud and odom to synced message and publish both messages
        sync_pc_odom_msg.cloud = pc_msg
        sync_pc_odom_msg.odom = odom
        self.synced_pc_pub_.publish(sync_pc_odom_msg)
        self.pc_pub_.publish(pc_msg)
        rospy.loginfo_throttle(3, "Published synced point cloud odom msg")

    def make_fields(self):
        """ Creates point cloud fields to be reused for each PointCloud2 message. """

        def make_point_field(name: str, count: int, offset: int, datatype) -> PointField:
            field = PointField()
            field.name = name
            field.count = count
            field.offset = offset
            field.datatype = datatype
            return field
        
        fields: list[PointField] = []
        fields.append(make_point_field('x', 1, 0, PointField.FLOAT32))
        fields.append(make_point_field('y', 1, 4, PointField.FLOAT32))
        fields.append(make_point_field('z', 1, 8, PointField.FLOAT32))
        fields.append(make_point_field('intensity', 1, 12, PointField.FLOAT32)) # TODO: Shouldn't this be cls_id?
        fields.append(make_point_field('id', 1, 16, PointField.FLOAT32))
        fields.append(make_point_field('confidence', 1, 20, PointField.FLOAT32))
        return fields

if __name__ == '__main__':
    node = Semantic_Detector_From_Sem_Labels()
    node.run()
