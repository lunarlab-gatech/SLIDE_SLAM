#!/usr/bin/env python3.10

import rospy
import rospkg
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
import cv2
import yaml
from sloam_msgs.msg import syncPcOdom
import sys

bridge = CvBridge()


class sem_detection:
    def __init__(self) -> None:
        # Initialize the ROS node
        rospy.init_node('sem_detection')

        rospack = rospkg.RosPack()

        
        self.model_path = rospack.get_path(
            'object_modeller') + '/models/yolov8x-worldv2.pt'
        self.cls_config_path = rospack.get_path(
            'scan2shape_launch') + '/config/process_cloud_node_indoor_open_vocab_cls_info.yaml'
        with open(self.cls_config_path, 'r') as file:
            self.cls_data_all = yaml.load(file, Loader=yaml.FullLoader)
        self.cls_data = {
            cls_name: self.cls_data_all[cls_name]["id"] for cls_name in self.cls_data_all.keys()}

        # Define custom classes
        list_of_queries = [cls_name for cls_name in self.cls_data.keys()]

        ##################################### PARAMS / CONFIG #####################################
        param_name_prefix = rospy.get_name() + '/'
        self.robot_name = rospy.get_param(param_name_prefix + 'robot_name', 'robot0')
        self.odom_topic = rospy.get_param(param_name_prefix + 'odom_topic', '/dragonfly67/quadrotor_ukf/control_odom')
        self.visualize = rospy.get_param(param_name_prefix + 'visualize', False)
        self.confidence_threshold = rospy.get_param(param_name_prefix + 'confidence_threshold', 0.4)
        self.sim = rospy.get_param(param_name_prefix + 'sim', False)
        self.color_fx = rospy.get_param(param_name_prefix + 'color_fx', 386.2926330566406)
        self.color_fy = rospy.get_param(param_name_prefix + 'color_fy', 386.2926330566406)
        self.color_cx = rospy.get_param(param_name_prefix + 'color_cx', 324.7949523925781)
        self.color_cy = rospy.get_param(param_name_prefix + 'color_cy', 242.52862548828125)
        self.k_depth_scaling_factor = rospy.get_param(param_name_prefix + 'k_depth_scaling_factor', 1000.0)
        self.desired_rate = rospy.get_param(param_name_prefix + 'desired_rate', 2.0)
        ##################################### PARAMS / CONFIG #####################################

        self.prev_time = rospy.Time.now()

        self.yolo = YOLO(self.model_path)
        self.yolo.set_classes(list_of_queries)

        if self.sim:
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            self.depth_scale = 1
        else:
            self.depth_scale = 1 / self.k_depth_scaling_factor

        print("Depth scale: ", self.depth_scale)
        print("fx: ", self.color_fx)
        print("fy: ", self.color_fy)
        print("cx: ", self.color_cx)
        print("cy: ", self.color_cy)

        # Subscriber topics names
        self.rgb_topic = self.robot_name + '/camera/color/image_raw'
        self.depth_topic = self.robot_name + '/camera/depth/image_rect_raw'
        self.aligned_depth_topic = self.robot_name + \
            '/camera/aligned_depth_to_color/image_raw'
        self.sync_odom_measurements = True

        # Publisher topics names
        self.sync_pc_odom_topic = self.robot_name + '/sem_detection/sync_pc_odom'
        self.pc_topic = self.robot_name + '/sem_detection/pointcloud'

        # Subscriber and publisher
        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)

        # Subscribe to the depth image
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.aligned_depth_sub = message_filters.Subscriber(
            self.aligned_depth_topic, Image)
        self.odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)

        self.pc_pub_ = rospy.Publisher(
            self.pc_topic, PointCloud2, queue_size=1)
        self.synced_pc_pub_ = rospy.Publisher(
            self.sync_pc_odom_topic, syncPcOdom, queue_size=1)

        # Synchronize the two image topics with a time delay of 0.1 seconds
        # ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        if (self.sync_odom_measurements):
            rospy.loginfo("Syncing rgb, aligned depth and odom")
            # ApproximateTimeSynchronizer to allow for 0.01s time difference
            ts = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_sub, self.aligned_depth_sub, self.odom_sub], 10, 0.05)
            ts.registerCallback(self.rgb_aligned_depth_odom_callback)
        else:
            ts = message_filters.TimeSynchronizer(
                [self.rgb_sub, self.aligned_depth_sub], 5)
            ts.registerCallback(self.rgb_aligned_depth_callback)

        self.pc_fields_ = self.make_fields()

    def rgb_aligned_depth_odom_callback(self, rgb, aligned_depth, odom):
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            rospy.loginfo_throttle(
                3, f"Time elapsed since last depth rgb callback is: {(rospy.Time.now() - self.prev_time).to_sec()}")
            rospy.loginfo_throttle(
                3, f"Skipping current depth image to get desired rate of {self.desired_rate}")
            return
        else:
            self.prev_time = rospy.Time.now()

        try:
            # Convert ROS image message to OpenCV image
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(
                aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(e)
            return

        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        detections = self.yolo.predict(
            color_img,  show=self.visualize, conf=self.confidence_threshold)

        # 2. open img_size * 2 array save class and id
        label = np.zeros([color_img.shape[0], color_img.shape[1]])
        id = np.zeros([color_img.shape[0], color_img.shape[1]])
        conf = np.zeros([color_img.shape[0], color_img.shape[1]])

        # 3. go though all masks, fill them in class and id
        if len(detections[0]) != 0:
            # detections is of size 1
            # detections[0] contains everything
            # masks is of shape (N obj, H, W)
            for detection in detections:
                # segmentation
                # masks, (N, H, W)
                # bounding coordinates of masks, List[segment] * N
                # confidence score, (N, 1)
                num_obj = detection.boxes.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]

                    # cur_mask is one for all pixels in the box
                    cur_mask = np.zeros(
                        (color_img.shape[0], color_img.shape[1]))
                    x1, y1, x2, y2 = detection.boxes.xyxy[i]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    cur_mask[y1:y2, x1:x2] = 1

                    mask_pos = np.where(cur_mask != 0)
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i+1
                    conf[mask_pos] = float(detection.boxes.conf[i])

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(
            np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        d = depth_img.flatten()
        d = d * self.depth_scale
        x = (u.flatten() - self.color_cx) * d / self.color_fx
        y = (v.flatten() - self.color_cy) * d / self.color_fy
        z = d
        x_pt = x.reshape(-1, depth_img.shape[1])
        y_pt = y.reshape(-1, depth_img.shape[1])
        z_pt = z.reshape(-1, depth_img.shape[1])
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)

        # 5. Stack labels, id, conf
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate(
            (points, label, id, conf), axis=2).astype(np.float32)

        self.make_fields()

        # 6. publish point cloud
        sync_pc_odom_msg = syncPcOdom()
        sync_pc_odom_msg.header = Header()
        sync_pc_odom_msg.header.stamp = odom.header.stamp
        sync_pc_odom_msg.header.frame_id = "camera"

        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = odom.header.stamp
        pc_msg.header.frame_id = "camera"
        rospy.logwarn_throttle(
            7, 'Hard coding segmented pc frame_id to \'camera\'. Change this if needed.')
        pc_msg.width = color_img.shape[1]
        pc_msg.height = color_img.shape[0]
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        sync_pc_odom_msg.cloud = pc_msg
        sync_pc_odom_msg.odom = odom
        rospy.loginfo_throttle(5, "Published synced point cloud odom message")
        self.synced_pc_pub_.publish(sync_pc_odom_msg)
        self.pc_pub_.publish(pc_msg)

    def rgb_aligned_depth_callback(self, rgb, aligned_depth):
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            rospy.loginfo_throttle(
                3, f"Time elapsed since last depth rgb callback is: {(rospy.Time.now() - self.prev_time).to_sec()}")
            rospy.loginfo_throttle(
                3, f"Skipping current depth image to get desired rate of {self.desired_rate}")
            return
        else:
            self.prev_time = rospy.Time.now()
        try:
            # Convert ROS image message to OpenCV image
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(
                aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(e)
            return

        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        detections = self.yolo.predict(
            color_img,  show=self.visualize, conf=self.confidence_threshold)

        # 2. open img_size * 2 array save class and id
        label = np.zeros([color_img.shape[0], color_img.shape[1]])
        id = np.zeros([color_img.shape[0], color_img.shape[1]])
        conf = np.zeros([color_img.shape[0], color_img.shape[1]])

        # 3. go though all masks, fill them in class and id
        if len(detections[0]) != 0:
            # detections is of size 1
            # detections[0] contains everything
            # masks is of shape (N obj, H, W)
            for detection in detections:
                # segmentation
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()

                    kernel = np.ones((20, 20), np.uint8)
                    # cur_mask = cv2.erode(cur_mask,kernel,iterations = 1)

                    mask_pos = np.where(cur_mask != 0)
                    # print(mask_pos, type(mask_pos))
                    # print(cls_str, type(cls_str))
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i+1
                    conf[mask_pos] = float(detection.boxes.conf[i])
                    # print(detection.boxes.conf[i], float(detection.boxes.conf[i]))

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(
            np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        d = depth_img.flatten()
        d = d * self.depth_scale
        x = (u.flatten() - self.color_cx) * d / self.color_fx
        y = (v.flatten() - self.color_cy) * d / self.color_fy
        z = d
        x_pt = x.reshape(-1, depth_img.shape[1])
        y_pt = y.reshape(-1, depth_img.shape[1])
        z_pt = z.reshape(-1, depth_img.shape[1])
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)

        # 5. Stack labels, id, conf
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate(
            (points, label, id, conf), axis=2).astype(np.float32)

        self.make_fields()

        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = aligned_depth.header.stamp
        pc_msg.header.frame_id = "camera"
        rospy.logwarn_throttle(
            7, 'Hard coding segmented pc frame_id to \'camera\'. Change this if needed.')
        pc_msg.width = color_img.shape[1]
        pc_msg.height = color_img.shape[0]
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        self.pc_pub_.publish(pc_msg)

    def get_cls_label(self, cls_str):
        if cls_str in self.cls_data.keys():
            return self.cls_data[cls_str]
        else:
            return 0  # 4

    def run(self):
        # Spin the ROS node
        rospy.loginfo("Object detection node init.")
        rospy.spin()

    def make_fields(self):
        # manually add fiels by Ian
        fields = []
        field = PointField()
        field.name = 'x'
        field.count = 1
        field.offset = 0
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'y'
        field.count = 1
        field.offset = 4
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'z'
        field.count = 1
        field.offset = 8
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'intensity'
        field.count = 1
        field.offset = 12
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'id'
        field.count = 1
        field.offset = 16
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'confidence'
        field.count = 1
        field.offset = 20
        field.datatype = PointField.FLOAT32
        fields.append(field)
        return fields


if __name__ == '__main__':
    node = sem_detection()
    node.run()
