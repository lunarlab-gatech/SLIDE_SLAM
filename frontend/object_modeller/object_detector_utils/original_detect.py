#!/usr/bin/env python

import rospy
import rospkg
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import message_filters
import numpy as np
import torch
import matplotlib.pyplot as plt
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
import cv2
from sem_detection_msgs.msg import syncPcOdom

bridge = CvBridge()

# observation 1: in the bag file, color and depth images can go out of sync
# observation 2: might be able to use aligned depth?



class sem_detection:
    def __init__(self) -> None:
        # Initialize the ROS node
        rospy.init_node('sem_detection')

        self.prev_time = rospy.Time.now()
        rospack = rospkg.RosPack()
        self.model_path = rospack.get_path('rgb_sem_segmentation') + '/models/yolov8m-seg.pt'
        self.yolo = YOLO(self.model_path)

        # Subscribe to the rgb image
        # ros params
        self.sim = rospy.get_param('~sim', False)
        self.color_fx = rospy.get_param('~fx', 603.7166748046875)
        self.color_fy = rospy.get_param('~fy', 603.9064331054688)
        self.color_cx = rospy.get_param('~cx', 314.62518310546875)
        self.color_cy = rospy.get_param('~cy', 244.9166717529297)
        if self.sim:
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            self.depth_scale = 1
        else:
            self.depth_scale = 1 / rospy.get_param('~k_depth_scaling_factor', 1000)
        print("Depth scale: ", self.depth_scale)
        print("fx: ", self.color_fx)
        print("fy: ", self.color_fy)
        print("cx: ", self.color_cx)
        print("cy: ", self.color_cy)
        # self.color_fx = 603.7166748046875
        # self.color_cx = 314.62518310546875
        # self.color_fy = 603.9064331054688
        # self.color_cy = 244.9166717529297

        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw/')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw/')
        self.aligned_depth_topic = rospy.get_param('~aligned_depth_topic', '/camera/aligned_depth_to_color/image_raw')
        # self.odom_topic = rospy.get_param('~odom_topic', '/dragonfly67/quadrotor_ukf/control_odom')
        self.odom_topic = rospy.get_param('~odom_topic', '/scarab41/odom_laser')
        self.sync_odom_measurements = rospy.get_param('~sync_odom_measurements', True)
        self.sync_pc_odom_topic = rospy.get_param('~sync_pc_odom_topic', '/sem_detection/sync_pc_odom')
        self.pc_topic = rospy.get_param('~pc_topic', '/sem_detection/point_cloud')

        self.desired_rate = rospy.get_param('~desired_rate', 2.0)
        # Subscriber and publisher
        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)

        # Subscribe to the depth image
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.aligned_depth_sub = message_filters.Subscriber(self.aligned_depth_topic, Image)
        self.odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)

        self.pc_pub_ = rospy.Publisher(self.pc_topic, PointCloud2, queue_size=1)
        self.synced_pc_pub_ = rospy.Publisher(self.sync_pc_odom_topic, syncPcOdom, queue_size=1)


        # Synchronize the two image topics with a time delay of 0.1 seconds
        # ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        if (self.sync_odom_measurements):
            rospy.loginfo("syncing rgb, aligned depth and odom")
            # ApproximateTimeSynchronizer to allow for 0.01s time difference
            ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.aligned_depth_sub, self.odom_sub], 10, 0.05)
            ts.registerCallback(self.rgb_aligned_depth_odom_callback)
        else:
            ts = message_filters.TimeSynchronizer([self.rgb_sub, self.aligned_depth_sub], 5)
            ts.registerCallback(self.rgb_aligned_depth_callback)

        # ts.registerCallback(self.depth_rgb_callback)

        # Get the camera intrinsics from the ROS parameter server
        # fx = rospy.get_param("/camera/depth/camera_info/K[0]")
        # fy = rospy.get_param("/camera/depth/camera_info/K[4]")
        # cx = rospy.get_param("/camera/depth/camera_info/K[2]")
        # cy = rospy.get_param("/camera/depth/camera_info/K[5]")

        # self.color_fx = 603.7166748046875
        # self.color_cx = 314.62518310546875
        # self.color_fy = 603.9064331054688
        # self.color_cy = 244.9166717529297

        # self.depth_scale = 0.001

        self.pc_fields_ = self.make_fields()

        # self.depth_fx = 381.5846862792969
        # self.depth_cx = 323.48199462890625
        # self.depth_fy = 381.5846862792969
        # self.depth_cy = 238.85711669921875

        # self.depth_to_color = np.array([[0.9999183416366577, -0.004022588022053242, -0.012131188064813614, 0.014722058549523354],
        #                                 [0.004131767433136702, 0.9999510645866394, 0.008988325484097004, 0.00022420687309931964],
        #                                 [0.012094438076019287, -0.009037714451551437, 0.9998860359191895, 6.471802771557122e-05],
        #                                 [0.0, 0.0, 0.0, 1.0]])
        # self.color_to_depth = np.linalg.inv(self.depth_to_color)

        # self.cls = {"chair": 1, "suitcase": 2, "dining table": 3} # all others: 4
        # self.cls = {"chair": 1}
        self.cls = {"chair": 1, "dining table": 2, "tv": 3}


    def rgb_aligned_depth_odom_callback(self, rgb, aligned_depth, odom):
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            print("time elapsed since last depth rgb callback is: ", (rospy.Time.now() - self.prev_time).to_sec())
            print("skipping current depth image to get desired rate of ", self.desired_rate)
            return
        else:
            self.prev_time = rospy.Time.now()

        rospy.loginfo("depth rgb odom callback")
        try:
            # Convert ROS image message to OpenCV image
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8") 
            depth_img = bridge.imgmsg_to_cv2(aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(e)
            return
        
        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        detections = self.yolo.predict(color_img,  show=False)

        # 2. open img_size * 2 array save class and id
        print("color shape:", color_img.shape)
        # pc_pos = np.zeros([color_img.shape[0], color_img.shape[1], 3])
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
                # print(detection.masks.masks)     # masks, (N, H, W)
                # print(detection.masks.segments)  # bounding coordinates of masks, List[segment] * N
                # print(detection.boxes.conf)   # confidence score, (N, 1)
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()
                    
                    kernel = np.ones((20,20),np.uint8)
                    # cur_mask = cv2.erode(cur_mask,kernel,iterations = 1)

                    mask_pos = np.where(cur_mask != 0)
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i+1
                    conf[mask_pos] = float(detection.boxes.conf[i])

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        d = depth_img.flatten()
        # unique, counts = np.unique(d, return_counts=True)
        # print(dict(zip(unique, counts)))
        # print("depth raw:" , d)
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
        # print(z_pt)
        # 5. Stack labels, id, conf
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate((points, label, id, conf), axis=2).astype(np.float32)

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
        print('hard coding seg pc frame_id to camera')
        pc_msg.width = color_img.shape[1]
        pc_msg.height = color_img.shape[0]
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        sync_pc_odom_msg.cloud = pc_msg
        sync_pc_odom_msg.odom = odom
        print("Pub synced pc odom msg")
        self.synced_pc_pub_.publish(sync_pc_odom_msg)
        self.pc_pub_.publish(pc_msg)

                

    def rgb_aligned_depth_callback(self, rgb, aligned_depth):
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            print("time elapsed since last depth rgb callback is: ", (rospy.Time.now() - self.prev_time).to_sec())
            print("skipping current depth image to get desired rate of ", self.desired_rate)
            return
        else:
            self.prev_time = rospy.Time.now()

        rospy.loginfo("depth rgb callback")
        try:
            # Convert ROS image message to OpenCV image
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8") 
            depth_img = bridge.imgmsg_to_cv2(aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(e)
            return
        
        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        detections = self.yolo.predict(color_img,  show=False)

        # 2. open img_size * 2 array save class and id
        print("color shape:", color_img.shape)
        # pc_pos = np.zeros([color_img.shape[0], color_img.shape[1], 3])
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
                # print(detection.masks.masks)     # masks, (N, H, W)
                # print(detection.masks.segments)  # bounding coordinates of masks, List[segment] * N
                # print(detection.boxes.conf)   # confidence score, (N, 1)
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()
                    
                    kernel = np.ones((20,20),np.uint8)
                    # cur_mask = cv2.erode(cur_mask,kernel,iterations = 1)

                    mask_pos = np.where(cur_mask != 0)
                    # print(mask_pos, type(mask_pos))
                    # print(cls_str, type(cls_str))
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i+1
                    conf[mask_pos] = float(detection.boxes.conf[i])
                    # print(detection.boxes.conf[i], float(detection.boxes.conf[i]))
        
        # 4. go through depth, project them to 3D
        # print(depth_img.shape)

        # check depth values
        # unique, counts = np.unique(depth_img, return_counts=True)
        # print(dict(zip(unique, counts)))

        # Convert the depth image to a 3D point cloud
        # rows, cols = depth_img.shape[:2]
        # for u in range(cols):
        #     for v in range(rows):
        #         d = depth_img[v, u]
        #         if not np.isnan(d) and not np.isinf(d):
        #             x = (u - self.depth_cx) * d / self.depth_fx
        #             y = (v - self.depth_cy) * d / self.depth_fy
        #             z = d
        #             pc_pos[v, u, :] = [x, y, z]
        #         else:
        #             pc_pos[v, u, :] = [-1, -1, -1]

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
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
        # print(x_pt.shape)
        # print(y_pt.shape)
        # print(z_pt.shape)
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)
        # print(points)
        # print("Nan")
        # print(np.where(points==np.NAN))
        # valid_points = ~np.isnan(points).any(axis=1)
        # points[~valid_points] = [(-1, -1, -1)]

        # 5. Stack labels, id, conf
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate((points, label, id, conf), axis=2).astype(np.float32)

        self.make_fields()

        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = aligned_depth.header.stamp
        pc_msg.header.frame_id = "camera"
        # print('hard coding seg pc frame_id to body')
        pc_msg.width = color_img.shape[1]
        pc_msg.height = color_img.shape[0]
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        self.pc_pub_.publish(pc_msg)
                
        # print(pc_data.shape)


    # def depth_rgb_callback(self, rgb, depth):
    #     rospy.loginfo("depth rgb callback")
    #     try:
    #         # Convert ROS image message to OpenCV image
    #         color_img = bridge.imgmsg_to_cv2(rgb, "bgr8")
    #         depth_img = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
    #     except Exception as e:
    #         rospy.logerr(e)
    #         return


    #     # Perform instance segmentation using YOLOv8
    #     detections = self.yolo.predict(color_img,  show=True)

    #     # Convert detections to a list of dictionaries
    #     output = []
    #     print("type n shape:", type(detections))
    #     print(len(detections))
    #     # print(type(detections[0]))
    #     print(len(detections[0]))
        
    #     if len(detections[0]) != 0: 
    #         for detection in detections:
    #             # print(detection)
    #             # segmentation
    #             print(detection.masks.masks)     # masks, (N, H, W)
    #             print(detection.masks.segments)  # bounding coordinates of masks, List[segment] * N
    #             print(detection.boxes.conf)   # confidence score, (N, 1)
    #             print(detection.boxes.cls)    # cls, (N, 1)
    


    #     # Convert the color image coordinates to normalized camera coordinates
    #     x_color = np.arange(color_img.shape[1])
    #     y_color = np.arange(color_img.shape[0])
    #     xx, yy = np.meshgrid(x_color, y_color)
    #     X_color = (xx - self.color_cx) / self.color_fx
    #     Y_color = (yy - self.color_cy) / self.color_fy
    #     Z_color = np.ones(X_color.shape)

    #     # Convert the color camera coordinates to depth camera coordinates
    #     X_depth, Y_depth, Z_depth, _ = self.color_to_depth @ np.vstack((X_color.flatten(),
    #                                                                     Y_color.flatten(),
    #                                                                     Z_color.flatten(),
    #                                                                     np.ones_like(X_color.flatten())))

    #     # Convert the depth camera coordinates to depth image coordinates
    #     x_depth = (X_depth * self.depth_fx / Z_depth + self.depth_cx).reshape(color_img.shape[:2])
    #     y_depth = (Y_depth * self.depth_fy / Z_depth + self.depth_cy).reshape(color_img.shape[:2])

    # # TODO: 1. Sync depth img and rgb image
    # #       2. According to tf, label points in depth img
    # #       3. Project depth to point cloud

    def get_cls_label(self, cls_str):
        if cls_str in self.cls.keys():
            return self.cls[cls_str]
        else:
            return 0#4

    def run(self):
        # Spin the ROS node
        rospy.loginfo("Semantic detection node init.")
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


# depth to color extrinsics.
# rotation: [0.9999183416366577, -0.004022588022053242, -0.012131188064813614, 0.004131767433136702, 0.9999510645866394, 0.008988325484097004, 0.012094438076019287, -0.009037714451551437, 0.9998860359191895]
# translation: [0.014722058549523354, 0.00022420687309931964, 6.471802771557122e-05]

# depth intrinsics:
# 381.5846862792969, 0.0, cx = 323.48199462890625, 0.0, fy=381.5846862792969, cy=238.85711669921875, 0.0, 0.0, 1.0
# fx = 381

# color intrinsics:
# K: [603.7166748046875, 0.0, 314.62518310546875, 0.0, 603.9064331054688, 244.9166717529297, 0.0, 0.0, 1.0]

# aligned depth:
# K: 603.7166748046875, 0.0, 314.62518310546875, 0.0, 603.9064331054688, 244.9166717529297, 0.0, 0.0, 1.0
