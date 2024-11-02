#! /usr/bin/env python3

import time
from matplotlib import use
import tf
from sensor_msgs.msg import PointCloud2
import copy
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
import open3d as o3d
from sloam_msgs.msg import syncPcOdom
import rospy
import rospkg
import ros_numpy
import numpy as np
from utils import transform_publish_pc, send_tfs, make_fields, threshold_by_range
from cuboid_utils_indoor import fit_cuboid_indoor, cuboid_detection_indoor, generate_publish_instance_cloud_indoor, cluster_indoor, publish_cuboid_and_range_bearing_measurements_final
from object_tracker_utils import track_objects_indoor, publish_markers
from nav_msgs.msg import Odometry
import sys
import yaml


class ProcessCloudNode:
    def __init__(self, node_name):

        # DO NOT CHNAGE THIS TO ANYTHING OTHER THAN /ODOM. This is automatically remapped in the launch file
        self.odom_topic = "/odom"

        rospack = rospkg.RosPack()

        # detect_no_seg: if true, meaning only object detection is done, no instance segmentation. This scenario uses YOLO-WORLD
        self.detect_no_seg = rospy.get_param('~detect_no_seg', False)

        if self.detect_no_seg:
            self.cls_config_path = rospack.get_path(
                'scan2shape_launch') + '/config/process_cloud_node_indoor_open_vocab_cls_info.yaml'
        else:
            self.cls_config_path = rospack.get_path(
                'scan2shape_launch') + '/config/process_cloud_node_indoor_cls_info.yaml'

        with open(self.cls_config_path, 'r') as file:
            self.cls_data_all = yaml.load(file, Loader=yaml.FullLoader)

        self.cls = {cls_name: self.cls_data_all[cls_name]["id"]
                    for cls_name in self.cls_data_all.keys()}

        self.length_cutoffs = {cls_name: tuple(
            self.cls_data_all[cls_name]["length_cutoff"]) for cls_name in self.cls_data_all.keys()}
        self.height_cutoffs = {cls_name: tuple(
            self.cls_data_all[cls_name]["height_cutoff"]) for cls_name in self.cls_data_all.keys()}

        self.class_color = {cls_name: tuple(
            self.cls_data_all[cls_name]["color"]) for cls_name in self.cls_data_all.keys()}

        self.class_model_path = {
            cls_name: self.cls_data_all[cls_name]["mesh_model_path"] for cls_name in self.cls_data_all.keys()}
        self.class_model_scale = {
            cls_name: self.cls_data_all[cls_name]["mesh_model_scale"] for cls_name in self.cls_data_all.keys()}

        self.class_assignment_thresh = {
            cls_name: self.cls_data_all[cls_name]["class_assignment_thresh"] for cls_name in self.cls_data_all.keys()}
        
        robot_name = rospy.get_param("/robot_name", default="robot0")
        param_name_prefix = f"/{robot_name}/{node_name}/"

        self.color_by_floors = False  # For debugging only, leave it as False
        self.floor_height_thresh = {"floor_1": (
            0.0, 2.5), "floor_2": (3.0, 6.0), "floor_3": (6.0, 15.0)}
        self.floor_color = {"floor_1": (1.0, 0.0, 0.0), "floor_2": (
            0.0, 1.0, 0.0), "floor_3": (0.0, 0.0, 1.0)}
        # this is only for floor-wise object clustering clustering
        self.epsilon_scan = 2.5
        # this is only for floor clustering
        self.min_samples_scan = 1

        ################################## IMPORTANT PARAMS ##################################
        # TODO(ankit): See if making different confidence for different classes makes sense
        self.confidence_threshold = rospy.get_param(
            param_name_prefix+"confidence_threshold", default=0.4)
        self.desired_acc_obj_pub_rate = rospy.get_param(
            param_name_prefix+"desired_acc_obj_pub_rate", default=1.0)
        self.expected_segmentation_rate = rospy.get_param(
            param_name_prefix+"expected_segmentation_frequency", default=2.0)
        self.use_sim = rospy.get_param(
            param_name_prefix+"use_sim", default=False)
        self.visualize = rospy.get_param(
            param_name_prefix+"visualize_DBSCAN_results", default=False)
        self.valid_range_threshold = rospy.get_param(
            param_name_prefix+"valid_range_threshold", default=40.0)
        self.fit_cuboid_length_thresh = rospy.get_param(
            param_name_prefix+"fit_cuboid_dim_thresh", default=0.2)

        depth_percentile_lower = rospy.get_param(
            param_name_prefix+"depth_percentile_lower", default=35)
        depth_percentile_uppper = rospy.get_param(
            param_name_prefix+"depth_percentile_upper", default=45)
        self.depth_percentile = (
            depth_percentile_lower, depth_percentile_uppper)

        time_to_initialize_cuboid = rospy.get_param(
            param_name_prefix+"time_to_initialize_cuboid", default=0.75)
        self.tracker_age_thresh_lower = self.expected_segmentation_rate * \
            time_to_initialize_cuboid

        time_to_delete_lost_track_cuboid = rospy.get_param(
            param_name_prefix+"time_to_delete_lost_track_cuboid", default=30)
        self.num_lost_track_times_thresh = self.expected_segmentation_rate * \
            time_to_delete_lost_track_cuboid

        self.downsample_res = rospy.get_param(
            param_name_prefix+"downsample_res", default=-1)
        self.num_instance_point_lim = rospy.get_param(
            param_name_prefix+"num_instance_point_lim", default=10000)
        self.pc_width = rospy.get_param(
            param_name_prefix+"pc_width", default=1024)
        self.pc_height = rospy.get_param(
            param_name_prefix+"pc_height", default=64)
        self.pc_point_step = rospy.get_param(
            param_name_prefix+"pc_point_step", default=16)
        ################################## IMPORTANT PARAMS ENDS ##################################

        # CONTAINERS and VARIABLES
        # IMPORTANT: THESE TWO NEED TO BE UPDATED SIMULTANEOUSLY, THEIR LENGTH SHOULD MATCH!
        # the x, y, length and width of each object
        self.all_objects = []
        # the ObjectTrack instance of each object
        self.all_tracks = []
        self.save_fig_idx = 0
        self.save_fig_counter = 0
        self.processed_scan_idx = -1
        self.prev_acc_obj_pub_time = None

        self.tf_listener2 = tf.TransformListener()
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.pc_fields_ = make_fields()

        # PUBLISHERS
        self.segmented_pc_pub = rospy.Publisher(
            "filtered_semantic_segmentation", PointCloud2, queue_size=1)
        self.cuboid_center_marker_pub = rospy.Publisher(
            "cuboid_centers", MarkerArray, queue_size=1)
        # TODO(ankit): Check and remove covariance markers if not needed
        self.cuboid_center_cov_pub = rospy.Publisher(
            "cuboid_centers_covariance", MarkerArray, queue_size=1)
        self.instance_cloud_pub = rospy.Publisher(
            "pc_instance_segmentation_accumulated", PointCloud2, queue_size=1)
        # TODO(ankit): Current "chair_cuboids" topic is used for publishing all object models. Change this to a more generic name
        self.cuboid_marker_pub = rospy.Publisher(
            "chair_cuboids", MarkerArray, queue_size=5)
        self.cuboid_marker_body_pub = rospy.Publisher(
            "chair_cuboids_body", MarkerArray, queue_size=5)
        self.tree_cloud_pub = rospy.Publisher(
            "tree_cloud", PointCloud2, queue_size=1)
        self.ground_cloud_pub = rospy.Publisher(
            "ground_cloud", PointCloud2, queue_size=1)
        # TODO(ankit): Check if this is needed
        self.odom_pub = rospy.Publisher(
            "quadrotor/lidar_odom", Odometry, queue_size=100)

        # frame ids
        if self.use_sim == False:
            # range image frame
            self.range_image_frame = "body"
            self.reference_frame = "dragonfly67/odom"
            # undistorted point cloud frame
            self.undistorted_cloud_frame = "camera"
        else:
            self.range_image_frame = "body"
            self.reference_frame = "world"
            self.undistorted_cloud_frame = "camera"

        # subscriber and publisher
        if self.use_sim == False:
            rospy.loginfo("Running real-world experiments...")
            time.sleep(1)

            self.segmented_pc_sub = rospy.Subscriber(
                "sem_detection/sync_pc_odom", syncPcOdom, callback=self.segmented_pc_cb, queue_size=1)

            self.odom_sub = rospy.Subscriber(
                self.odom_topic, Odometry, callback=self.odom_callback, queue_size=100)

        else:
            rospy.logwarn("Running simulation experiments. This mode is still under development and is not fully tested. Switch the self.use_sim flag to False to run real-world experiments which work correctly.")
            time.sleep(10)
            self.segmented_pc_sub = rospy.Subscriber(
                "sem_detection/sync_pc_odom", syncPcOdom, callback=self.segmented_pc_cb, queue_size=1)
            self.odom_sub = rospy.Subscriber(
                self.odom_topic, Odometry, callback=self.odom_callback, queue_size=100)

    def sim_segmented_synced_pc_cb(self, chair_cloud_msg, odom_msg):
        self.segmented_synced_pc_cb(chair_cloud_msg, None)

    def segmented_pc_cb(self, seg_cloud_msg):
        # Now, this seg_cloud_msg has three parts,
        # Header, cloud, odom (synced with cloud)
        self.odom_from_cloud_msg = seg_cloud_msg.odom
        self.segmented_synced_pc_cb(seg_cloud_msg.cloud, None)

    def segmented_synced_pc_cb(self, segmented_cloud_msg, undistorted_cloud_msg):

        self.processed_scan_idx += 1
        current_raw_timestamp = segmented_cloud_msg.header.stamp
        # create pc from the undistorted_cloud
        segmented_pc = ros_numpy.numpify(segmented_cloud_msg)
        # remove nan values
        x_coords = np.nan_to_num(
            segmented_pc['x'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        y_coords = np.nan_to_num(
            segmented_pc['y'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        z_coords = np.nan_to_num(
            segmented_pc['z'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        # fill in the intensity values that represent the class labels
        if self.use_sim:
            intensities = (segmented_pc['intensity']).flatten()
            ids = (segmented_pc['id']).flatten()
            confs = (segmented_pc['confidence']).flatten()
        else:
            intensities = (segmented_pc['intensity']).flatten()
            ids = (segmented_pc['id']).flatten()
            confs = (segmented_pc['confidence']).flatten()

        pc_xyzi_id_conf = np.zeros((x_coords.shape[0], 6))
        pc_xyzi_id_conf[:, 0] = x_coords
        pc_xyzi_id_conf[:, 1] = y_coords
        pc_xyzi_id_conf[:, 2] = z_coords
        pc_xyzi_id_conf[:, 3] = intensities
        pc_xyzi_id_conf[:, 4] = ids
        pc_xyzi_id_conf[:, 5] = confs

        # threshold by range. Remove points that are farther away than self.valid_range_threshold
        valid_indices = threshold_by_range(
            self.valid_range_threshold, pc_xyzi_id_conf)

        if np.sum(valid_indices) == 0:
            rospy.logwarn(
                "No valid points found after range thresholding. Skipping this scan!!! Make sure the self.valid_range_threshold is set correctly.")
            return

        # apply thresholding
        pc_xyzi_id_conf_thresholded = pc_xyzi_id_conf[valid_indices, :]

        # Use odometry and transform point cloud to world frame and then publish it
        points_world_xyzi_id_conf_depth, points_body_xyzi_id_conf = transform_publish_pc(self,
                                                                                         current_raw_timestamp, pc_xyzi_id_conf_thresholded)

        if points_world_xyzi_id_conf_depth is None or points_body_xyzi_id_conf is None:
            rospy.logwarn(
                "Failed to transform point cloud to world frame. Skipping this scan!!!")
            rospy.logwarn(
                "This may be caused due to transform_publish_pc function not performing correctly. Check the above warning messages.")
            rospy.logwarn(
                "If you are replying bags, try setting /use_sim_time to true and add --clock flag to rosbag play")
            rospy.logwarn(
                "It may also be caused by excessive CPU load, play bag with slower rate")

        else:
            for cur_object_class in self.cls.keys():
                # skip background if present
                if cur_object_class == "background":
                    continue
                cur_class_label = self.cls[cur_object_class]
                pc_world_cur_class = points_world_xyzi_id_conf_depth[
                    points_world_xyzi_id_conf_depth[:, 3] == cur_class_label, :]

                # find object instances
                if pc_world_cur_class.shape[0] == 0:
                    continue

                # Fit cuboids to the semantic instances to start the tracking process
                xcs, ycs, lengths, widths, raw_points = fit_cuboid_indoor(
                    self.fit_cuboid_length_thresh, pc_world_cur_class, self.depth_percentile, self.confidence_threshold)

                # N*4 objects, first two columns are x and y coordinates, third column is length (x-range), and fourth colum is width (y-range)
                cur_objects = np.transpose(
                    np.asarray([xcs, ycs, lengths, widths]))

                if cur_objects.shape[0] != 0:
                    self.all_objects, self.all_tracks = track_objects_indoor(self, cur_class_label, cur_object_class,
                                                                             cur_objects, self.all_objects, self.all_tracks, self.processed_scan_idx, copy.deepcopy(raw_points), self.downsample_res, self.num_instance_point_lim)

                    # TODO(ankit): Maybe use age_threshold parameter here
                    publish_markers(self, self.all_tracks, cur_cls_name=cur_object_class,
                                    age_threshold=self.tracker_age_thresh_lower)
                else:
                    rospy.logwarn_throttle(
                        7, "No valid objects found for object fitting")

                # get rid of too old tracks to bound computation and make sure our cuboid measurements are local and do not incorporate too much odom noise
                idx_to_delete = []
                for idx, track in enumerate(self.all_tracks):
                    # if track.age > self.tracker_age_thresh_upper:
                    num_lost_track_times = self.processed_scan_idx - track.last_update_scan_idx
                    if num_lost_track_times > self.num_lost_track_times_thresh:
                        idx_to_delete.append(idx)

                # delete in descending order so that it does not get messed up
                for idx in sorted(idx_to_delete, reverse=True):
                    del self.all_tracks[idx]
                    del self.all_objects[idx]

            # Publishing segmented, tracked, and accumulated instance point cloud
            extracted_instances_xyzl, instance_global_ids = generate_publish_instance_cloud_indoor(self,
                                                                                                   current_raw_timestamp)

            if extracted_instances_xyzl is not None:
                cuboids, cuboid_clus_centroids = cuboid_detection_indoor(self,
                                                                         extracted_instances_xyzl, instance_global_ids)

                if len(cuboids) > 0:

                    if (self.prev_acc_obj_pub_time is not None) and ((rospy.Time.now() - self.prev_acc_obj_pub_time).to_sec() < 1.0/self.desired_acc_obj_pub_rate):
                        rospy.logwarn_throttle(5, "Time elapsed since last depth rgb callback is: " + str((rospy.Time.now() - self.prev_acc_obj_pub_time).to_sec(
                        )) + " seconds. Skipping current depth image to get desired rate of " + str(self.desired_acc_obj_pub_rate) + " Hz")
                    else:
                        self.prev_acc_obj_pub_time = rospy.Time.now()

                        if self.color_by_floors == True:
                            cuboid_clus_labels = cluster_indoor(np.array(
                                cuboid_clus_centroids), self.epsilon_scan, self.min_samples_scan, use_2d=False)
                        else:
                            cuboid_clus_labels = None

                        publish_cuboid_and_range_bearing_measurements_final(self, copy.deepcopy(
                            cuboids), cuboid_clus_labels, current_raw_timestamp, self.odom_from_cloud_msg)

    def odom_callback(self, msg):
        send_tfs(self, msg)


if __name__ == '__main__':

    node_name = rospy.get_param(
        '/process_cloud_node_name', 'process_cloud_node')

    rospy.init_node(node_name)

    process_cloud_node = ProcessCloudNode(node_name)

    while not rospy.is_shutdown():
        print("node started!")
        rospy.spin()
    print("node killed!")
