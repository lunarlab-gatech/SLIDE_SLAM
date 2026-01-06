#! /usr/bin/env python3
from __future__ import annotations

import ast
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
import ros_numpy
import numpy as np
from utils import transform_publish_pc, make_fields, threshold_by_range
from cuboid_utils_indoor import fit_cuboid_indoor, cuboid_detection_indoor, generate_publish_instance_cloud_indoor, cluster_indoor, publish_cuboid_and_range_bearing_measurements_final
from object_tracker_utils import track_objects_indoor, publish_markers
from object_tracker import ObjectTrack
from nav_msgs.msg import Odometry
import yaml


class ProcessCloudNode:
    def __init__(self, node_name):

        # Open class config file & load 
        self.cls_config_path = rospy.get_param('~cls_config_path')
        with open(self.cls_config_path, 'r') as file:
            self.cls_data_all = yaml.load(file, Loader=yaml.FullLoader)

        # Create mappings from class name to class data/parameters
        self.cls: dict[str, int] = {cls_name: self.cls_data_all[cls_name]["id"]
                    for cls_name in self.cls_data_all.keys()}
        self.length_cutoffs: dict = {cls_name: tuple(
            self.cls_data_all[cls_name]["length_cutoff"]) for cls_name in self.cls_data_all.keys()}
        self.height_cutoffs: dict = {cls_name: tuple(
            self.cls_data_all[cls_name]["height_cutoff"]) for cls_name in self.cls_data_all.keys()}
        self.class_color: dict = {cls_name: tuple(
            self.cls_data_all[cls_name]["color"]) for cls_name in self.cls_data_all.keys()}
        self.class_model_path: dict = {
            cls_name: self.cls_data_all[cls_name].get("mesh_model_path") for cls_name in self.cls_data_all.keys()}
        self.class_model_scale: dict = {
            cls_name: self.cls_data_all[cls_name].get("mesh_model_scale") for cls_name in self.cls_data_all.keys()}
        self.class_assignment_thresh: dict = {
            cls_name: self.cls_data_all[cls_name]["class_assignment_thresh"] for cls_name in self.cls_data_all.keys()}
        
        # These values here for debugging purposes, often left as off
        self.color_by_floors = False
        self.floor_height_thresh = {"floor_1": (
            0.0, 2.5), "floor_2": (3.0, 6.0), "floor_3": (6.0, 15.0)}
        self.floor_color = {"floor_1": (1.0, 0.0, 0.0), "floor_2": (
            0.0, 1.0, 0.0), "floor_3": (0.0, 0.0, 1.0)}
        self.epsilon_scan = 2.5   # Only for floor-wise object clustering clustering
        self.min_samples_scan = 1 # Only for floor clustering

        # Load Important Parameters
        self.robot_name = rospy.get_param("~robot_name")
        self.camera_frame_wrt_odom_frame_T: np.ndarray = np.array(ast.literal_eval(rospy.get_param("~camera_frame_wrt_odom_frame_T")), dtype=float)
        self.camera_frame_wrt_odom_frame_R_quat: np.ndarray = \
            np.array(ast.literal_eval(rospy.get_param("~camera_frame_wrt_odom_frame_R_quat")), dtype=float)
        param_name_prefix = f"/{self.robot_name}/{node_name}/"

        self.confidence_threshold = rospy.get_param(param_name_prefix+"confidence_threshold")
        self.valid_range_threshold: float = rospy.get_param(param_name_prefix+"valid_range_threshold", default=40.0)
        self.expected_segmentation_rate = rospy.get_param(param_name_prefix+"expected_segmentation_frequency", default=2.0)
        time_to_initialize_cuboid = rospy.get_param(param_name_prefix+"time_to_initialize_cuboid", default=0.75)

        depth_percentile_lower = rospy.get_param(param_name_prefix+"depth_percentile_lower", default=35)
        depth_percentile_uppper = rospy.get_param(param_name_prefix+"depth_percentile_upper", default=45)
        self.depth_percentile: tuple[int, int] = (depth_percentile_lower, depth_percentile_uppper)

        self.pc_width = rospy.get_param(param_name_prefix+"pc_width", default=1024)
        self.pc_height = rospy.get_param(param_name_prefix+"pc_height", default=64)
        self.pc_point_step = rospy.get_param(param_name_prefix+"pc_point_step")
        if self.pc_point_step != 16:
            raise NotImplementedError("Only point step of 16 is supported currently! Methods are hard-coded for this...")

        # Load other parameters
        self.desired_acc_obj_pub_rate = rospy.get_param(param_name_prefix+"desired_acc_obj_pub_rate", default=1.0)
        self.use_sim = rospy.get_param(param_name_prefix+"use_sim")
        if self.use_sim: raise NotImplementedError("Simulation mode is not implemented currently!")
        self.fit_cuboid_length_thresh: float = rospy.get_param(param_name_prefix+"fit_cuboid_dim_thresh", default=0.2)
        self.downsample_res = rospy.get_param(param_name_prefix+"downsample_res", default=-1)
        self.num_instance_point_lim = rospy.get_param(param_name_prefix+"num_instance_point_lim", default=10000)
        time_to_delete_lost_track_cuboid = rospy.get_param(param_name_prefix+"time_to_delete_lost_track_cuboid", default=30)
        self.visualize = rospy.get_param(param_name_prefix+"visualize_DBSCAN_results", default=False)

        # Calculate other useful thresholds
        self.tracker_age_thresh_lower = self.expected_segmentation_rate * time_to_initialize_cuboid
        rospy.loginfo(f"Cuboid initialization age threshold (in number of frames): {self.tracker_age_thresh_lower}")
        self.num_lost_track_times_thresh = self.expected_segmentation_rate * time_to_delete_lost_track_cuboid

        # Object tracking variables
        self.all_objects: list[np.ndarray] = [] # the x, y, length and width of each object, along with age and class id
        self.all_tracks: list[ObjectTrack] = []

        self.save_fig_idx = 0
        self.save_fig_counter = 0
        self.processed_scan_idx: int = -1
        self.prev_acc_obj_pub_time = None

        # Listen and publish transforms
        self.tf_listener2 = tf.TransformListener()
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Create reusable point cloud field definition
        self.pc_fields_ = make_fields()

        # =========== Publishers ============
        # Publishes point cloud in reference frame
        self.segmented_pc_pub = rospy.Publisher("filtered_semantic_segmentation", PointCloud2, queue_size=1) 

        # Publishes vertical cylinder markers for current object tracks
        self.cuboid_center_marker_pub = rospy.Publisher("cuboid_centers", MarkerArray, queue_size=1)

        # Publishes accumulated instance point cloud from tracks over age threshold
        self.instance_cloud_pub = rospy.Publisher("pc_instance_segmentation_accumulated", PointCloud2, queue_size=1)

        # Publish cuboids in reference frame and range_image_frame (changed to car from chair so they are cuboids in the backend)
        self.cuboid_marker_pub = rospy.Publisher("car_cuboids", MarkerArray, queue_size=5)
        self.cuboid_marker_body_pub = rospy.Publisher("car_cuboids_body", MarkerArray, queue_size=5)
        # ===================================

        # Frame IDs (for publishing TFs)
        self.range_image_frame = self.robot_name + "/odom"
        self.reference_frame = "world"
        self.undistorted_cloud_frame = self.robot_name + "/camera"

        # Print status to screen
        rospy.loginfo("Running real-world experiments...")
        time.sleep(1)

        # Subscribers
        self.segmented_pc_sub = rospy.Subscriber("sem_detection/sync_pc_odom", syncPcOdom, 
                                                 callback=self.segmented_pc_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, callback=self.odom_callback, queue_size=100)

    def segmented_pc_cb(self, seg_cloud_msg: syncPcOdom) -> None:
        """ Callback for receiving segmented point cloud with odometry """

        self.odom_from_cloud_msg: Odometry = seg_cloud_msg.odom
        self.segmented_synced_pc_cb(seg_cloud_msg.cloud)

    def odom_callback(self, msg: Odometry) -> None:
        """ Callback for odometry used to update the TF tree. """
        
        # Publish the transformation of the odom frame wrt. the reference frame (world)
        self.odom_broadcaster.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            msg.header.stamp, self.range_image_frame, self.reference_frame)
        
        # Publish the transformation of the camera frame wrt. the odom frame
        self.odom_broadcaster.sendTransform(tuple(self.camera_frame_wrt_odom_frame_T), 
                                            tuple(self.camera_frame_wrt_odom_frame_R_quat),
            msg.header.stamp, self.undistorted_cloud_frame, self.range_image_frame)

    def segmented_synced_pc_cb(self, segmented_cloud_msg: PointCloud2):
        
        # Increment processed scan index
        self.processed_scan_idx += 1

        # Keep track of current timestamp
        current_raw_timestamp: rospy.Time = segmented_cloud_msg.header.stamp
        
        # create pc from the undistorted_cloud
        segmented_pc: np.ndarray = ros_numpy.numpify(segmented_cloud_msg)

        # Remove NaN values from point cloud and extract x,y,z coordinates
        x_coords = np.nan_to_num(segmented_pc['x'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        y_coords = np.nan_to_num(segmented_pc['y'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        z_coords = np.nan_to_num(segmented_pc['z'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        
        # Extract class ids, segment ids and confidence values
        cls_ids = (segmented_pc['intensity']).flatten()
        seg_ids = (segmented_pc['id']).flatten()
        confidences = (segmented_pc['confidence']).flatten()

        # Stack all aboves values into single array
        pc_xyzi_id_conf: np.ndarray = np.zeros((x_coords.shape[0], 6))
        pc_xyzi_id_conf[:, 0] = x_coords
        pc_xyzi_id_conf[:, 1] = y_coords
        pc_xyzi_id_conf[:, 2] = z_coords
        pc_xyzi_id_conf[:, 3] = cls_ids
        pc_xyzi_id_conf[:, 4] = seg_ids
        pc_xyzi_id_conf[:, 5] = confidences

        # Threshold by range. Remove points that are farther away than self.valid_range_threshold
        valid_indices: np.ndarray[bool] = threshold_by_range(self.valid_range_threshold, pc_xyzi_id_conf)
        if np.sum(valid_indices) == 0:
            rospy.logwarn("No valid points found after range thresholding. Skipping this scan!")
            return
        pc_xyzi_id_conf_thresholded: np.ndarray = pc_xyzi_id_conf[valid_indices, :]

        # Transform point cloud to reference frame (and range image frame) and publish points in reference frame
        points_world_xyzi_id_conf_depth, points_body_xyzi_id_conf = \
            transform_publish_pc(self, current_raw_timestamp, pc_xyzi_id_conf_thresholded)

        # If the above failed, print error messages and skip this scan
        if points_world_xyzi_id_conf_depth is None or points_body_xyzi_id_conf is None:
            rospy.logwarn("Failed to transform point cloud to world frame. Skipping this scan!!!")
            rospy.logwarn("This may be caused due to transform_publish_pc function not performing correctly. Check the above warning messages.")
            rospy.logwarn("If you are replying bags, try setting /use_sim_time to true and add --clock flag to rosbag play")
            rospy.logwarn("It may also be caused by excessive CPU load, play bag with slower rate")
            return

        # For each object class
        for cur_object_class in self.cls.keys():

            # Skip "background" if present
            if cur_object_class == "background":
                continue
            
            # Get current class id and extract points belonging to that class
            cur_cls_id: int = self.cls[cur_object_class]
            pc_world_cur_class = points_world_xyzi_id_conf_depth[points_world_xyzi_id_conf_depth[:, 3] == cur_cls_id, :]

            # If there are no points for this class, skip
            if pc_world_cur_class.shape[0] == 0:
                continue

            # Fit cuboids to the semantic instances to start the tracking process
            xcs, ycs, lengths, widths, raw_points = fit_cuboid_indoor(self.fit_cuboid_length_thresh, pc_world_cur_class, 
                                                                      self.depth_percentile, self.confidence_threshold)

            # Merge into Nx4 single array with rows as cuboids, and columns as x, y, length, width
            cur_objects: np.ndarray = np.transpose(np.asarray([xcs, ycs, lengths, widths]))

            # Track objects over time
            if cur_objects.shape[0] != 0:
                self.all_objects, self.all_tracks = track_objects_indoor(self, cur_cls_id, cur_object_class,
                                cur_objects, self.all_objects, self.all_tracks, self.processed_scan_idx, copy.deepcopy(raw_points), 
                                self.downsample_res, self.num_instance_point_lim)
                
                # Publish markers for all tracks
                publish_markers(self, self.all_tracks, cur_cls_name=cur_object_class, age_threshold=self.tracker_age_thresh_lower, 
                                frame_id=self.reference_frame)
            else:
                rospy.logwarn_throttle( 7, "No valid objects found for object fitting")

            # Delete old tracks to bound computation & ensure cuboid measurements are local (do not incorporate too much odom noise).
            idx_to_delete: list[int] = []
            for idx, track in enumerate(self.all_tracks):
                num_lost_track_times: int = self.processed_scan_idx - track.last_update_scan_idx
                if num_lost_track_times > self.num_lost_track_times_thresh:
                    idx_to_delete.append(idx)

            # Delete in descending order so that it does not get messed up
            for idx in sorted(idx_to_delete, reverse=True):
                del self.all_tracks[idx]
                del self.all_objects[idx]

        # Publishing segmented, tracked, and accumulated instance point cloud
        extracted_instances_xyzl, instance_global_ids = generate_publish_instance_cloud_indoor(self, current_raw_timestamp)

        # If there are tracks over the age threshold, perform cuboid detection
        if extracted_instances_xyzl is not None:
            cuboids, cuboid_clus_centroids = cuboid_detection_indoor(self, extracted_instances_xyzl, instance_global_ids)

            # If at least one cuboid detected, publish cuboids and range-bearing measurements
            if len(cuboids) > 0:
                
                # Skip publishing if not enough time has elapsed since last publish
                if (self.prev_acc_obj_pub_time is not None) and ((rospy.Time.now() - self.prev_acc_obj_pub_time).to_sec() < 1.0/self.desired_acc_obj_pub_rate):
                    rospy.logwarn_throttle(5, "Time elapsed since last depth rgb callback is: " + str((rospy.Time.now() - self.prev_acc_obj_pub_time).to_sec()) + " seconds. Skipping current depth image to get desired rate of " + str(self.desired_acc_obj_pub_rate) + " Hz")
                else:
                    # Update previous publish time
                    self.prev_acc_obj_pub_time = rospy.Time.now()

                    # color_by_floors option for debugging purposes
                    if self.color_by_floors == True:
                        cuboid_clus_labels = cluster_indoor(np.array(cuboid_clus_centroids), self.epsilon_scan, 
                                                            self.min_samples_scan, use_2d=False)
                    else:
                        cuboid_clus_labels = None

                    # Publish cuboids and range-bearing measurements
                    publish_cuboid_and_range_bearing_measurements_final(self, copy.deepcopy(cuboids), 
                                cuboid_clus_labels, current_raw_timestamp)
                    
if __name__ == '__main__':

    node_name = rospy.get_param(
        '/process_cloud_node_name', 'process_cloud_node')

    rospy.init_node(node_name)

    process_cloud_node = ProcessCloudNode(node_name)

    while not rospy.is_shutdown():
        print("node started!")
        rospy.spin()
    print("node killed!")
