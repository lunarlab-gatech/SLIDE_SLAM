#! /usr/bin/env python3

import time
# from matplotlib import use
import tf
# from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
import message_filters
import open3d as o3d
import rospy
import rospkg
import yaml
import ros_numpy
import numpy as np
from utils_outdoor import cluster, publish_cylinder_cloud, publish_ground_cloud, transform_publish_pc, send_tfs, make_fields, threshold_by_range, calc_dist_to_ground
from cuboid_utils_outdoor import fit_cuboid, publish_cuboid_markers, cuboid_detection, generate_publish_instance_cloud
from object_tracker_utils import publish_markers, track_objects_final
from nav_msgs.msg import Odometry
import sys


class ProcessCloudNode:
    def __init__(self, node_name):

        ################################## IMPORTANT PARAMS ##################################
        class_info_yaml_path = rospkg.RosPack().get_path('scan2shape_launch') + \
            "/config/process_cloud_node_outdoor_class_info.yaml"
        with open(class_info_yaml_path, 'r') as file:
            self.cls_data_all = yaml.load(file, Loader=yaml.FullLoader)

        # Creating dictionaries for easy access to class data
        self.cls_label_to_name = {
            self.cls_data_all[cls_name]['id']: cls_name for cls_name in self.cls_data_all.keys()}
        self.cls_name_to_label = {
            cls_name: self.cls_data_all[cls_name]['id'] for cls_name in self.cls_data_all.keys()}
        self.cls_labels_for_cuboid = [self.cls_data_all[cls_name]['id'] for cls_name in self.cls_data_all.keys(
        ) if self.cls_data_all[cls_name]['model'] == "cuboid"]
        self.cls_labels_for_cylinder = [self.cls_data_all[cls_name]['id'] for cls_name in self.cls_data_all.keys(
        ) if self.cls_data_all[cls_name]['model'] == "cylinder"]

        self.cuboid_length_cutoff_per_cls_label = {cls_label: tuple(
            self.cls_data_all[self.cls_label_to_name[cls_label]]['length_cutoff']) for cls_label in self.cls_labels_for_cuboid}
        self.cuboid_width_cutoff_per_cls_label = {cls_label: tuple(
            self.cls_data_all[self.cls_label_to_name[cls_label]]['width_cutoff']) for cls_label in self.cls_labels_for_cuboid}
        self.cuboid_height_cutoff_per_cls_label = {cls_label: tuple(
            self.cls_data_all[self.cls_label_to_name[cls_label]]['height_cutoff']) for cls_label in self.cls_labels_for_cuboid}
        
        self.clustering_params_per_cls_label = {cls_label: tuple(
            self.cls_data_all[self.cls_label_to_name[cls_label]]['clustering_params']) for cls_label in self.cls_labels_for_cuboid}

        self.cuboid_assignment_threshold_per_cls_label = {
            cls_label: self.cls_data_all[self.cls_label_to_name[cls_label]]['track_assignment_threshold'] for cls_label in self.cls_labels_for_cuboid}
        

        # Point cloud fields for creating PointCloud2 messages
        self.pc_fields_ = make_fields()

        robot_name = rospy.get_param("/robot_name", default="robot0")
        param_name_prefix = f"/{robot_name}/{node_name}/"

        # PARAMETERS
        # Consult the params file for more information on each parameter
        self.ransac_n_points = rospy.get_param(
            param_name_prefix+"ransac_n_points", default=50)
        self.ground_median_increment = rospy.get_param(
            param_name_prefix+"ground_median_increment", default=0.3)
        self.valid_range_threshold = rospy.get_param(
            param_name_prefix+"valid_range_threshold", default=40.0)
        self.use_sim = rospy.get_param(
            param_name_prefix+"use_sim", default=False)
        self.track_always_visualize = rospy.get_param(
            param_name_prefix+"track_always_visualize", default=False)
        time_to_initialize_cuboid = rospy.get_param(
            param_name_prefix+"time_to_initialize_cuboid", default=3.0)
        self.expected_segmentation_frequency = rospy.get_param(
            param_name_prefix+"expected_segmentation_frequency", default=2.0)
        # self.tracker_age_thresh_lower = self.expected_segmentation_frequency * \
        #     time_to_initialize_cuboid
        self.use_2d_1st_layer = rospy.get_param(
            param_name_prefix+"use_1st_layer_clustering", default=True)
        self.use_2d_2nd_layer = rospy.get_param(
            param_name_prefix+"use_2nd_layer_clustering", default=True)
        self.min_samples_scan_1st_layer = rospy.get_param(
            param_name_prefix+"min_samples_scan_1st_layer", default=7)
        self.epsilon_scan_1st_layer = rospy.get_param(
            param_name_prefix+"epsilon_scan_1st_layer", default=0.1)
        self.epsilon_scan_2nd_layer = rospy.get_param(
            param_name_prefix+"epsilon_scan_2nd_layer", default=0.5)
        self.min_samples_scan_2nd_layer = rospy.get_param(
            param_name_prefix+"min_samples_scan_2nd_layer", default=25)
        self.fit_cuboid_dim_thresh = rospy.get_param(
            param_name_prefix+"fit_cuboid_dim_thresh", default=0.5)
        self.downsample_res = rospy.get_param(
            param_name_prefix+"downsample_res", default=-1)
        self.num_instance_point_lim = rospy.get_param(
            param_name_prefix+"num_instance_point_lim", default=10000)
        
        self.cuboid_track_age_threshold_per_cls_label = {
            cls_label: self.cls_data_all[self.cls_label_to_name[cls_label]]['track_age_threshold'] * self.expected_segmentation_frequency for cls_label in self.cls_labels_for_cuboid}
        
        if self.track_always_visualize == True:
            time_to_delete_lost_track_cuboid = np.inf
        else:
            time_to_delete_lost_track_cuboid = rospy.get_param(
                param_name_prefix+"time_to_delete_lost_track_cuboid", default=45.0)

        self.num_lost_track_times_thresh = self.expected_segmentation_frequency * \
            time_to_delete_lost_track_cuboid

        self.estimate_facing_dir_car = rospy.get_param(
            param_name_prefix+"estimate_facing_dir_car", default=False)
        self.cluster_and_fix_cuboid_orientation = rospy.get_param(
            param_name_prefix+"cluster_and_fix_cuboid_orientation", default=True)

        # save figures of object clustering results from DBSCAN
        self.visualize = rospy.get_param(
            param_name_prefix+"visualize_DBSCAN_results", default=False)
        self.output_dir = rospy.get_param(
            param_name_prefix+"output_dir_to_save_figs", default="./")

        self.save_fig_idx = 0
        self.save_fig_counter = 0

        # input point cloud parameters
        self.pc_width = rospy.get_param(
            param_name_prefix+"pc_width", default=1024)
        self.pc_height = rospy.get_param(
            param_name_prefix+"pc_height", default=64)
        self.pc_point_step = rospy.get_param(
            param_name_prefix+"pc_point_step", default=16)

        seg_pc_namespace = "/os_node"
        odom_topic = "/Odometry"

        # CONTAINERS
        # IMPORTANT: THESE TWO NEED TO BE UPDATED SIMULTANEOUSLY, THEIR LENGTH SHOULD MATCH!
        # the x, y, length and width of each object
        self.all_objects = []
        # the ObjectTrack instance of each object
        self.all_tracks = []
        # for storing all track information with raw points for visualization everything always
        # based on whether self.track_always_visualize is True or False
        self.raw_cloud_dict = {}
        # Number of point cloud scans processed
        self.processed_scan_idx = -1

        # PUBLIHSERS
        self.tf_listener2 = tf.TransformListener()
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.segmented_pc_pub = rospy.Publisher(
            "filtered_semantic_segmentation", PointCloud2, queue_size=1)
        self.cuboid_center_marker_pub = rospy.Publisher(
            "cuboid_centers", MarkerArray, queue_size=1)
        # TODO(ankit): Check and remove covariance markers if not needed
        self.cuboid_center_cov_pub = rospy.Publisher(
            "cuboid_centers_covariance", MarkerArray, queue_size=1)
        self.instance_cloud_pub = rospy.Publisher(
            "pc_instance_segmentation_accumulated", PointCloud2, queue_size=1)
        # TODO(ankit): Currently "car_cuboids" is used for all cuboid objects. Carefully check and change this later 
        self.cuboid_marker_pub = rospy.Publisher(
            "car_cuboids", MarkerArray, queue_size=5)
        self.cuboid_marker_body_pub = rospy.Publisher(
            "car_cuboids_body", MarkerArray, queue_size=5)
        self.cylinder_cloud_pub = rospy.Publisher(
            "tree_cloud", PointCloud2, queue_size=1)
        self.ground_cloud_pub = rospy.Publisher(
            "ground_cloud", PointCloud2, queue_size=1)
        # TODO(ankit): Check if this is needed
        self.odom_pub = rospy.Publisher(
            "/quadrotor/lidar_odom", Odometry, queue_size=100)

        # defining frame ids
        if self.use_sim == False:
            # range image frame
            self.range_image_frame = "body"
            self.reference_frame = "odom"
            self.faster_lio_world_frame = "camera_init"
            # undistorted point cloud frame
            self.undistorted_cloud_frame = "lidar"
        else:
            self.range_image_frame = "quadrotor"
            self.undistorted_cloud_frame = "quadrotor"
            self.reference_frame = "quadrotor/odom"

        # SUBSCRIBERS
        if self.use_sim == False:
            rospy.loginfo("Running real-world experiments...")
            time.sleep(1)

            self.segmented_pc_sub = rospy.Subscriber(
                seg_pc_namespace + "/segmented_point_cloud_no_destagger", PointCloud2, callback=self.segmented_pc_cb, queue_size=10)
            self.odom_sub = rospy.Subscriber(
                odom_topic, Odometry, callback=self.odom_callback, queue_size=100)

        else:
            rospy.logwarn("Running simulation experiments. This mode is still under development and is not fully tested. Switch the self.use_sim flag to False to run real-world experiments which work correctly.")
            time.sleep(10)
            # we need to sync point cloud with odometry in order to be able to find the transform precisely
            self.sim_segmentation_sub = message_filters.Subscriber(
                "/quadrotor/fake_lidar/car_cloud", PointCloud2)
            self.odom_sub = message_filters.Subscriber(
                "/quadrotor/odom", Odometry)
            ts = message_filters.ApproximateTimeSynchronizer(
                [self.sim_segmentation_sub, self.odom_sub], 20, 0.01)
            ts.registerCallback(self.sim_segmented_synced_pc_cb)

    def sim_segmented_synced_pc_cb(self, car_cloud_msg, odom_msg):
        self.segmented_synced_pc_cb(car_cloud_msg)

    def segmented_pc_cb(self, seg_cloud_msg):
        self.segmented_synced_pc_cb(seg_cloud_msg)

    def segmented_synced_pc_cb(self, segmented_cloud_msg):
        rospy.loginfo_throttle(
            7, "Segmented point cloud received. Executing callback...")

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
            intensities = np.zeros_like(x_coords)
            # NOTE: Hardcoding car label for simulated experiments
            intensities[x_coords != 0] = 5
        else:
            intensities = (segmented_pc['intensity']).flatten()

        pc_xyzi = np.zeros((x_coords.shape[0], 4))
        pc_xyzi[:, 0] = x_coords
        pc_xyzi[:, 1] = y_coords
        pc_xyzi[:, 2] = z_coords
        pc_xyzi[:, 3] = intensities

        # threshold by range. Remove points that are farther away than self.valid_range_threshold
        valid_indices = threshold_by_range(self.valid_range_threshold, pc_xyzi)

        if np.sum(valid_indices) == 0:
            rospy.logwarn(
                "No valid points found after range thresholding. Skipping this scan!!! Make sure the self.valid_range_threshold is set correctly.")
            return

        # apply range thresholding
        pc_xyzi_thresholded = pc_xyzi[valid_indices, :]

        indices_mask = np.cumsum(valid_indices) - 1

        # Use odometry and transform point cloud to world frame and then publish it
        points_world_xyzi, points_body_xyzi = transform_publish_pc(self,
                                                                   current_raw_timestamp, pc_xyzi_thresholded)

        if points_world_xyzi is None or points_body_xyzi is None:
            rospy.logwarn(
                "Failed to transform point cloud to world frame. Skipping this scan!!!")
            rospy.logwarn(
                "This may be caused due to transform_publish_pc function not performing correctly. Check the above warning messages.")
            rospy.logwarn(
                "If you are replying bags, try setting /use_sim_time to true and add --clock flag to rosbag play")
            rospy.logwarn(
                "It may also be caused by excessive CPU load, play bag with slower rate")
            return
        else:
            # Intensity channel is filled with the class label for each segmented point cloud

            # Peforming Ground Plane Fitting (if ground is segmented as a class)
            if "ground" in self.cls_name_to_label.keys():
                points_world_xyzi_ground = points_world_xyzi[points_world_xyzi[:, 3]
                                                             == self.cls_name_to_label["ground"], :]

                if points_world_xyzi_ground.shape[0] > self.ransac_n_points:
                    # TODO(ankit): Maybe need to convert this ground plane coeff extraction process into a function
                    ground_pcd = o3d.geometry.PointCloud()
                    ground_pcd.points = o3d.utility.Vector3dVector(
                        points_world_xyzi_ground[:, :3])
                    plane_eq, _ = ground_pcd.segment_plane(distance_threshold=0.1,
                                                           ransac_n=self.ransac_n_points, num_iterations=100)
                    [a, b, c, d] = plane_eq
                    self.ground_plane_coeff = np.array([a, b, c, d])
                    rospy.loginfo_throttle(
                        7, f"Ground plane coefficients properly updated: {self.ground_plane_coeff} ...")
                else:
                    rospy.logwarn_throttle(
                        5, f"Not enough ground points({points_world_xyzi_ground.shape[0]}) found to fit ground plane (atleast {self.ransac_n_points} are needed). Setting ground plane coefficients to simulate flat ground everywhere.")
                    rospy.logwarn_throttle(
                        5, "If this warning persists, tune the self.ransac_n_points parameter according to the number of ground points.")
                    self.ground_plane_coeff = np.array([0, 0, 1, 0])

                # Preparing points for cylinder fitting
                # TODO(ankit): Do we still require the cylinder point cloud to be in the body frame?
                # TODO(ankit): Maybe the current method of cylinder fitting doesn't require an organized point cloud. Check and remove this if not needed.
                # Recreate the point cloud with NaNs for the points that are thresholded out from the original point cloud
                # This is done to maintain the original point cloud structure (organized point cloud) for cylinder fitting
                points_body_xyzi_organized = points_body_xyzi[indices_mask, :]
                points_body_xyzi_organized[valid_indices == 0, :] = np.NaN

                points_body_xyzi_cylinder_masked = points_body_xyzi_organized.copy()
                points_body_xyzi_ground_masked = points_body_xyzi_organized.copy()
                non_cylinder_idx = None
                points_body_xyzi_ground_masked[points_body_xyzi_organized[:, 3]
                                               != self.cls_name_to_label["ground"], :] = np.NaN

                for cur_class_label in self.cls_labels_for_cylinder:
                    if non_cylinder_idx is None:
                        non_cylinder_idx = points_body_xyzi_organized[:,
                                                                      3] != cur_class_label
                    else:
                        non_cylinder_idx = np.logical_and(
                            non_cylinder_idx, points_body_xyzi_organized[:, 3] != cur_class_label)

                points_body_xyzi_cylinder_masked[non_cylinder_idx, :] = np.NaN

                pc_msg_cylinder = publish_cylinder_cloud(self.pc_fields_, points_body_xyzi_cylinder_masked.reshape(
                    (self.pc_height, self.pc_width, 4)), current_raw_timestamp, self.range_image_frame, pc_width=self.pc_width, pc_height=self.pc_height, pc_point_step=self.pc_point_step)
                self.cylinder_cloud_pub.publish(pc_msg_cylinder)

                pc_msg_ground = publish_ground_cloud(self.pc_fields_, points_body_xyzi_ground_masked.reshape(
                    (self.pc_height, self.pc_width, 4)), current_raw_timestamp, self.range_image_frame, pc_width=self.pc_width, pc_height=self.pc_height, pc_point_step=self.pc_point_step)
                self.ground_cloud_pub.publish(pc_msg_ground)
            else:
                # setting ground plane coeff to simulate flat ground everywhere.
                self.ground_plane_coeff = np.array([0, 0, 1, 0])
                rospy.logwarn_throttle(
                    5, f"No ground class found in the segmented point cloud. Skipping cylinder model fitting & setting ground plane coefficients to simulate flat ground everywhere ({self.ground_plane_coeff}).")

            # Performing Cuboid Fitting and Tracking
            # TODO(ankit): Maybe in the future, vectorizing this per-class instance segmentation pipeline may provide performance improvements.
            for cur_class_label in self.cls_labels_for_cuboid:
                points_world_xyzi_cuboid = points_world_xyzi[points_world_xyzi[:, 3]
                                                             == cur_class_label, :]

                if points_world_xyzi_cuboid.shape[0] == 0:
                    rospy.logwarn_throttle(
                        5, f"No points found to fit cuboid for class {self.cls_label_to_name[cur_class_label]}. Skipping this class for current scan.")
                    continue

                # filter out points that are too close to the ground
                dist_to_ground = calc_dist_to_ground(
                    self, points_world_xyzi_cuboid)
                points_cuboid_valid = points_world_xyzi_cuboid[dist_to_ground >
                                                               self.ground_median_increment, :]

                if points_cuboid_valid.shape[0] == 0:
                    rospy.logwarn(
                        f"No valid {self.cls_label_to_name[cur_class_label]} points found above ground. Skipping this class for current scan.")
                    rospy.logwarn(
                        "This is likely due to all points being too close to the ground or an error in the calc_dist_to_ground function. Also try tuning the self.ground_median_increment parameter.")
                    continue

                # Perform two layer clustering to remove noisy points
                # First layer clustering is done with small epsilon and min_samples to remove noisy points which don't belong to any cluster.
                # These noisy points by being close to other clusters can cause two clusters to be merged into one.
                # E.g. when two cars are close to each other, the noisy points in between them can cause them to be merged into one cluster.
                labels = cluster(self, points_cuboid_valid, epsilon=self.epsilon_scan_1st_layer,
                                 min_samples=self.min_samples_scan_1st_layer, use_2d=self.use_2d_1st_layer)

                points_cuboid_valid = points_cuboid_valid[labels != -1, :]

                if points_cuboid_valid.shape[0] == 0:
                    rospy.logwarn(
                        f"No valid {self.cls_label_to_name[cur_class_label]} clusters found after FIRST LAYER of clustering. Skipping this class for current scan. Tuning of parameters may be needed.")
                    continue

                # Second layer clustering is done with larger epsilon and min_samples to cluster the remaining clean points into separate cluster instances.
                # labels = cluster(self, points_cuboid_valid, epsilon=self.epsilon_scan_2nd_layer,
                #                  min_samples=self.min_samples_scan_2nd_layer, use_2d=self.use_2d_2nd_layer)
                
                labels = cluster(self, points_cuboid_valid, epsilon=self.clustering_params_per_cls_label[cur_class_label][0],
                                 min_samples=self.clustering_params_per_cls_label[cur_class_label][1], use_2d=self.use_2d_2nd_layer)

                # extract final valid points and their instance labels
                points_cuboid_valid = points_cuboid_valid[labels != -1, :]
                labels = labels[labels != -1]

                if points_cuboid_valid.shape[0] == 0:
                    rospy.logwarn(
                        f"No valid {self.cls_label_to_name[cur_class_label]} clusters found after SECOND LAYER of clustering. Skipping this class for current scan. Tuning of parameters may be needed.")
                    continue

                # Fit cuboids to the semantic instances to start the tracking process
                raw_points_xyz = points_cuboid_valid[:, :3]
                rospy.loginfo_throttle(
                    7, f"Fitting initial cuboids for {self.cls_label_to_name[cur_class_label]} instances to start tracking...")
                # xcs, ycs, lengths, widths, raw_points = fit_cuboid(self.fit_cuboid_dim_thresh, raw_points_xyz, labels)
                xcs, ycs, lengths, widths, raw_points = fit_cuboid(
                    self.fit_cuboid_dim_thresh, points_cuboid_valid[:, :3], labels)

                # N*4 objects, first two columns are x and y coordinates, third column is length (x-range), and fourth colum is width (y-range)
                cur_objects = np.transpose(
                    np.asarray([xcs, ycs, lengths, widths]))

                if cur_objects.shape[0] == 0:
                    rospy.logwarn_throttle(
                        5, f"No valid initial cuboid to start tracking for {self.cls_label_to_name[cur_class_label]}. Skipping this class for current scan.")
                    rospy.logwarn_throttle(
                        5, "If this warning persists, try tuning the self.fit_cuboid_dim_thresh parameter.")
                    continue

                # Tracking the current class instance across scans
                rospy.loginfo_throttle(
                    7, f"Tracking {self.cls_label_to_name[cur_class_label]} instances across scans...")
                self.all_objects, self.all_tracks = track_objects_final(
                    self, cur_class_label, cur_objects, raw_points, self.all_objects, self.all_tracks, self.processed_scan_idx, self.downsample_res, self.num_instance_point_lim)

                # Publish track centroids as cylinders for visualization
                publish_markers(
                    self, self.all_tracks, cur_cls_name=self.cls_label_to_name[cur_class_label])

            # get rid of too old tracks to bound computation and make sure our cuboid measurements are local and do not incorporate too much odom noise
            if self.track_always_visualize == False:
                idx_to_delete = []
                for idx, track in enumerate(self.all_tracks):
                    num_lost_track_times = self.processed_scan_idx - track.last_update_scan_idx
                    if num_lost_track_times > self.num_lost_track_times_thresh:
                        idx_to_delete.append(idx)

                # delete in descending order
                for idx in sorted(idx_to_delete, reverse=True):
                    del self.all_tracks[idx]
                    del self.all_objects[idx]

            # Publishing segmented, tracked, and accumulated instance point cloud based on the age of the accumulated track.
            # Tracks below a certain age are not considered for instance point cloud generation and cuboid fitting.
            # self.cuboid_track_age_threshold_per_cls_label dict is used inside this function to filter out tracks below a certain age.
            extracted_instances_xyzl = generate_publish_instance_cloud(self,
                                                                       current_raw_timestamp)

            if extracted_instances_xyzl is not None:
                # Fitting final cuboids on the accumulated instance point cloud
                cuboids = cuboid_detection(
                    self, extracted_instances_xyzl, current_raw_timestamp)
                if len(cuboids) > 0:
                    # Publishing cuboids
                    publish_cuboid_markers(
                        self, cuboids, current_raw_timestamp)
                else:
                    rospy.logwarn_throttle(
                        5, "No valid cuboids found from the accumulated instance point cloud. Tune the cuboid dimension thresholds and other parameters in cuboid_detection function if this warning persists.")

    def odom_callback(self, msg):
        send_tfs(self, msg)


if __name__ == '__main__':

    node_name = rospy.get_param(
        "/process_cloud_node_name", default="process_cloud_node")
    rospy.init_node(node_name)
    process_cloud_node = ProcessCloudNode(node_name)
    while not rospy.is_shutdown():
        print("node started!")
        rospy.spin()

    print("node killed!")
