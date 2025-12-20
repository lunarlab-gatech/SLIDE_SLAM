#! /usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from scipy.spatial.transform import Rotation as R
import tf
from typing import Type, Any
from sklearn.decomposition import PCA
import open3d as o3d
import copy
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def generate_publish_instance_cloud_indoor(process_cloud_node_object: Type[Any], timestamp: rospy.Time) -> tuple:
    
    instances_xyzl: list = []
    global_track_ids: list[int] = []
    current_instance_num: int = -1

    # Flag to check if it's safe to publish (due to having at least one valid cluster)
    safe_to_publish = False

    # If there are tracks available, iterate through each track older than age threshold
    if len(process_cloud_node_object.all_tracks) > 0:
        for track in process_cloud_node_object.all_tracks:
            if track.age > process_cloud_node_object.tracker_age_thresh_lower:

                # Increment instance number
                current_instance_num += 1

                # Extract points and class label
                current_xyz: np.ndarray = track.all_raw_points
                cur_class_label: int | None = track.class_label

                # Store track points/labels and track ids
                instances_xyzl.append([current_xyz, cur_class_label])
                global_track_ids.append(track.track_idx)

                # Create array with points and instance ids (used for visualization)
                current_i = np.ones((track.all_raw_points.shape[0], 1)) * current_instance_num
                current_xyzi = np.hstack((current_xyz, current_i))

                # Accumulate all valid points and labels
                if current_instance_num == 0:
                    valid_points_labels = current_xyzi
                    safe_to_publish = True
                else:
                    valid_points_labels = np.vstack((valid_points_labels, current_xyzi))
    else:
        print("No valid cluster found in current accumulated point cloud!")
        return None, None

    # Publish the accumulated instance cloud if safe
    if safe_to_publish:

        # Create PointCloud2 message
        pc_msg_2 = PointCloud2()
        header = Header()
        header.stamp = timestamp 
        header.frame_id = process_cloud_node_object.reference_frame
        pc_msg_2.header = header
        pc_msg_2.width = valid_points_labels.shape[0]
        pc_msg_2.height = 1
        pc_msg_2.point_step = 16
        pc_msg_2.row_step = pc_msg_2.width * pc_msg_2.point_step
        pc_msg_2.fields = process_cloud_node_object.pc_fields_
        full_data_2 = valid_points_labels.astype(np.float32)
        pc_msg_2.data = full_data_2.tobytes()

        # Publish the point cloud
        process_cloud_node_object.instance_cloud_pub.publish(pc_msg_2)
        rospy.loginfo_throttle(5, "Published segmented and accumulated instance cloud")

        # Return copies of the points/class ids and global track ids
        instances_xyzl_copied = copy.deepcopy(instances_xyzl)
        global_track_ids_copied = copy.deepcopy(global_track_ids)

        return instances_xyzl_copied, global_track_ids_copied
    else:
        return None, None


def cuboid_detection_indoor(process_cloud_node_object: Type[Any], instances_xyzl: list, 
                            instance_global_ids: list):

    # Initialize lists for detected cuboids and their centroids
    cuboids: list = []
    cuboid_plus_centroids: list= []

    # Check input argument consistency
    assert len(instances_xyzl) == len(instance_global_ids)

    # Always doing PCA.
    do_pca = True

    # For each track instance
    for instance_id, instance_xyzl in enumerate(instances_xyzl):

        # Extract the data
        instance_xyz: np.ndarray = instance_xyzl[0]
        cur_class_label: int = instance_xyzl[1]

        # Find the corresponding class name
        for key, value in process_cloud_node_object.cls.items():
            if value == cur_class_label:
                cur_object_class: str = key
                break
        
        # If PCA is enabled (currently hardcoded to always be enabled)
        if do_pca:
            # Initialize PCA class
            pca = PCA(n_components=2)

            # If more than 50 points, compute convex hull and only fit on hull points
            if instance_xyz.shape[0] > 50:
                o_pcd = o3d.geometry.PointCloud()
                o_pcd.points = o3d.utility.Vector3dVector(instance_xyz)
                try:
                    hull_points = o_pcd.compute_convex_hull()[1]
                    instance_hull_points = instance_xyz[hull_points, :]
                except: # In case convex hull computation fails
                    instance_hull_points = instance_xyz

            # Skipping PCA if less than 50 points for the instance (just using all points)
            else:
                instance_hull_points = instance_xyz

            # Fit PCA on the xy instance hull points
            pca.fit(instance_hull_points[:, :2])

            components = pca.components_
            x = np.array([components[0, 0], components[0, 1], 0])
            x = x / np.linalg.norm(x)
            z = np.array([0, 0, 1])
            y = np.cross(z, x)
            y = y / np.linalg.norm(y)

            # get the rotation
            # use arctan2 to avoid angle ambiguity, further, we constrain the yaw to [0,pi) so that the heading of object does not jump
            yaw = np.arctan2(x[1], x[0])
            # record the raw rotation which will be used for centroid projection since it is representing the exact rotation from PCA to world frame
            raw_yaw = yaw

            if yaw < 0:
                yaw += np.pi
            if yaw == np.pi:
                yaw -= np.pi

            # rotation from PCA frame to world frame -- will be used to find centroid as well
            r_pca_world_raw = R.from_rotvec(raw_yaw * z)

            x_projections = instance_xyz @ x
            max_x = np.max(x_projections)
            min_x = np.min(x_projections)
            length = max_x - min_x

            y_projections = instance_xyz @ y
            max_y = np.max(y_projections)
            min_y = np.min(y_projections)
            width = max_y - min_y

            z_projections = instance_xyz @ z
            max_z = np.max(z_projections)
            min_z = np.min(z_projections)
            height = max_z - min_z

            # Calculate the centroid found based on body frame coords
            x_centroid_pca = 0.5*(max_x + min_x)
            y_centroid_pca = 0.5*(max_y + min_y)
            z_centroid_pca = 0.5*(max_z + min_z)
            centroid_pca = np.array(
                [x_centroid_pca, y_centroid_pca, z_centroid_pca])
            centroid_world = r_pca_world_raw .as_matrix() @ centroid_pca

            r_pca_world = R.from_rotvec(yaw * z)
            orient_world = r_pca_world.as_quat()

        # If PCA is disabled (currently hardcoded to always be enabled)
        else:
            # Not using PCA
            x = np.array([1, 0, 0])
            z = np.array([0, 0, 1])
            y = np.array([0, 1, 0])

            # use arctan2 to avoid angle ambiguity, further, we constrain the yaw to [0,pi) so that the heading of car does not jump
            yaw = np.arctan2(x[1], x[0])
            # record the raw rotation which will be used for centroid projection since it is representing the exact rotation from PCA to world frame
            raw_yaw = yaw

            if yaw < 0:
                yaw += np.pi
            if yaw == np.pi:
                yaw -= np.pi

            # take the max - min along x axis
            length = np.max(instance_xyz[:, 0]) - np.min(instance_xyz[:, 0])
            # take the max - min along y axis
            width = np.max(instance_xyz[:, 1]) - np.min(instance_xyz[:, 1])
            # take the max - min along z axis
            height = np.max(instance_xyz[:, 2]) - np.min(instance_xyz[:, 2])

            # Calculate the centroid
            x_centroid = np.median(instance_xyz[:, 0])
            y_centroid = np.median(instance_xyz[:, 1])
            z_centroid = np.median(instance_xyz[:, 2])
            centroid_world = np.array([x_centroid, y_centroid, z_centroid])
            orient_world = [0, 0, 0, 1]

        # Check if the detected cuboid meets the size criteria for its class
        flag = (length > process_cloud_node_object.length_cutoffs[cur_object_class][0] and 
                length < process_cloud_node_object.length_cutoffs[cur_object_class][1] and 
                height > process_cloud_node_object.height_cutoffs[cur_object_class][0] and 
                height < process_cloud_node_object.height_cutoffs[cur_object_class][1])
        
        # If criteria met, store the cuboid information (consider it successfully fit)
        if flag:
            current_cube: dict = {}
            current_cube['dimensions'] = np.array([length, width, height])
            current_cube['centroid'] = centroid_world
            current_cube['orientation'] = orient_world  # unit quaternion
            current_cube['global_id'] = instance_global_ids[instance_id]
            current_cube['class_label_str'] = cur_object_class
            cuboids.append(current_cube)
            cuboid_plus_centroids.append(np.array([0.0, 0.0, centroid_world[2]]))

    return (cuboids, cuboid_plus_centroids)


def fit_cuboid_indoor(fit_cuboid_length_thresh: float, input_pc: np.ndarray, depth_percentile: tuple, 
                      confidence_threshold: float):
    """
    Parameters:
        input_pc: Array with x, y, z, intensity, instance_id, confidence, depth
    """

    # Extract individual components from input_pc
    cloud_mat_3d = input_pc[:, :3]
    seg_ids = input_pc[:, 4]
    confidence_values = input_pc[:, 5]
    depth_values = input_pc[:, 6]

    # Get unique segment ids
    unique_seg_ids: np.ndarray = np.unique(seg_ids)

    # Create variables for storing fit cuboids
    xcs: list = []
    ycs: list = []
    lengths: list = []
    widths: list = []
    raw_points: list = []

    # For each unique segment id, fit a cuboid if conditions are met
    for k in unique_seg_ids:

        # Skip background points or other instances
        if k == 0: continue
        
        # Get masks of points belonging to current segment id
        class_member_mask: np.ndarray = (seg_ids == k)

        # If at least one point belongs to this segment
        if np.sum(class_member_mask) > 0:

            # Extract depth values for the current seg id
            cur_depth_values: np.ndarray = depth_values[class_member_mask]

            # Convert the depth percentile thresholds to actual depth value thresholds
            depth_thres_low: float = np.percentile(cur_depth_values, depth_percentile[0])
            depth_thres_high: float = np.percentile(cur_depth_values, depth_percentile[1])

            # Get median confidence for the current seg id points
            confidence: float = np.median(confidence_values[class_member_mask])

            # If confidence threshold is met...
            if confidence > confidence_threshold:

                # And if there are valid points within the depth thresholds...
                valid_pts_mask: np.ndarray = np.logical_and((cur_depth_values > depth_thres_low), (cur_depth_values < depth_thres_high))
                if np.sum(valid_pts_mask) > 0:

                    # Extract valid points for this seg id
                    xyzs_instance: np.ndarray = cloud_mat_3d[class_member_mask, :]
                    xyzs: np.ndarray = xyzs_instance[valid_pts_mask, :]
                    xyzs_valid = xyzs

                    # Fit cuboid to these valid points
                    xs: np.ndarray = xyzs_valid[:, 0]
                    ys: np.ndarray = xyzs_valid[:, 1]
                    x_max: float = np.max(xs)
                    y_max: float = np.max(ys)
                    x_min: float = np.min(xs)
                    y_min: float = np.min(ys)
                    xc: float = np.median(xs)
                    yc: float = np.median(ys)
                    length: float = x_max - x_min
                    width: float = y_max - y_min

                    # If the length is greater than threshold, save the fit cuboid
                    if length > fit_cuboid_length_thresh:
                        xcs.append(xc)
                        ycs.append(yc)
                        lengths.append(length)
                        widths.append(width)
                        raw_points.append(xyzs_valid)

    # Verify data consistency and return results
    assert (len(xcs) == len(ycs) == len(lengths) == len(widths) == len(raw_points))
    return xcs, ycs, lengths, widths, raw_points

def cluster_indoor(xyzi, epsilon, min_samples, use_2d, cpu_thread_clutering_=2):
    # reduce to 2d
    if use_2d:
        cloud = xyzi[:, :2]
    else:
        cloud = xyzi[:, :3]
    # ref[https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html#sklearn.cluster.DBSCAN]
    object_clusters = DBSCAN(eps=epsilon, min_samples=min_samples, metric='euclidean', metric_params=None,
                             algorithm='auto', leaf_size=30, p=None, n_jobs=cpu_thread_clutering_).fit(cloud)
    labels = object_clusters.labels_
    # components = object_clusters.components_
    # core_sample_indices = object_clusters.core_sample_indices_

    return labels


def publish_cuboid_and_range_bearing_measurements_final(process_cloud_node_object: Type, cuboids: list, 
                                                        cuboid_clus_labels, current_raw_timestamp: rospy.Time):
    
    # Create MarkerArray messages (for reference frame and range image frame)
    car_markers = MarkerArray()
    car_markers.markers = []
    car_markers_body = MarkerArray()
    car_markers_body.markers = []

    # Track the timestamps
    stamp: rospy.Time = current_raw_timestamp

    # Get the pose of the reference frame wrt range image frame
    H_body_world = np.zeros((4, 4), dtype=np.float32)
    try:
        (t_body_world, quat_body_world) = process_cloud_node_object.tf_listener2.lookupTransform(
            process_cloud_node_object.reference_frame, process_cloud_node_object.range_image_frame, current_raw_timestamp)
        r_body_world = R.from_quat(quat_body_world)
        H_body_world_rot = r_body_world.as_matrix()
        H_body_world_trans = np.array(t_body_world)
        H_body_world[:3, :3] = H_body_world_rot
        H_body_world[:3, 3] = H_body_world_trans
        H_body_world[3, 3] = 1

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("cannot find TF from " + process_cloud_node_object.range_image_frame +
                      " to " + process_cloud_node_object.reference_frame)
        return

    # For each detected cuboid, create and publish markers
    for idx, cuboid in enumerate(cuboids):

        # Create the main marker
        marker = Marker()
        marker.header.frame_id = process_cloud_node_object.reference_frame
        marker.header.stamp = stamp
        if cuboid_clus_labels is not None: marker.ns = cuboid['class_label_str'] + "_" + str(cuboid_clus_labels[idx] + 1)
        else: marker.ns = cuboid['class_label_str']
        marker.id = idx

        # Get the CAD model if it exists
        cad_model_exists = False
        if process_cloud_node_object.class_model_path[cuboid['class_label_str']] is not None:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = process_cloud_node_object.class_model_path[cuboid['class_label_str']]
            cad_model_exists = True
        else:
            marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = cuboid['centroid'][0]
        marker.pose.position.y = cuboid['centroid'][1]
        marker.pose.position.z = cuboid['centroid'][2]

        # Create a second marker for the range image frame
        marker_body = Marker()
        marker_body.header.frame_id = process_cloud_node_object.range_image_frame
        marker_body.header.stamp = stamp
        marker_body.ns = cuboid['class_label_str']
        marker_body.id = idx
        marker_body.type = Marker.CUBE
        marker_body.action = Marker.ADD

        cuboid_center_homg = np.ones((4,), dtype=float)
        cuboid_center_homg[:3] = cuboid['centroid']
        cuboid_center_body_homg = np.linalg.pinv(H_body_world) @ cuboid_center_homg
        cuboid_center_body = cuboid_center_body_homg[:3] / cuboid_center_body_homg[3]

        marker_body.pose.position.x = cuboid_center_body[0]
        marker_body.pose.position.y = cuboid_center_body[1]
        marker_body.pose.position.z = cuboid_center_body[2]

        # color_by_floors option for debugging purposes
        if process_cloud_node_object.color_by_floors:
            for floor_name, floor_height_thresh in process_cloud_node_object.floor_height_thresh.items():
                if floor_height_thresh[0] <= cuboid['centroid'][2] <= floor_height_thresh[1]:
                    marker.color.a = 0.6  # alpha = 1 means not transparent at all
                    marker.color.r = process_cloud_node_object.floor_color[floor_name][0]
                    marker.color.g = process_cloud_node_object.floor_color[floor_name][1]
                    marker.color.b = process_cloud_node_object.floor_color[floor_name][2]

                    marker_body.color.a = 0.6  # alpha = 1 means not transparent at all
                    marker_body.color.r = process_cloud_node_object.floor_color[floor_name][0]
                    marker_body.color.g = process_cloud_node_object.floor_color[floor_name][1]
                    marker_body.color.b = process_cloud_node_object.floor_color[floor_name][2]

                    marker_colored = True

            if not marker_colored:
                marker.color.a = 0.8
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker_body.color.a = 0.6
                marker_body.color.r = 0.0
                marker_body.color.g = 0.0
                marker_body.color.b = 0.0

        # If not coloring by floors, use class colors
        else:
            marker.color.a = 1.0 
            marker.color.r = process_cloud_node_object.class_color[cuboid['class_label_str']][0]
            marker.color.g = process_cloud_node_object.class_color[cuboid['class_label_str']][1]
            marker.color.b = process_cloud_node_object.class_color[cuboid['class_label_str']][2]

            marker_body.color.a = 0.6 
            marker_body.color.r = process_cloud_node_object.class_color[cuboid['class_label_str']][0]
            marker_body.color.g = process_cloud_node_object.class_color[cuboid['class_label_str']][1]
            marker_body.color.b = process_cloud_node_object.class_color[cuboid['class_label_str']][2]

        # Set marker scales
        if cad_model_exists:
            fixed_dim = 0.6
            marker.scale.x = fixed_dim * process_cloud_node_object.class_model_scale[cuboid['class_label_str']]
            marker.scale.y = fixed_dim * process_cloud_node_object.class_model_scale[cuboid['class_label_str']]
            marker.scale.z = fixed_dim * process_cloud_node_object.class_model_scale[cuboid['class_label_str']]
        else:
            marker.scale.x = cuboid['dimensions'][0]
            marker.scale.y = cuboid['dimensions'][1]
            marker.scale.z = cuboid['dimensions'][2]

        marker_body.scale.x = cuboid['dimensions'][0]
        marker_body.scale.y = cuboid['dimensions'][0]
        marker_body.scale.z = cuboid['dimensions'][2]

        # Set marker orientations
        marker.pose.orientation.x = cuboid['orientation'][0]
        marker.pose.orientation.y = cuboid['orientation'][1]
        marker.pose.orientation.z = cuboid['orientation'][2]
        marker.pose.orientation.w = cuboid['orientation'][3]

        r_cube_world = R.from_quat(cuboid['orientation'])
        H_cube_world_rot = r_cube_world.as_matrix()
        H_cube_body_rot = H_body_world_rot.T @ H_cube_world_rot
        quat_body_cube = R.from_matrix(H_cube_body_rot).as_quat()
        marker_body.pose.orientation.x = quat_body_cube[0]
        marker_body.pose.orientation.y = quat_body_cube[1]
        marker_body.pose.orientation.z = quat_body_cube[2]
        marker_body.pose.orientation.w = quat_body_cube[3]

        # Append the markers to the MarkerArray messages
        car_markers.markers.append(marker)
        car_markers_body.markers.append(marker_body)

        # Add text to show labels
        text_marker = Marker()
        text_marker.header.frame_id = process_cloud_node_object.reference_frame
        text_marker.header.stamp = stamp
        text_marker.ns = "text"
        text_marker.id = idx
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = cuboid['centroid'][0]
        text_marker.pose.position.y = cuboid['centroid'][1]
        text_marker.pose.position.z = cuboid['centroid'][2] + 0.75
        text_marker.pose.orientation.w = 1.0
        text_marker.text = cuboid['class_label_str']
        text_marker.scale.z = 0.5
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        car_markers.markers.append(text_marker)

    # Publish the MarkerArray messages
    process_cloud_node_object.cuboid_marker_pub.publish(car_markers)
    process_cloud_node_object.cuboid_marker_body_pub.publish(car_markers_body)
