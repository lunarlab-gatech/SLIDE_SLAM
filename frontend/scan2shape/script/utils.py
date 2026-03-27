#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf
from typing import Type, Any
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


# TODO(ankit): Figure out the purpose of appending parent and grandparent directories to sys.path
# make module semantic_exploration visible by adding it to python path
# import sys
# import os
# dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
# sys.path.append(dir)
# dir2 = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(dir2)


def show_clusters(cloud_mat_2d, labels, tree_clustering, save_fig_idx, output_dir):

    lower_lim = np.median(cloud_mat_2d, axis=0) - 30
    upper_lim = np.median(cloud_mat_2d, axis=0) + 30

    # visualize raw data first
    plt.xlim([lower_lim[0], upper_lim[0]])
    plt.ylim([lower_lim[1], upper_lim[1]])
    plt.scatter(cloud_mat_2d[:, 0], cloud_mat_2d[:, 1], s=0.03)
    plt.savefig(output_dir +
                "raw_data_{0:04d}.png".format(save_fig_idx))
    plt.clf()

    # visualize clustering result -- ref [https://scikit-learn.org/stable/auto_examples/cluster/plot_dbscan.html#sphx-glr-auto-examples-cluster-plot-dbscan-py]
    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)
    # print('Estimated number of clusters: %d' % n_clusters_)
    # print('Estimated number of noise points: %d' % n_noise_)

    # Black color is removed and is used for noise points instead.
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [1, 1, 1, 1]
        class_member_mask = (labels == k)

        core_samples_mask = np.zeros_like(labels, dtype=bool)
        core_samples_mask[tree_clustering.core_sample_indices_] = True

        xy = cloud_mat_2d[class_member_mask & core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor=tuple(col), markersize=1)

        xy = cloud_mat_2d[class_member_mask & ~core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor=tuple(col), markersize=0.5)

    plt.xlim([lower_lim[0], upper_lim[0]])
    plt.ylim([lower_lim[1], upper_lim[1]])

    plt.title('Estimated number of clusters: %d' % n_clusters_)
    plt.savefig(output_dir +
                "clustered_data_{0:04d}.png".format(save_fig_idx))
    plt.clf()

def publish_cylinder_cloud(pc_fields, cylinder_cloud, timestamp, frame_id, pc_width=1024, pc_height=64, pc_point_step=16):
    pc_msg = PointCloud2()
    # define our own header to publish in the world frame
    header = Header()
    header.stamp = timestamp  # rospy.Time.now()
    header.frame_id = frame_id
    pc_msg.header = header
    pc_msg.width = pc_width
    pc_msg.height = pc_height
    pc_msg.point_step = pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = pc_fields
    full_data = cylinder_cloud.astype(np.float32)
    pc_msg.data = full_data.tobytes()
    return pc_msg

def publish_ground_cloud(pc_fields, ground_cloud, timestamp, frame_id, pc_height=64, pc_width=1024, pc_point_step=16):

    pc_msg = PointCloud2()
    # define our own header to publish in the world frame
    header = Header()
    header.stamp = timestamp  # rospy.Time.now()
    header.frame_id = frame_id
    pc_msg.header = header
    pc_msg.height = pc_height
    if pc_height != 1:
        pc_msg.width = pc_width
    else:
        pc_msg.width = ground_cloud.shape[0]
    pc_msg.point_step = pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = pc_fields
    full_data = ground_cloud.astype(np.float32)
    pc_msg.data = full_data.tobytes()

    return pc_msg

def calc_dist_to_ground(process_cloud_node_object, points):

    numerator = np.abs(points[:, 0] * process_cloud_node_object.ground_plane_coeff[0]
                       + points[:, 1] *
                       process_cloud_node_object.ground_plane_coeff[1]
                       + points[:, 2] *
                       process_cloud_node_object.ground_plane_coeff[2]
                       + process_cloud_node_object.ground_plane_coeff[3] * np.ones(points.shape[0]))
    denominator = np.linalg.norm(
        process_cloud_node_object.ground_plane_coeff[:3])
    dist_to_groud = numerator / denominator
    return dist_to_groud

def transform_publish_pc(process_cloud_node_object: Type[Any], current_timestamp: rospy.Time, 
                         pc_xyzi_id_conf_thres_camera: np.ndarray) -> tuple:
    """ 
    Transform semantic cloud, apply filtering, and publish the segmented and 
    filtered point cloud in world frame.
    """

    H_world_pano: np.ndarray = np.zeros((4, 4))
    try:
        # Get transformation of undistorted cloud frame wrt. reference frame
        (t_world_pano, quat_world_pano) = process_cloud_node_object.tf_listener2.lookupTransform(
            process_cloud_node_object.undistorted_cloud_frame, process_cloud_node_object.reference_frame, current_timestamp)
        r_world_pano: R = R.from_quat(quat_world_pano)
        H_world_pano_rot: np.ndarray = r_world_pano.as_matrix()
        H_world_pano_trans: np.ndarray = np.array(t_world_pano)

        # Convert into a 4x4 homogeneous transformation matrix
        H_world_pano[:3, :3] = H_world_pano_rot
        H_world_pano[:3, 3] = H_world_pano_trans
        H_world_pano[3, 3] = 1

    # Handle exceptions for TF lookup failures
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("For timestamp, " + str(current_timestamp.to_sec()) + ", cannot find TF from " + process_cloud_node_object.reference_frame + " to " + process_cloud_node_object.undistorted_cloud_frame + ", skipping this point cloud.")
        return None, None

    # Extract xyz points from the array
    points_pano_xyz: np.ndarray = pc_xyzi_id_conf_thres_camera[:, :3]

    # Convert points from undistorted cloud frame to reference frame
    points_pano_xyz_homg = np.ones((points_pano_xyz.shape[0], 4))
    points_pano_xyz_homg[:, :3] = points_pano_xyz
    points_world_xyz_homg = (np.linalg.pinv(H_world_pano) @ points_pano_xyz_homg.T).T

    # Normalize to convert from homogeneous to euclidean coordinates (due to use of pseudo-inverse)
    factor_stacked = np.repeat(points_world_xyz_homg[:, 3].reshape(-1, 1), 3, axis=1)
    points_world_xyz = np.divide(points_world_xyz_homg[:, :3], factor_stacked)

    # Recreate array with points and labels, but with points in reference frame (and extra depth dimension)
    points_world_xyzi_id_conf_depth = np.zeros((pc_xyzi_id_conf_thres_camera.shape[0], 7))
    points_world_xyzi_id_conf_depth[:, :3] = points_world_xyz
    points_world_xyzi_id_conf_depth[:, 3:6] = pc_xyzi_id_conf_thres_camera[:, 3:6]
    points_world_xyzi_id_conf_depth[:, 6] = pc_xyzi_id_conf_thres_camera[:, 2]

    # Create a PointCloud2 message for publishing points in the reference frame
    pc_msg = PointCloud2()
    header = Header()
    header.stamp = current_timestamp
    header.frame_id = process_cloud_node_object.reference_frame
    pc_msg.header = header
    pc_msg.width = points_world_xyzi_id_conf_depth.shape[0]
    pc_msg.height = 1
    pc_msg.point_step = process_cloud_node_object.pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = process_cloud_node_object.pc_fields_
    pc_msg.data = points_world_xyzi_id_conf_depth[:, :4].astype(np.float32).tobytes()

    # Publish the message
    process_cloud_node_object.segmented_pc_pub.publish(pc_msg)
    rospy.loginfo_throttle(5, "Filtered segmented point cloud published")

    # Prepare point clouds for cylinder fitting (currently not used)
    if process_cloud_node_object.use_sim == False:
        H_body_pano: np.ndarray = np.zeros((4, 4))
        try:
            # Find transformation of the undistorted cloud frame wrt. the range image frame
            (t_body_pano, quat_body_pano) = process_cloud_node_object.tf_listener2.lookupTransform(
                process_cloud_node_object.undistorted_cloud_frame, process_cloud_node_object.range_image_frame, current_timestamp)
            r_body_pano: R = R.from_quat(quat_body_pano)
            H_body_pano_rot = r_body_pano.as_matrix()
            H_body_pano_trans = np.array(t_body_pano)
            H_body_pano[:3, :3] = H_body_pano_rot
            H_body_pano[:3, 3] = H_body_pano_trans
            H_body_pano[3, 3] = 1

        # Handle exceptions for TF lookup failures
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("For timestamp, " + current_timestamp + ", cannot find TF from " + process_cloud_node_object.range_image_frame +
                          " to " + process_cloud_node_object.undistorted_cloud_frame + ", skipping this point cloud.")
            return None, None

        # Calculate the points in the range image frame (using normalization due to pseudo-inverse)
        points_body_xyz_homg = (np.linalg.pinv(H_body_pano) @ points_pano_xyz_homg.T).T
        factor_stacked = np.repeat(points_body_xyz_homg[:, 3].reshape(-1, 1), 3, axis=1)
        points_body_xyz = np.divide(points_body_xyz_homg[:, :3], factor_stacked)
        
    else:
        if process_cloud_node_object.range_image_frame == process_cloud_node_object.undistorted_cloud_frame:
            points_body_xyz = points_pano_xyz_homg[:, :3]
        else:
            raise Exception(
                "range_image_frame is different from undistorted_cloud_frame for running simulation, which is incorrect!!")

    # Get points w/labels in the range image frame
    points_body_xyzi = np.zeros((pc_xyzi_id_conf_thres_camera.shape[0], 6))
    points_body_xyzi[:, :3] = points_body_xyz
    points_body_xyzi[:, 3:] = pc_xyzi_id_conf_thres_camera[:, 3:]

    # Return the points w/labels in the reference frame, and points w/labels in the range image frame
    return points_world_xyzi_id_conf_depth, points_body_xyzi


def send_tfs(process_cloud_node_object: Type[Any], msg: Odometry):

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        msg.header.stamp,
        "quadrotor/odom",
        "quadrotor/map"  # If using LLOL's ouster driver, frame is rotated by 180 degrees
    )

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        msg.header.stamp,
        "quadrotor/map",
        "dragonfly67/odom"  # If using LLOL's ouster driver, frame is rotated by 180 degrees
    )

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        msg.header.stamp,
        "dragonfly67/odom",
        "odom"
    )

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        msg.header.stamp,
        "odom",
        "world"
    )

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
        msg.header.stamp,
        process_cloud_node_object.range_image_frame,
        process_cloud_node_object.reference_frame
    )

    odom_msg = msg
    process_cloud_node_object.odom_pub.publish(odom_msg)

    rot_mat = np.array([[0.0000000, -1.0000000,  0.0000000],
                        [0.0000000,  0.0000000, -1.0000000],
                        [1.0000000,  0.0000000,  0.0000000]])
    rot_cam_body_quat = R.from_matrix(rot_mat.T).as_quat()

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (rot_cam_body_quat[0], rot_cam_body_quat[1],
         rot_cam_body_quat[2], rot_cam_body_quat[3]),
        msg.header.stamp,
        process_cloud_node_object.undistorted_cloud_frame,
        process_cloud_node_object.range_image_frame
    )


def publish_accumulated_cloud(process_cloud_node_object, timestamp):
    pc_msg = PointCloud2()
    # define our own header to publish in the world frame
    header = Header()
    header.stamp = timestamp  # rospy.Time.now()
    header.frame_id = process_cloud_node_object.reference_frame
    pc_msg.header = header
    pc_msg.width = process_cloud_node_object.accumulated_semantic_cloud.shape[0]
    pc_msg.height = 1
    pc_msg.point_step = process_cloud_node_object.pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = process_cloud_node_object.pc_fields_
    full_data = process_cloud_node_object.accumulated_semantic_cloud.astype(
        np.float32)
    pc_msg.data = full_data.tobytes()
    process_cloud_node_object.accumulated_cloud_pub.publish(pc_msg)
    rospy.loginfo_throttle(5, "Accumulated semantic point cloud published")


def make_fields():
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
    return fields

def threshold_by_range(valid_range_threshold: float, pc_xyzi: np.ndarray, 
                       valid_range_threshold_lower: float = 0.1):
    """ Filters point cloud by range thresholds. """

    # Extract only xyz points from the array
    points_pano_xyz: np.ndarray = pc_xyzi[:, :3]

    # Compute range values from the sensor origin for each point
    range_values: np.ndarray = np.linalg.norm(points_pano_xyz, axis=1)

    # Filter out points closer than 'valid_range_threshold_lower'
    valid_indices: np.ndarray[bool] = (range_values >= valid_range_threshold_lower)

    # Filter out points further than 'valid_range_threshold'
    valid_indices: np.ndarray[bool] = np.logical_and((range_values < valid_range_threshold), valid_indices)
    return valid_indices
