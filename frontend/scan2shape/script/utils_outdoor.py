#! /usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
# from ros_numpy.point_cloud2 import fields_to_dtype, DUMMY_FIELD_PREFIX

# make module semantic_exploration visible by adding it to python path
# TODO(ankit): Figure out the purpose of appending parent and grandparent directories to sys.path
# -------------------------------------------------------------------
# import sys
# import os
# dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
# sys.path.append(dir)
# dir2 = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(dir2)
# -------------------------------------------------------------------


def cluster(process_cloud_node_object, xyzi, epsilon=1.25, min_samples=100, use_2d=True):
    # reduce to 2d
    if use_2d:
        cloud = xyzi[:, :2]
    else:
        cloud = xyzi[:, :3]
    # ref[https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html#sklearn.cluster.DBSCAN]
    object_clusters = DBSCAN(eps=epsilon, min_samples=min_samples, metric='euclidean', metric_params=None,
                             algorithm='auto', leaf_size=30, p=None, n_jobs=1).fit(cloud)
    labels = object_clusters.labels_
    process_cloud_node_object.save_fig_counter += 1
    if process_cloud_node_object.visualize & (process_cloud_node_object.save_fig_counter % 10 == 0):
        process_cloud_node_object.save_fig_idx += 1
        # TODO(ankit): Add directory (process_cloud_node_object.output_dir) as param to save figure
        show_clusters(cloud, labels, object_clusters,
                      process_cloud_node_object.save_fig_idx, process_cloud_node_object.output_dir)
    return labels


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

# Currently not used
# ---------------------------------------------------------------------
# def cloud2array(cloud_msg, squeeze=True):
#     dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
#     # parse the cloud into an array
#     cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
#     # remove the dummy fields that were added
#     cloud_arr = cloud_arr[
#         [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]
#     if squeeze and cloud_msg.height == 1:
#         return np.reshape(cloud_arr, (cloud_msg.width,))
#     else:
#         return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))
# ---------------------------------------------------------------------


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


def transform_publish_pc(process_cloud_node_object, current_timestamp, pc_xyzi_thresholded):
    # transform semantic cloud, apply filtering, and publish the segmented and filtered point cloud in world frame
    H_world_pano = np.zeros((4, 4))

    try:
        # transform data from the source_frame into the target_frame
        (t_world_pano, quat_world_pano) = process_cloud_node_object.tf_listener2.lookupTransform(
            process_cloud_node_object.undistorted_cloud_frame, process_cloud_node_object.reference_frame, current_timestamp)
        r_world_pano = R.from_quat(quat_world_pano)
        H_world_pano_rot = r_world_pano.as_matrix()
        H_world_pano_trans = np.array(t_world_pano)

        # transformation matrix from world to robot
        H_world_pano[:3, :3] = H_world_pano_rot
        H_world_pano[:3, 3] = H_world_pano_trans
        H_world_pano[3, 3] = 1

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("For timestamp, " + str(current_timestamp.to_sec()) + ", cannot find TF from " + process_cloud_node_object.reference_frame +
                      " to " + process_cloud_node_object.undistorted_cloud_frame + ", skipping this point cloud.")
        return None, None

    points_pano_xyz = pc_xyzi_thresholded[:, :3]

    # convert to homogeneous representation
    points_pano_xyz_homg = np.ones((points_pano_xyz.shape[0], 4))
    points_pano_xyz_homg[:, :3] = points_pano_xyz
    points_world_xyz_homg = (np.linalg.pinv(
        H_world_pano) @ points_pano_xyz_homg.T).T

    factor_stacked = np.repeat(
        points_world_xyz_homg[:, 3].reshape(-1, 1), 3, axis=1)
    # normalize
    points_world_xyz = np.divide(
        points_world_xyz_homg[:, :3], factor_stacked)

    points_world_xyzi = np.zeros((pc_xyzi_thresholded.shape[0], 4))
    points_world_xyzi[:, :3] = points_world_xyz
    points_world_xyzi[:, 3] = pc_xyzi_thresholded[:, 3]

    pc_msg = PointCloud2()
    # define our own header to publish in the world frame
    header = Header()
    header.stamp = current_timestamp
    header.frame_id = process_cloud_node_object.reference_frame
    pc_msg.header = header
    pc_msg.width = points_world_xyzi.shape[0]
    pc_msg.height = 1
    pc_msg.point_step = process_cloud_node_object.pc_point_step
    pc_msg.row_step = pc_msg.width * pc_msg.point_step
    pc_msg.fields = process_cloud_node_object.pc_fields_
    full_data = points_world_xyzi.astype(np.float32)
    pc_msg.data = full_data.tobytes()

    process_cloud_node_object.segmented_pc_pub.publish(pc_msg)
    rospy.loginfo_throttle(5, "Filtered segmented point cloud published")

    # prepare point clouds for cylinder fitting
    if process_cloud_node_object.use_sim == False:
        H_body_pano = np.zeros((4, 4))
        try:
            # transform data in the source_frame into the target_frame
            (t_body_pano, quat_body_pano) = process_cloud_node_object.tf_listener2.lookupTransform(
                process_cloud_node_object.undistorted_cloud_frame, process_cloud_node_object.range_image_frame, current_timestamp)
            r_body_pano = R.from_quat(quat_body_pano)
            H_body_pano_rot = r_body_pano.as_matrix()
            H_body_pano_trans = np.array(t_body_pano)
            H_body_pano[:3, :3] = H_body_pano_rot
            H_body_pano[:3, 3] = H_body_pano_trans
            H_body_pano[3, 3] = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("For timestamp, " + current_timestamp + ", cannot find TF from " + process_cloud_node_object.range_image_frame +
                          " to " + process_cloud_node_object.undistorted_cloud_frame + ", skipping this point cloud.")
            return None, None

        points_body_xyz_homg = (np.linalg.pinv(
            H_body_pano) @ points_pano_xyz_homg.T).T

        factor_stacked = np.repeat(
            points_body_xyz_homg[:, 3].reshape(-1, 1), 3, axis=1)
        # normalize
        points_body_xyz = np.divide(
            points_body_xyz_homg[:, :3], factor_stacked)
    else:
        if process_cloud_node_object.range_image_frame == process_cloud_node_object.undistorted_cloud_frame:
            points_body_xyz = points_pano_xyz_homg[:, :3]
        else:
            raise Exception(
                "range_image_frame is different from undistorted_cloud_frame for running simulation, which is incorrect!!")

    points_body_xyzi = np.zeros((pc_xyzi_thresholded.shape[0], 4))
    points_body_xyzi[:, :3] = points_body_xyz
    points_body_xyzi[:, 3] = pc_xyzi_thresholded[:, 3]

    return points_world_xyzi, points_body_xyzi


def send_tfs(process_cloud_node_object, msg):

    # send the transform from the camera_init (faster-lio's world frame) to our world frame (odom)
    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        msg.header.stamp,
        process_cloud_node_object.faster_lio_world_frame,
        # If using LLOL's ouster driver, frame is rotated by 180 degrees
        process_cloud_node_object.reference_frame
    )
    
    if process_cloud_node_object.run_kitti:
        # publish odom as tf between faster_lio_world_frame and range_image_frame
        # extract the rotation and translation from the odom message
        t = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # here pose is pose_camera according to semantic KITTI
        # pose_lidar = Tr_inv * Pose_camera *Tr
        # Tr is transformation matrix from camera to lidar
        # Tr: Tr: 4.276802385584e-04 -9.999672484946e-01 -8.084491683471e-03 -1.198459927713e-02 -7.210626507497e-03 8.081198471645e-03 -9.999413164504e-01 -5.403984729748e-02 9.999738645903e-01 4.859485810390e-04 -7.206933692422e-03 -2.921968648686e-01
        # Tr = np.array([[4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02],
        #                     [-7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02],
        #                         [9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01],
        #                             [0, 0, 0, 1]])
        # # get the transformation matrix
        # H_cam_world = np.eye(4)
        # H_cam_world[:3, :3] = R.from_quat(q).as_matrix()
        # H_cam_world[:3, 3] = t

        # # invert the Tr
        # Tr_inv = np.linalg.inv(Tr)
        # # apply the transformation matrix
        # H_lidar_world = Tr_inv @ H_cam_world @ Tr
        # # extract the rotation and translation from the inverted transformation matrix
        # t = H_lidar_world[:3, 3]
        # q = R.from_matrix(H_lidar_world[:3, :3]).as_quat()
        # rospy.logwarn_throttle(5, "RUNNING KITTI BENCHMARK!")

        # # # get the transformation matrix
        # # H = np.eye(4)
        # # H[:3, :3] = R.from_quat(q).as_matrix()
        # # H[:3, 3] = t
        # # # invert the transformation matrix
        # # H_inv = np.linalg.inv(H)
        # # # extract the rotation and translation from the inverted transformation matrix
        # # t = H_inv[:3, 3]
        # # q = R.from_matrix(H_inv[:3, :3]).as_quat()


        H = np.eye(4)
        H[:3, :3] = R.from_quat(q).as_matrix()
        H[:3, 3] = t
        # invert the transformation matrix
        # H_inv = np.linalg.inv(H)
        # H_inv = H
        # extract the rotation and translation from the inverted transformation matrix
        t = H[:3, 3]
        q = R.from_matrix(H[:3, :3]).as_quat()
        process_cloud_node_object.odom_broadcaster.sendTransform(
            t,
            q,
            msg.header.stamp,
            process_cloud_node_object.range_image_frame,
            process_cloud_node_object.faster_lio_world_frame
        )

    # publish the odometry message
    odom_msg = msg
    process_cloud_node_object.odom_pub.publish(odom_msg)

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        # for LLOL's ouster driver use (0 0 1 0), and for original ouster driver use (0 0 0 1)
        (0, 0, 0, 1),
        msg.header.stamp,
        "os_sensor",
        process_cloud_node_object.range_image_frame
    )

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        msg.header.stamp,
        process_cloud_node_object.undistorted_cloud_frame,
        "os_sensor"
    )

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        msg.header.stamp,
        "quadrotor/odom",
        "quadrotor/map"
    )

    process_cloud_node_object.odom_broadcaster.sendTransform(
        (0, 0, 0),
        (0, 0, 0, 1),
        msg.header.stamp,
        "quadrotor/map",
        process_cloud_node_object.reference_frame
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
    # create fields for the point cloud publisher
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
    return fields


def threshold_by_range(valid_range_threshold, pc_xyzi, valid_range_threshold_lower=0.1):
    points_pano_xyz = pc_xyzi[:, :3]
    range_values = np.linalg.norm(points_pano_xyz, axis=1)
    # filter out points at 0
    valid_indices = (range_values >= valid_range_threshold_lower)
    # filter out points larger than valid_range_threshold
    valid_indices = np.logical_and(
        (range_values < valid_range_threshold), valid_indices)
    return valid_indices
