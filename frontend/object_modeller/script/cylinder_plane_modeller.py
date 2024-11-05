#! /usr/bin/env python3

# subscribe to both /tree_cloud and /ground_cloud

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
import open3d as o3d
import copy
# import rviz marker and marker array
from visualization_msgs.msg import Marker, MarkerArray
# import R from scipy
from scipy.spatial.transform import Rotation as R
from sloam_msgs.msg import ROSCylinder, ROSCylinderArray, StampedRvizMarkerArray
import argparse
import message_filters
import sys

# This node models the tree as a cylinder and the ground as a plane


class CylinderPlaneModeller:
    def __init__(self, args):

        self.synced_tree_and_ground_clouds = None
        # make subcriber with queue size 1
        self.point_cloud_ns = args["point_cloud_ns"]

        # SUBSCRIBERS
        # create a synchronizer that subscribes to both tree_cloud and ground_cloud
        # TODO(ankit): since our process cloud node always publish tree_cloud and ground_cloud at the same time, this implementation is accurate, for future implemnetation, consider syncing them
        self.tree_cloud_sub = message_filters.Subscriber(
            self.point_cloud_ns+"tree_cloud", PointCloud2)
        self.ground_cloud_sub = message_filters.Subscriber(
            self.point_cloud_ns+"ground_cloud", PointCloud2)
        # sync with maximum time difference of 0.01 seconds
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.tree_cloud_sub, self.ground_cloud_sub], 10, 0.01)
        # register one callback named clouds_callback
        self.sync.registerCallback(self.clouds_callback)

        self.cuboid_sub = rospy.Subscriber(
            self.point_cloud_ns+"car_cuboids_body", MarkerArray, self.cuboid_callback)
        self.cuboid_indoor_sub = rospy.Subscriber(
            "chair_cuboids_body", MarkerArray, self.cuboid_indoor_callback)

        # PUBLISHERS
        self.tree_instance_pub = rospy.Publisher(
            "tree_instance_cloud", PointCloud2, queue_size=1)
        self.cylinder_marker_pub = rospy.Publisher(
            "cylinder_marker", MarkerArray, queue_size=1)
        self.ground_plane_marker_pub = rospy.Publisher(
            "ground_plane_marker", MarkerArray, queue_size=1)
        self.sync_meas_pub = rospy.Publisher(
            "cylinder_measurements", ROSCylinderArray, queue_size=10)
        self.cuboid_pub = rospy.Publisher(
            "cuboid_measurements", StampedRvizMarkerArray, queue_size=10)
        self.cuboid_indoor_pub = rospy.Publisher(
            "chair_cuboids_stamped", StampedRvizMarkerArray, queue_size=10)

        self.pc_fields_ = self.make_fields()

        params_name_prefix = f"{rospy.get_name()}/"

        # ----------------PARAMETERS-------------------
        # TODO(ankit): Delete unused parameters
        # self.min_distance_to_ground = rospy.get_param(params_name_prefix + "min_distance_to_ground", -0.5)
        self.angle_cutoff = np.deg2rad(rospy.get_param(
            params_name_prefix + "angle_cutoff", 15))
        self.radius_cutoff = rospy.get_param(
            params_name_prefix + "radius_cutoff", (0.05, 0.5))
        self.min_points_per_tree = rospy.get_param(
            params_name_prefix + "min_points_per_tree", 15)
        self.min_points_for_radius = rospy.get_param(
            params_name_prefix + "min_points_for_radius", 5)
        self.min_points_per_ground_patch = rospy.get_param(
            params_name_prefix + "min_points_per_ground_patch", 40)
        self.ground_plane_patch_size = rospy.get_param(
            params_name_prefix + "ground_plane_patch_size", 10)
        self.num_ground_plane_model_to_keep = rospy.get_param(
            params_name_prefix + "num_ground_plane_model_to_keep", 50)
        self.clus_eps = rospy.get_param(params_name_prefix + "clus_eps", 0.5)
        self.clus_min_samples = rospy.get_param(
            params_name_prefix + "clus_min_samples", 10)
        self.diameter_measure_height_above_ground = rospy.get_param(
            params_name_prefix + "diameter_measure_height_above_ground", 1.3716)
        self.default_radius = rospy.get_param(
            params_name_prefix + "default_radius", 0.2)
        run_rate = rospy.get_param(params_name_prefix + "run_rate", 20)
        # ----------------------------------------------

        # CONTAINERS
        self.latest_ground_plane_models = []
        self.latest_ground_plane_models_centroids = []
        # tree cloud counter
        self.tree_cloud_counter = 0
        self.tree_cloud_start_time = rospy.Time.now()
        self.average_tree_cloud_rate = 0

        self.tree_cloud_process_timer = rospy.Timer(
            rospy.Duration(1.0/run_rate), self.tree_cloud_process)

    # Callback to convert the standard MarkerArray to custom StampedRvizMarkerArray which also contains the msg header
    def cuboid_callback(self, msg):
        # create a new message
        stamped_rviz_marker_array = StampedRvizMarkerArray()
        # fill in the header
        stamped_rviz_marker_array.header = msg.markers[-1].header
        # fill in the marker array
        stamped_rviz_marker_array.cuboid_rviz_markers = msg
        # publish
        self.cuboid_pub.publish(stamped_rviz_marker_array)
        rospy.loginfo_throttle(
            7, "Successfully published stamped outdoor cuboid measurements, next will do msg syncing!")

    # Same as cuboid_callback but for indoor cuboids
    def cuboid_indoor_callback(self, msg):
       # read the timestamp of the last cuboid in the message
        # publish cuboid with the stamp
        # create a new message
        stamped_rviz_marker_array = StampedRvizMarkerArray()
        # fill in the header
        stamped_rviz_marker_array.header = msg.markers[-1].header
        # fill in the marker array
        stamped_rviz_marker_array.cuboid_rviz_markers = msg
        # publish
        self.cuboid_indoor_pub.publish(stamped_rviz_marker_array)
        rospy.loginfo_throttle(
            7, "Successfully published stamped indoor cuboid measurements, next will do msg syncing!")

    # Timesynced callback for both tree and ground cloud
    def clouds_callback(self, tree_cloud, ground_cloud):
        rospy.loginfo_throttle(
            5, "Received synced ground cloud and tree cloud")
        if self.tree_cloud_counter == 0:
            self.tree_cloud_start_time = rospy.Time.now()
        else:
            self.average_tree_cloud_rate = self.tree_cloud_counter / \
                max(0.01, (rospy.Time.now().to_sec() -
                    self.tree_cloud_start_time.to_sec()))
        self.tree_cloud_counter += 1

        self.synced_tree_and_ground_clouds = [tree_cloud, ground_cloud]

    # make timer back process function

    def tree_cloud_process(self, event):
        if self.synced_tree_and_ground_clouds is not None:
            start_time = rospy.Time.now().to_sec()
            self.cluster(self.synced_tree_and_ground_clouds)
            time_to_process = rospy.Time.now().to_sec() - start_time
            # avoid division by zero
            if time_to_process > 0:
                rate = 1.0 / time_to_process
            else:
                rate = 0.0

            if rate < self.average_tree_cloud_rate:
                rospy.logwarn("Time to process tree cloud is longer than the incoming tree point cloud rate. Tree and ground cloud will be reset. Tune the clustering parameters or slow down the incoming point cloud. Processing rate: %.2f, Average incoming rate: %.2f", rate, self.average_tree_cloud_rate)
                # reset the latest_tree_cloud
                # reset it to avoid processing the same cloud again
                self.synced_tree_and_ground_clouds = None

    def cluster(self, cur_synced_tree_and_ground_clouds):
        latest_tree_cloud = cur_synced_tree_and_ground_clouds[0]
        cur_ground_points = np.array(list(pc2.read_points(
            cur_synced_tree_and_ground_clouds[1], skip_nans=True, field_names=("x", "y", "z"))))
        original_cloud = latest_tree_cloud
        # extract xyz from the cloud
        cloud_points = np.array(list(pc2.read_points(
            original_cloud, skip_nans=True, field_names=("x", "y", "z"))))
        # check if tree cloud points is more than a given threshold
        # TODO(ankit): remove hardcoded value for minimum tree points in cloud
        if cloud_points.shape[0] > 40: # 40 is the minimum number of points in the tree cloud
            projected_points = cloud_points
            valid_indices = np.ones(cloud_points.shape[0], dtype=bool)
            tree_cloud_header = original_cloud.header
        else:
            rospy.logwarn_throttle(
                3, "Not enough points in the tree cloud. Skipping this cloud...")
            return

        rospy.loginfo_throttle(5, "Clustering tree instance cloud")
        db = DBSCAN(eps=self.clus_eps, min_samples=self.clus_min_samples).fit(
            projected_points)
        labels = db.labels_
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        # extract the points in each cluster
        tree_instance_cloud = []
        for i in range(n_clusters_):
            # index the points in the original cloud (i.e. cloud_points)
            # that belong to the i-th cluster
            tree_instance_cloud.append(
                cloud_points[valid_indices, :][labels == i])

        # publish tree instance cloud one by one
        # full_data record all instances, each instance has different intensity
        full_data = None
        # init header
        tree_instance_cloud_msg = PointCloud2()
        # same frame_id and same timestamp as the tree cloud
        tree_instance_cloud_msg.header.frame_id = tree_cloud_header.frame_id
        tree_instance_cloud_msg.header.stamp = tree_cloud_header.stamp
        cylinders = []

        for i in range(n_clusters_):
            # check if tree_instance_cloud[i] is None
            if tree_instance_cloud is None:
                rospy.logwarn_throttle(
                    2, "Tree instance cloud is None, skipping this cloud...")
                continue

            # fit a cylinder to the points
            radius, ray, root = self.fit_cylinder(
                tree_instance_cloud[i], cur_ground_points, tree_cloud_header, i)

            if radius is not None:
                cylinders.append([radius, ray, root])

            if full_data is None:
                full_data = np.hstack(
                    (tree_instance_cloud[i], np.ones((tree_instance_cloud[i].shape[0], 1)) * i)).astype(np.float32)
            else:
                full_data = np.vstack((full_data, np.hstack(
                    (tree_instance_cloud[i], np.ones((tree_instance_cloud[i].shape[0], 1)) * i)).astype(np.float32)))

            rospy.loginfo_throttle(
                5, "Successfully published tree instance cloud")

        # visualize the cylinders in rviz
        if len(cylinders) > 0:
            self.visualize_cylinders(cylinders, tree_cloud_header)
            # create ROSCylinderArray message
            ros_cylinder_array = ROSCylinderArray()
            # fill in the ROSCylinder message and publish it
            # fill in the cylinders
            temp_id = 0
            for cylinder in cylinders:
                # cylinder factor should have
                # float32[3] root
                # float32[3] ray
                # float64[] radii
                # float32 radius
                # int64 id
                ros_cylinder = ROSCylinder()
                ros_cylinder.root = cylinder[2]
                ros_cylinder.ray = cylinder[1]
                ros_cylinder.radius = cylinder[0]
                ros_cylinder.id = temp_id  # TODO: do the tracking to globally assign ID!!!!!!!!!!!!!
                temp_id += 1
                # append to the ROSCylinder message
                ros_cylinder_array.cylinders.append(ros_cylinder)

            # fill in the header
            ros_cylinder_array.header = tree_cloud_header
            # publish
            self.sync_meas_pub.publish(ros_cylinder_array)

        if full_data is not None:
            # init width and height
            tree_instance_cloud_msg.width = full_data.shape[0]
            tree_instance_cloud_msg.height = 1

            # init fields
            tree_instance_cloud_msg.fields = self.pc_fields_

            # point step is 16
            tree_instance_cloud_msg.point_step = 16

            # row step is point step * width
            tree_instance_cloud_msg.row_step = tree_instance_cloud_msg.point_step * \
                tree_instance_cloud_msg.width

            # init data
            tree_instance_cloud_msg.data = full_data.tobytes()

            # publish
            self.tree_instance_pub.publish(tree_instance_cloud_msg)

    def visualize_cylinders(self, cylinders, header):
        # first delete all markers
        marker_array = MarkerArray()
        for i in range(100):
            marker = Marker()
            marker.header.frame_id = header.frame_id
            marker.header.stamp = header.stamp
            marker.ns = "cylinder"
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
        self.cylinder_marker_pub.publish(marker_array)

        # create marker array
        marker_array = MarkerArray()
        # enumerate all cylinders
        for i, cylinder in enumerate(cylinders):
            axis = cylinder[1]
            # normalize the axis
            axis = axis / np.linalg.norm(axis)
            radius = cylinder[0]
            root = cylinder[2]
            # create a cylinder marker
            marker = Marker()
            marker.header.frame_id = header.frame_id
            marker.header.stamp = header.stamp
            marker.ns = "cylinder"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            # normalized x,y,z displacement to move from root to center of the cylinder
            z_scale = 10
            half_scale = 0.5 * z_scale
            z_shift = half_scale * axis[2]
            x_shift = half_scale * axis[0]
            y_shift = half_scale * axis[1]
            marker.pose.position.x = root[0] + x_shift
            marker.pose.position.y = root[1] + y_shift
            marker.pose.position.z = root[2] + z_shift

            # get a rotation that rotates the z axis to the axis
            rotation_axis, rotation_angle = self.compute_rotation(
                axis, np.array([0, 0, 1]))
            rotation = R.from_rotvec(rotation_angle * rotation_axis)

            # invert the rotation
            rotation = rotation.inv()
            # convert to quaternion
            quaternion = rotation.as_quat()
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]

            marker.scale.x = radius * 2
            marker.scale.y = radius * 2
            marker.scale.z = z_scale * 0.65

            marker.color.a = 0.9
            # colored by i
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            # append to marker array
            marker_array.markers.append(marker)
        # publish
        self.cylinder_marker_pub.publish(marker_array)

    def normalize(self, v):
        """Normalize a vector."""
        return v / np.linalg.norm(v)

    def compute_rotation(self, vec_1, vec_2):
        """Compute rotation axis and angle between two vectors."""
        # Ensure A and B are normalized
        A = self.normalize(vec_1)
        B = self.normalize(vec_2)

        # Compute the rotation axis
        rotation_axis = np.cross(A, B)
        rotation_axis = self.normalize(rotation_axis)

        # Compute the rotation angle
        rotation_angle = np.arccos(np.dot(A, B))
        return rotation_axis, rotation_angle

    def fit_cylinder(self, points, cur_ground_points, tree_cloud_header, cylinder_id):

        if points.shape[0] > self.min_points_per_tree:
            # get the centroid (median) of points
            centroid = np.median(points, axis=0)
            # search the latest point ground cloud, crop a patch of size self.ground_plane_patch_size x self.ground_plane_patch_size around the centroid
            dist_xy = np.linalg.norm(
                cur_ground_points[:, :2] - centroid[:2], axis=1)
            valid_points_idx = dist_xy < self.ground_plane_patch_size / 2
            local_ground_points = cur_ground_points[valid_points_idx]

            # only do ground fitting if there are enough points
            if local_ground_points.shape[0] > self.min_points_per_ground_patch:
                # fit a plane model to the local ground points
                ground_plane_coeff = self.ground_fitting(
                    local_ground_points, ransac_n_points=5)
            else:
                rospy.logwarn_throttle(7, "Not enough points for ground plane fitting. Setting ground plane to None. Current ground points: %d, Required ground points: %d",
                                       local_ground_points.shape[0], self.min_points_per_ground_patch)
                ground_plane_coeff = None

            if ground_plane_coeff is None and len(self.latest_ground_plane_models) > 1:
                # search the existing ground plane models, if there is one within self.ground_path_size * 2, take it as the ground plane model
                rospy.loginfo_throttle(
                    5, "No local ground plane model found, searching the closeby ground plane models...")
                # create an array from latest_ground_plane_models
                latest_ground_plane_models_centroids_array = np.array(
                    self.latest_ground_plane_models_centroids)
                # get the distance to the centroid
                distance_to_centroid = np.linalg.norm(
                    latest_ground_plane_models_centroids_array - centroid, axis=1)
                # find the min distance, if it is less than self.ground_plane_patch_size * 2, take it as the ground plane model
                assert len(self.latest_ground_plane_models_centroids) == len(
                    self.latest_ground_plane_models)
                if np.min(distance_to_centroid) < self.ground_plane_patch_size * 2:
                    ground_plane_coeff = self.latest_ground_plane_models[np.argmin(
                        distance_to_centroid)]
                else:
                    rospy.logwarn_throttle(
                        7, "No closeby ground plane model found, discarding the current axis ray.")
                    return None, None, None

            elif ground_plane_coeff is not None:
                # visualize the ground plane
                ground_centroid = np.median(local_ground_points, axis=0)
                self.visualize_ground_plane(
                    ground_plane_coeff, ground_centroid, tree_cloud_header, cylinder_id)
                # add this to the latest ground plane models
                self.latest_ground_plane_models.append(ground_plane_coeff)
                self.latest_ground_plane_models_centroids.append(
                    ground_centroid)
                if len(self.latest_ground_plane_models) > self.num_ground_plane_model_to_keep:
                   # pop the oldest ground plane model
                    self.latest_ground_plane_models.pop(0)
                    self.latest_ground_plane_models_centroids.pop(0)
            else:
                # print in red color
                rospy.logwarn_throttle(
                    5, "No ground plane model detected, and no existing ground plane models.")
                return None, None, None

            # fit a line to the points, which is the axis ray
            axis = self.fit_line(points, ground_plane_coeff)
            if axis is None:
                return None, None, None

            # get the radius
            a, b, c, d = ground_plane_coeff
            distance_to_ground = np.abs(np.dot(points, np.array(
                [a, b, c])) + d) / np.sqrt(a ** 2 + b ** 2 + c ** 2)
            # pick the points that have distance to the ground plane close to diameter_measure_height_above_ground
            valid_indices = np.abs(
                distance_to_ground - self.diameter_measure_height_above_ground) < 0.3
            diameter_points = points[valid_indices, :]

            if diameter_points.shape[0] < self.min_points_for_radius:
                rospy.logwarn_throttle(5, "Not enough points to calculate the radius. Current points: {}, Minimum required points: {}".format(
                    diameter_points.shape[0], self.min_points_for_radius))
                return None, None, None

            else:
                # get the diameter, which is the largest distance between two points in diameter_points
                # get the distance matrix
                distance_matrix = np.linalg.norm(
                    diameter_points[:, None, :] - diameter_points[None, :, :], axis=-1)
                # get the largest distance
                diameter = np.max(distance_matrix)
                # centroid of the diameter points
                representative_point = np.mean(diameter_points, axis=0)

            min_diameter = self.radius_cutoff[0] * 2
            max_diameter = self.radius_cutoff[1] * 2

            if diameter < min_diameter or diameter > max_diameter:
                rospy.logwarn_throttle(3, "Diameter out of range ({}). Setting as default. Min diameter: {}, Max diameter: {}".format(
                    diameter, min_diameter, max_diameter))
                radius = self.default_radius
            else:
                radius = 0.5 * diameter

            # get the root which is the intersection of the axis ray and the ground plane
            # intropolate the point along axis to the ground plane to get the root
            root_z = (
                d - a * representative_point[0] - b * representative_point[1]) / c
            root = representative_point - axis * root_z
            return radius, axis, root
        else:
            rospy.logwarn_throttle(
                3, "Not enough points to fit a cylinder. Current points: %d, Minimum required points: %d", points.shape[0], self.min_points_per_tree)
            return None, None, None

    def fit_line(self, points, ground_plane_coeff):
        # fit a line to the points
        # return the axis ray
        # calculate the centroid of the points
        centroid = np.median(points, axis=0)
        # subtract the centroid from the points
        centered_points = points - centroid
        # calculate the covariance matrix
        covariance_matrix = np.dot(centered_points.T, centered_points)
        # calculate the eigenvalues and eigenvectors of the covariance matrix
        eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
        # the eigenvector with the largest eigenvalue is the axis ray
        axis = eigenvectors[:, np.argmax(eigenvalues)]
        a, b, c, d = ground_plane_coeff
        ground_plane_axis = np.array([a, b, c])
        # if the angle between axis ray and ground plane axis is more than discard the axis ray
        angle = np.arccos(np.dot(axis, ground_plane_axis))
        if angle > self.angle_cutoff:
            rospy.logwarn_throttle(
                3, "Axis ray is more than threshold from ground plane axis, discarding the axis ray")
            if angle * 2 < self.angle_cutoff:
                # make axis lie in the middle of axis and ground plane axis
                axis = (axis + ground_plane_axis) / 2
                # normalize the axis
                axis = axis / np.linalg.norm(axis)
            return None
        else:
            return axis

    def make_fields(self):
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

    # TODO(ankit): Delete this after testing
    # def project_to_ground_plane(self, points, ground_plane_coeff):
    #     # project all points in the cloud to the ground plane
    #     # return the projected cloud

    #     a, b, c, d = ground_plane_coeff
    #     distance_to_ground = np.abs(np.dot(points, np.array([a, b, c])) + d) / np.sqrt(a ** 2 + b ** 2 + c ** 2)

    #     # Normal vector of the plane
    #     n = np.array([a, b, c])

    #     # Compute a point P0 on the plane
    #     # Here we assume z=0 to find P0. If c is zero, you might want to use another coordinate.
    #     P0 = np.array([-d/a, 0, 0]) if a != 0 else np.array([0, -d/b, 0])

    #     # Compute the projections of points onto the plane
    #     dot_product = np.dot(points - P0, n)
    #     projected_points = points - np.outer(dot_product / np.dot(n, n), n)

    #     # only keep distances larger than self.min_distance_to_ground
    #     valid_indices = distance_to_ground > self.min_distance_to_ground
    #     projected_points = projected_points[valid_indices, :]

    #     if projected_points.shape[0] > 20:
    #       return projected_points, valid_indices
    #     else:
    #       print("too few points left after projecting to the ground plane and throwing away points too close to the ground")
    #       return None, None
    #     # else:
    #     #     print("ground plane model not initialized")
    #     #     return None, None

    def ground_fitting(self, points, ransac_n_points=5):

        ground_pcd = o3d.geometry.PointCloud()
        ground_pcd.points = o3d.utility.Vector3dVector(points)
        plane_eq, _ = ground_pcd.segment_plane(
            distance_threshold=0.1, ransac_n=ransac_n_points, num_iterations=100)
        [a, b, c, d] = plane_eq
        ground_plane_coeff = np.array([a, b, c, d])

        return ground_plane_coeff

    def visualize_ground_plane(self, ground_plane_coeff, ground_centroid, header, id):

        markerarray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = header.frame_id
        marker.header.stamp = header.stamp
        marker.ns = "ground_plane"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = ground_centroid[0]
        marker.pose.position.y = ground_centroid[1]
        marker.pose.position.z = ground_centroid[2]
        # get a rotation that rotates the z axis to the ground plane normal
        rotation_axis, rotation_angle = self.compute_rotation(
            ground_plane_coeff[:3], np.array([0, 0, 1]))
        rotation = R.from_rotvec(rotation_angle * rotation_axis)
        # invert the rotation
        rotation = rotation.inv()
        # convert to quaternion
        quaternion = rotation.as_quat()
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = self.ground_plane_patch_size * 0.5
        marker.scale.y = self.ground_plane_patch_size * 0.5
        marker.scale.z = 0.01
        marker.color.a = 0.4
        # color the ground plane in ochre
        marker.color.r = 0.8
        marker.color.g = 0.4
        marker.color.b = 0.1
        markerarray.markers.append(marker)
        self.ground_plane_marker_pub.publish(markerarray)


if __name__ == "__main__":

    ap = argparse.ArgumentParser()

    # add indoor argument
    ap.add_argument("--point_cloud_ns", type=str, default="",
                    help="point_cloud_ns namespace")

    args = vars(ap.parse_args(rospy.myargv()[1:]))

    rospy.init_node("cylidner_plane_modeller")
    cylidner_plane_modeller = CylinderPlaneModeller(args)
    rospy.spin()
