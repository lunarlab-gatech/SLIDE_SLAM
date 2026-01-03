#!/usr/bin/env python3

from typing import Optional
import numpy as np
import rospy
import open3d as o3d

class ObjectTrack(object):
    def __init__(self, x: float, y: float, l: float, w: float, 
                 raw_points: np.ndarray, last_update_scan_idx: int, track_idx: int, 
                 downsample_res: float = 0.3, num_instance_point_lim: int = 50000, 
                 cur_class_label: Optional[int] = None):
        
        # Store center of object
        self.x: float = x
        self.y: float = y
        
        # Store size of object
        self.l: float = l
        self.w: float = w

        self.age: int = 1
        self.pos_update_rate = 0.1

        # Remember idx of last scan when this object was seen and track idx
        self.last_update_scan_idx: int = last_update_scan_idx
        self.track_idx: int = track_idx

        # Create Open3D point cloud
        self.o_pcd = o3d.geometry.PointCloud()

        # Class label of the object being tracked
        self.class_label: Optional[int] = cur_class_label

        # Resolution to perform voxel downsampling for instance point cloud accumulation
        self.downsample_res: float = downsample_res  # in meters, -1 means no downsample

        # Only keep the most recent num_points_limit_per_instance for any instance
        self.num_points_limit_per_instance: int = num_instance_point_lim

        # initial covariance and standard devation
        self.xy_cov = 3 * np.ones((2, 2))

        # N*2 history positions of the object
        self.xy_hist = np.array([[x, y]])

        # Downsample the raw points if requested
        if self.downsample_res > 0:
            self.all_raw_points = self.downsample_point_cloud(raw_points)
        else:
            self.all_raw_points = raw_points

    # update once it's assigned to a new detection, update all properties

    def update(self, x_new, y_new, l_new, w_new, raw_points_new, scan_idx):
        self.xy_hist = np.append(
            self.xy_hist, np.array([[x_new, y_new]]), axis=0)
        self.age = self.age+1

        self.x = self.pos_update_rate * x_new + \
            (1 - self.pos_update_rate) * self.x
        self.y = self.pos_update_rate * y_new + \
            (1 - self.pos_update_rate) * self.y
        self.l = self.pos_update_rate * l_new + \
            (1 - self.pos_update_rate) * self.l
        self.w = self.pos_update_rate * w_new + \
            (1 - self.pos_update_rate) * self.w

        # np.cov requires 2*N instead of N*2 [ref: https://numpy.org/doc/stable/reference/generated/numpy.cov.html]
        self.xy_cov = np.cov(np.transpose(self.xy_hist))

        # self.all_raw_points = np.vstack((self.all_raw_points, raw_points_new))
        if self.downsample_res > 0:
            self.all_raw_points = np.vstack(
                (self.all_raw_points, self.downsample_point_cloud(raw_points_new)))
        else:
            self.all_raw_points = np.vstack(
                (self.all_raw_points, raw_points_new))

        # Only keep the most recent num_points_limit_per_instance points
        if self.all_raw_points.shape[0] > self.num_points_limit_per_instance:
            rospy.loginfo_throttle(
                1, 'Max number of points per instance reached. Limiting the number of points to: %d',  self.num_points_limit_per_instance)
            # take the most recent num_points_limit points
            self.all_raw_points = self.all_raw_points[- self.num_points_limit_per_instance:, :]
        self.last_update_scan_idx = scan_idx

    def downsample_point_cloud(self, points_xyz):
        self.o_pcd.points = o3d.utility.Vector3dVector(points_xyz)
        downsampled_pcd = self.o_pcd.voxel_down_sample(
            voxel_size=self.downsample_res)
        rospy.loginfo_throttle(10, 'Downsampled point cloud from %d to %d points', len(
            points_xyz), len(downsampled_pcd.points))
        return np.asarray(downsampled_pcd.points)
