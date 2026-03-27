#!/usr/bin/env python3

import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
import rospy
from assignment import hungarian_assignment
from object_tracker import ObjectTrack
from typing import Type, Any

# global variable
track_id = 0

def calculate_cost_matrix(all_objects: list, cur_objects: np.ndarray) -> np.ndarray:
    """ Calculate the cost matrix between all_objects and cur_objects based on Euclidean xy distance of centroids."""

    all_objects_arr: np.ndarray = np.asarray(all_objects)

    # Get number of objects
    n1 = all_objects_arr.shape[0]
    n2 = cur_objects.shape[0]

    # Create the cost matrix
    cost: np.ndarray = np.zeros((n1, n2))
    for i2 in np.arange(n2):
        distances = np.linalg.norm(all_objects_arr[:, :2] - cur_objects[i2, :2], axis=1)
        cost[:, i2] = distances
    return cost

def object_assignment(all_objects: list, cur_objects: np.ndarray, assignment_threshold: float) -> tuple:
    """ Assign current objects to existing tracked objects using Hungarian algorithm based on cost matrix."""

    cost: np.ndarray = calculate_cost_matrix(all_objects, cur_objects)
    match_inds, lost_inds, new_inds = hungarian_assignment(cost, unassigned_cost=assignment_threshold)
    return match_inds, lost_inds, new_inds

# TODO(ankit): Unify all track_objects functions from below
def track_objects_final(process_cloud_node_obj, cur_class_label, cur_objects, cur_objects_raw_points, all_objects, all_tracks, scan_idx, downsample_res=0.3, num_instance_point_lim=50000):

    global track_id
    if scan_idx == 0 or len(all_objects) == 0:
        for object_xylw, cur_object_raw_points in zip(cur_objects, cur_objects_raw_points):
            object_track = ObjectTrack(object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3],
                                       cur_object_raw_points, scan_idx, track_id, downsample_res, num_instance_point_lim, cur_class_label)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array(
                [object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3], object_track.age, cur_class_label]))
        return all_objects, all_tracks

    else:
        # length of all_tracks and all_objects should match, keeping all_objects is just for vectorization to speed up
        if len(all_tracks) != len(all_objects):
            raise Exception(
                "Length of all_tracks and all_objects does not match!!")

        # record all objects that have class label as cur_class_label
        # record the index match from all_objects_cur_class to all_objects
        all_objects_cur_class = []
        all_objects_cur_class_to_original_list_match = {}
        for i, object in enumerate(all_objects):
            if object[5] == cur_class_label:
                all_objects_cur_class.append(object)
                index_in_cur_class = len(all_objects_cur_class) - 1
                all_objects_cur_class_to_original_list_match[index_in_cur_class] = i

        if len(all_objects_cur_class) == 0:
            # everything should be new inds and no match inds
            match_ind_sets = []
            lost_inds = []
            new_inds = np.arange(len(cur_objects))
        else:
            match_ind_sets, lost_inds, new_inds = object_assignment(
                all_objects_cur_class, cur_objects, process_cloud_node_obj.cuboid_assignment_threshold_per_cls_label[cur_class_label])

        # matched tracks
        for match in match_ind_sets:
            i_track, i_cur_match = match
            # original index in all_objects
            i_track_original = all_objects_cur_class_to_original_list_match[i_track]
            object_track = all_tracks[i_track_original]  # all_tracks[i_track]
            x_new = cur_objects[i_cur_match, 0]
            y_new = cur_objects[i_cur_match, 1]
            l_new = cur_objects[i_cur_match, 2]
            w_new = cur_objects[i_cur_match, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_match]

            object_track.update(x_new, y_new, l_new, w_new,
                                cur_object_raw_points_new, scan_idx)

            all_objects[i_track_original] = np.array(
                [object_track.x, object_track.y, object_track.l, object_track.w, object_track.age, object_track.class_label])

        # new tracks
        for i_cur_new in new_inds:
            x_new = cur_objects[i_cur_new, 0]
            y_new = cur_objects[i_cur_new, 1]
            l_new = cur_objects[i_cur_new, 2]
            w_new = cur_objects[i_cur_new, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_new]

            object_track = ObjectTrack(x_new, y_new, l_new, w_new, cur_object_raw_points_new,
                                       scan_idx, track_id, downsample_res, num_instance_point_lim, cur_class_label)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array(
                [x_new, y_new, l_new, w_new, object_track.age, object_track.class_label]))

        return all_objects, all_tracks


def track_objects_indoor(process_cloud_node_obj: Type[Any], cur_class_label: int, 
                         cur_obj_class: str, cur_objects: np.ndarray, all_objects: list, 
                         all_tracks: list, scan_idx: int, cur_objects_raw_points: list,
                         downsample_res: float = 0.3, num_instance_point_lim: int = 50000) -> tuple:

    # Get global track id variable
    global track_id

    # If first scan or no existing objects, initialize tracks
    if scan_idx == 0 or len(all_objects) == 0:
        for object_xylw, cur_object_raw_points in zip(cur_objects, cur_objects_raw_points):

            # Create an object track for this object
            object_track = ObjectTrack(object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3],
                                       cur_object_raw_points, scan_idx, track_id, downsample_res, 
                                       num_instance_point_lim, cur_class_label)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array([object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3], object_track.age, cur_class_label]))
        
        # Return newly initialized objects and tracks
        return all_objects, all_tracks

    # Otherwise, perform assignment and update tracks
    else:
        # Length of all_tracks and all_objects should match
        # NOTE: Keeping all_objects is for vectorization speed-up
        if len(all_tracks) != len(all_objects):
            raise Exception("Length of all_tracks and all_objects does not match!!")

        # Get all objects with the current class id
        all_objects_cur_class: list[np.ndarray] = []
        all_objects_cur_class_to_original_list_match: dict = {}
        for i, object in enumerate(all_objects):
            if object[5] == cur_class_label:
                all_objects_cur_class.append(object)
                index_in_cur_class = len(all_objects_cur_class) - 1
                all_objects_cur_class_to_original_list_match[index_in_cur_class] = i

        # If no existing objects of this class, all current objects are new
        if len(all_objects_cur_class) == 0:
            match_ind_sets, lost_inds = [], []
            new_inds = np.arange(len(cur_objects))

        # Otherwise, perform object assignment
        else:
            match_ind_sets, lost_inds, new_inds = object_assignment(
                all_objects_cur_class, cur_objects, process_cloud_node_obj.class_assignment_thresh[cur_obj_class])

        # For each match, update the corresponding object track
        for match in match_ind_sets:

            # Get indices of matched track and object
            i_track, i_cur_match = match

            # Get original track index in all_objects
            i_track_original = all_objects_cur_class_to_original_list_match[i_track]

            # Get the corresponding object track
            object_track: ObjectTrack = all_tracks[i_track_original]

            # Get corresponding object data (from new detection)
            x_new = cur_objects[i_cur_match, 0]
            y_new = cur_objects[i_cur_match, 1]
            l_new = cur_objects[i_cur_match, 2]
            w_new = cur_objects[i_cur_match, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_match]

            # Update the object track with new data
            object_track.update(x_new, y_new, l_new, w_new, cur_object_raw_points_new, scan_idx)

            # Update the all_objects list with updated track data
            all_objects[i_track_original] = np.array([object_track.x, object_track.y, object_track.l, object_track.w, 
                                                      object_track.age, object_track.class_label])

        # For unassigned new objects, create new tracks
        for i_cur_new in new_inds:

            # Get corresponding object data (from new detection)
            x_new = cur_objects[i_cur_new, 0]
            y_new = cur_objects[i_cur_new, 1]
            l_new = cur_objects[i_cur_new, 2]
            w_new = cur_objects[i_cur_new, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_new]

            # Create a new object track for this new object
            object_track = ObjectTrack(x_new, y_new, l_new, w_new, cur_object_raw_points_new,
                                       scan_idx, track_id, downsample_res, num_instance_point_lim, cur_class_label)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array([x_new, y_new, l_new, w_new, object_track.age, object_track.class_label]))

        # Return updated objects and tracks
        return all_objects, all_tracks


def track_objects(cur_objects, all_objects, all_tracks, scan_idx, cur_objects_raw_points, assignment_threshold=1.0, downsample_res=0.3, num_instance_point_lim=50000):

    global track_id
    if scan_idx == 0 or len(all_objects) == 0:

        for object_xylw, cur_object_raw_points in zip(cur_objects, cur_objects_raw_points):
            object_track = ObjectTrack(object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3],
                                       cur_object_raw_points, scan_idx, track_id, downsample_res, num_instance_point_lim)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(np.array(
                [object_xylw[0], object_xylw[1], object_xylw[2], object_xylw[3], object_track.age]))
        return all_objects, all_tracks

    else:
        # length of all_tracks and all_objects should match, keeping all_objects is just for vectorization to speed up
        if len(all_tracks) != len(all_objects):
            raise Exception(
                "Length of all_tracks and all_objects does not match!!")

        match_ind_sets, lost_inds, new_inds = object_assignment(
            all_objects, cur_objects, assignment_threshold)

        # matched tracks
        for match in match_ind_sets:
            i_track, i_cur_match = match
            object_track = all_tracks[i_track]
            x_new = cur_objects[i_cur_match, 0]
            y_new = cur_objects[i_cur_match, 1]
            l_new = cur_objects[i_cur_match, 2]
            w_new = cur_objects[i_cur_match, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_match]

            object_track.update(x_new, y_new, l_new, w_new,
                                cur_object_raw_points_new, scan_idx)
            all_objects[i_track] = np.array(
                [object_track.x, object_track.y, object_track.l, object_track.w, object_track.age])

        # new tracks
        for i_cur_new in new_inds:
            x_new = cur_objects[i_cur_new, 0]
            y_new = cur_objects[i_cur_new, 1]
            l_new = cur_objects[i_cur_new, 2]
            w_new = cur_objects[i_cur_new, 3]
            cur_object_raw_points_new = cur_objects_raw_points[i_cur_new]

            object_track = ObjectTrack(x_new, y_new, l_new, w_new, cur_object_raw_points_new,
                                       scan_idx, track_id, downsample_res, num_instance_point_lim)
            track_id += 1
            all_tracks.append(object_track)
            all_objects.append(
                np.array([x_new, y_new, l_new, w_new, object_track.age]))

        return all_objects, all_tracks


def publish_markers(process_cloud_node: Type[Any], all_tracks: list, 
                    cur_cls_name: str = None, height: float = 1.0, age_threshold: float = 6, 
                    frame_id: str = "quadrotor/map"):
    """ Publish each object track as a marker in rviz, visualized as a cylinder at the object center."""

    # Create MarkerArray message
    object_markers = MarkerArray()
    object_markers.markers = []

    largest_age: int = 10

    # For each Object Track...
    for i, track in enumerate(all_tracks):

        # Only publish markers for tracks older than age threshold
        if track.age < age_threshold: continue

        # Keep track of largest age
        if track.age > largest_age: largest_age = track.age

        # Create the Marker
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        if cur_cls_name is None: marker.ns = "track_centers"
        else: marker.ns = cur_cls_name

        # Visualize as a cylinder, since this is tracking the object xy center
        marker.id = i
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = track.x
        marker.pose.position.y = track.y
        marker.pose.position.z = height

        # Set marker scale and color
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2*height

        # Set marker orientation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set marker color, making older markers more green
        marker.color.a = 0.5  
        marker.color.r = 0.0
        marker.color.g = max(0.3, track.age / largest_age)
        marker.color.b = 0.0

        # Append marker to MarkerArray
        object_markers.markers.append(marker)

    # Publish the MarkerArray
    process_cloud_node.cuboid_center_marker_pub.publish(object_markers)
