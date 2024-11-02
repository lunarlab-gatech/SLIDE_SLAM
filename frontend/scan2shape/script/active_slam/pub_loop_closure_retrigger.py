#! /usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt64
import actionlib
from sloam_msgs.msg import ActiveLoopClosureAction, ActiveLoopClosureGoal, ActiveLoopClosureActionGoal
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


key_pose_submap_pairs = {}

def submap_cb(msg):
    # iterate through the markers in the MarkerArray
    global key_pose_submap_pairs
    key_pose_submap_pairs = {}
    for marker in msg.markers:
        if "delete" in marker.ns:
            return
        if "key_pose" in marker.ns:
            # the last one in the ns is the id
            key_pose_id = marker.id
            if key_pose_id not in key_pose_submap_pairs:
                key_pose_submap_pairs[key_pose_id] = {}
            key_pose_submap_pairs[key_pose_id]["key_pose"] = np.array([marker.points[0].x, marker.points[0].y, marker.points[0].z])
        elif "submap_landmarks" in marker.ns:
            # the last one in the ns is the id
            key_pose_id = int(marker.ns.split("_")[-1])
            if key_pose_id not in key_pose_submap_pairs:
                key_pose_submap_pairs[key_pose_id] = {}
            if "submap_landmarks" not in key_pose_submap_pairs[key_pose_id]:
                key_pose_submap_pairs[key_pose_id]["submap_landmarks"] = []
            key_pose_submap_pairs[key_pose_id]["submap_landmarks"].append(np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]))
        elif "submap_centroid_" in marker.ns:
            # the last one in the ns is the id
            key_pose_id = int(marker.ns.split("_")[-1])
            print(key_pose_id)
            key_pose_submap_pairs[key_pose_id]["submap_centroid"] = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
        

    # print all the key_pose_submap_pairs
    for key_pose_id in key_pose_submap_pairs:
        print("key_pose_id: ", key_pose_id)
        print("key_pose: ", key_pose_submap_pairs[key_pose_id]["key_pose"])
        print("submap_centroid: ", key_pose_submap_pairs[key_pose_id]["submap_centroid"])
        print("submap_landmarks: ", key_pose_submap_pairs[key_pose_id]["submap_landmarks"])
        print("")



def bag_goal_cb(msg):
    print("bag_goal_cb")
    print(msg.goal)
    key_pos_idx = msg.goal.key_pose_idx.data
    print("key_pose_idx: ", key_pos_idx)

    # Creates a goal to send to the action server.
    new_goal = ActiveLoopClosureGoal()
    if (key_pos_idx not in key_pose_submap_pairs):
        print("key_pose_idx not in key_pose_submap_pairs")
        print("Take first key pose index in key_pose_submap_pairs")
        key_pos_idx = list(key_pose_submap_pairs.keys())[0]
    print("key_pose_idx: ", key_pos_idx)
    new_goal.key_pose_idx.data = key_pos_idx
    # Fetch the data from submap according to key_pose_idx
    # Loop through key_pose_submap_pairs[key_pose_idx] and compose point message
    for i in range(len(key_pose_submap_pairs[new_goal.key_pose_idx.data]["submap_landmarks"])):
        point_msg = Point()
        point_msg.x = key_pose_submap_pairs[new_goal.key_pose_idx.data]["submap_landmarks"][i][0]
        point_msg.y = key_pose_submap_pairs[new_goal.key_pose_idx.data]["submap_landmarks"][i][1]
        point_msg.z = key_pose_submap_pairs[new_goal.key_pose_idx.data]["submap_landmarks"][i][2]
        new_goal.submap.append(point_msg)
    

    # Sends the goal to the action server.
    client.send_goal(new_goal)

    print("loop_closure trigger published!")
    # Waits for the server to finish performing the action.
    # client.wait_for_result(timeout=rospy.Duration(180.0))


if __name__ == '__main__':

    # loop_closure_pub = rospy.Publisher(
    # "/loop_closure/loop_closure_request_idx", UInt64, queue_size=100)

    rospy.init_node("loop_closure_request_node")
    client = actionlib.SimpleActionClient('/loop_closure/active_loop_closure_server', ActiveLoopClosureAction)
    
    submap_sub = rospy.Subscriber("/factor_graph_atl/loop_closure_submap", MarkerArray,  callback=submap_cb, queue_size=1)
    trigger_sub = rospy.Subscriber("/loop_closure/active_loop_closure_server/bag_goal", ActiveLoopClosureActionGoal, bag_goal_cb)

    print("node started!")
    client.wait_for_server()

    # r = rospy.Rate(0.001)
    while not rospy.is_shutdown():
        print("node started!")
        # sleep for 1 second
        # rospy.sleep(1)

        rospy.spin()
        # r.sleep()

    print("node killed!")
