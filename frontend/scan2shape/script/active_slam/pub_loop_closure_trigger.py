#! /usr/bin/env python3
# title			:
# description	:
# author		:Yuezhan Tao and Xu Liu

import rospy
import actionlib
from sloam_msgs.msg import ActiveLoopClosureAction, ActiveLoopClosureGoal, ActiveLoopClosureActionGoal
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

landmark_positions = [[-0.05, -3.63, 0.21],
                        [0.07, -5.82, 0.25],
                        [0.11, -1.41, 0.32],
                        [2.36, -4.08, 0.15],
                        [2.26, -5.86, 0.22]]


def trigger_closure():
    print("bag_goal_cb")
    # print(msg.goal)
    key_pos_idx = 0
    print("key_pose_idx: ", key_pos_idx)

    # Creates a goal to send to the action server.
    new_goal = ActiveLoopClosureGoal()
    print("key_pose_idx: ", key_pos_idx)
    new_goal.key_pose_idx.data = key_pos_idx
    # Fetch the data from submap according to key_pose_idx
    # Loop through key_pose_submap_pairs[key_pose_idx] and compose point message
    for i in range(len(landmark_positions)):
        point_msg = Point()
        point_msg.x = landmark_positions[i][0]
        point_msg.y = landmark_positions[i][1]
        point_msg.z = landmark_positions[i][2]
        new_goal.submap.append(point_msg)
    

    # Sends the goal to the action server.
    client.send_goal(new_goal)

    print("loop_closure trigger published!")
    # Waits for the server to finish performing the action.
    # client.wait_for_result(timeout=rospy.Duration(180.0))


if __name__ == '__main__':

    # loop_closure_pub = rospy.Publisher(
    # "/loop_closure/loop_closure_request_idx", UInt64, queue_size=100)

    rospy.init_node("pub_loop_closure_trigger_node")
    client = actionlib.SimpleActionClient('/loop_closure/active_loop_closure_server', ActiveLoopClosureAction)
    
    print("node started!")
    print("waiting for server...")
    client.wait_for_server()


    trigger_closure()
    print("closure triggered!")

    # # r = rospy.Rate(0.001)
    # while not rospy.is_shutdown():
    #     print("node started!")
    #     trigger_closure()
    #     # sleep for 1 second
    #     # rospy.sleep(1)
        
    #     rospy.spin()
    #     # r.sleep()

    print("node killed!")
    