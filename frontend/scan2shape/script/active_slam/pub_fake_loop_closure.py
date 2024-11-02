#! /usr/bin/env python3
# title			:
# description	:
# author		:Yuezhan Tao and Xu Liu

import rospy
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry



if __name__ == '__main__':

    odom_pub = rospy.Publisher(
    "/loop_closure/odom", Odometry, queue_size=100)


    rospy.init_node("loop_closure_odom_node")
    r = rospy.Rate(0.00000001)
    while not rospy.is_shutdown():
        print("node started!")
        rospy.sleep(1)

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "/map"
        # odom_msg.pose.pose = msg.pose.pose
        odom_msg.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom_msg)
        print("published!")
        #  rospy.spin()
        r.sleep()
            
    print("node killed!")
