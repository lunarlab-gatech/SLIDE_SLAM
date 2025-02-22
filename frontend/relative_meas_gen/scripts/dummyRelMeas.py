#!/usr/bin/env python

import rospy
from sloam_msgs.msg import RelativeInterRobotMeasurement

class DummyRelativeMeasGen:
    def __init__(self):
        # Publisher to the topic /relative_inter_robot_meas
        self.pub = rospy.Publisher("/relative_inter_robot_meas", RelativeInterRobotMeasurement, queue_size=10)

        # Timer to call self.publish_measurement every 1 second (1 Hz)
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_measurement)

    def publish_measurement(self, event):
        """Function that gets called periodically to publish data."""
        msg = RelativeInterRobotMeasurement()
        self.pub.publish(msg)

if __name__ == '__main__':

    # Initialize the node with given name
    node_name = rospy.get_param("/dummy_rel_node_name", default="dummy_rel_meas")
    rospy.init_node(node_name)

    # Run until shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))