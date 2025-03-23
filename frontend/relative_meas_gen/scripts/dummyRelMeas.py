#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import pyproj
import numpy as np
import geometry_msgs.msg
import sensor_msgs.msg
import message_filters
from sloam_msgs.msg import RelativeInterRobotMeasurement

class DummyRelativeMeasGen:
    def __init__(self):
        # Get parameters
        self.observer_frame = rospy.get_param("observing_robot_frame", default="wanda/base")
        self.gps_topic_observer = rospy.get_param("gps_topic_observing_robot", default="/wanda/gps")
        self.gps_topic_observed = rospy.get_param("gps_topic_observed_robot", default="/race1/mavros/global_position/raw/fix")
        self.robot_id_observer = rospy.get_param("robot_id_observer", default=0)
        self.robot_id_observed = rospy.get_param("robot_id_observed", default=1)

        # Subscribers to both GPS topics and both Odometry topics
        sub_observer = message_filters.Subscriber(self.gps_topic_observer, sensor_msgs.msg.NavSatFix)
        sub_observed = message_filters.Subscriber(self.gps_topic_observed, sensor_msgs.msg.NavSatFix)
        self.ts = message_filters.ApproximateTimeSynchronizer([sub_observer, sub_observed], queue_size=100, slop=0.0625)
        self.ts.registerCallback(self.publish_relative_meas)

        # Publisher to the topic /relative_inter_robot_meas
        self.pub = rospy.Publisher("/relative_inter_robot_meas", RelativeInterRobotMeasurement, queue_size=10)

        # Hold recieved messages
        self.observer_msgs = []
        self.observed_msgs = []

        # Keep track of messages sent
        self.sequence_number = 0
    
    # Calculate the relative measurement between two GPS messages,
    # returning a Pose message with no orientation information
    def calculate_relative_measurement(self, msg_observer: sensor_msgs.msg.NavSatFix, 
            msg_observed: sensor_msgs.msg.NavSatFix) -> geometry_msgs.msg.Pose:
        
        def geodetic_to_ecef(lat, lon, alt):
            """ Convert latitude, longitude, and altitude to ECEF coordinates. """
            ecef = pyproj.Proj(proj='ecef')
            geodetic = pyproj.Proj(proj='latlong', datum='WGS84')
            x, y, z = pyproj.transform(geodetic, ecef, lon, lat, alt, radians=False)
            return np.array([x, y, z])

        def ecef_to_enu(ecef_ref, ecef_target, lat_ref, lon_ref):
            """ Convert ECEF displacement to ENU (local XYZ) coordinates. """
            dx, dy, dz = ecef_target - ecef_ref  # Compute displacement vector in ECEF

            # Convert reference lat/lon to radians
            lat_ref, lon_ref = np.radians(lat_ref), np.radians(lon_ref)

            # Compute rotation matrix from ECEF to ENU
            R = np.array([
                [-np.sin(lon_ref), np.cos(lon_ref), 0],
                [-np.sin(lat_ref) * np.cos(lon_ref), -np.sin(lat_ref) * np.sin(lon_ref), np.cos(lat_ref)],
                [np.cos(lat_ref) * np.cos(lon_ref), np.cos(lat_ref) * np.sin(lon_ref), np.sin(lat_ref)]
            ])

            enu = R @ np.array([dx, dy, dz])  # Apply rotation matrix
            return enu  # Returns (ΔX, ΔY, ΔZ)

        def gps_to_xyz(lat1, lon1, alt1, lat2, lon2, alt2):
            """ Compute (X, Y, Z) transformation from GPS1 to GPS2 in meters. """
            ecef_ref = geodetic_to_ecef(lat1, lon1, alt1)
            ecef_target = geodetic_to_ecef(lat2, lon2, alt2)
            return ecef_to_enu(ecef_ref, ecef_target, lat1, lon1)

        xyz_transform = gps_to_xyz(msg_observer.latitude, msg_observer.longitude, msg_observer.altitude, 
                                   msg_observed.latitude, msg_observed.longitude, msg_observed.altitude)

        # Create the Pose message
        msg = geometry_msgs.msg.Pose()
        msg.position = geometry_msgs.msg.Point()
        msg.position.x = xyz_transform[0]
        msg.position.y = xyz_transform[1]
        msg.position.z = xyz_transform[2]
        msg.orientation = geometry_msgs.msg.Quaternion() # No orientation information is provided
        return msg

    def publish_relative_meas(self, observer_msg, observed_msg):        
        # Check both messages to make sure the timestamps are within 0.01 seconds
        observer_msg = self.observer_msgs[0]
        observed_msg = self.observed_msgs[0]
        timestamp_diff = observer_msg.header.stamp.to_sec() - observed_msg.header.stamp.to_sec()
        rospy.logwarn("Timestamps of GPS messages are within %f", timestamp_diff)
        
        # Create a RelativeInterRobotMeasurement message
        msg = RelativeInterRobotMeasurement()
        msg.header.seq = self.sequence_number
        msg.header.stamp = observed_msg.header.stamp
        msg.header.frame_id = self.observer_frame
        msg.relativePose = self.calculate_relative_measurement(observer_msg, observed_msg)
        msg.robotIdObserver = self.robot_id_observer
        msg.robotIdObserved = self.robot_id_observed

        # Publish the message
        self.pub.publish(msg)
        self.sequence_number += 1

if __name__ == '__main__':
    # Initialize the node with given name
    node_name = rospy.get_param("/dummy_rel_node_name", default="dummy_rel_meas")
    rospy.init_node(node_name)
    meas_gen = DummyRelativeMeasGen()

    # Run until shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))