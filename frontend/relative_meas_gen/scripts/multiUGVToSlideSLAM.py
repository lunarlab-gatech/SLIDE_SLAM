#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.msg import ModelStates
from sloam_msgs.msg import RelativeInterRobotMeasurement, StampedRvizMarkerArray
from relative_meas_gen import transforms
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
import xml.etree.ElementTree as ET

# This class takes GT robot poses and world structure from the multi-UGV-gazebo-sim and 
# publishes corresponding relative inter-robot measurements and object detections
# for the SlideSLAM backend (sloam)
class MultiUGVPub:
    def __init__(self):
        # Get parameters
        self.publish_for_rviz = rospy.get_param("publish_for_rviz", default=False)

        # Set up rospack
        self.rospack = rospkg.RosPack()

        # Subscribers 
        self.sub_gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

        # Publishers
        self.pub_relative = rospy.Publisher("/relative_inter_robot_meas", RelativeInterRobotMeasurement, queue_size=10)
        self.pub_jackal0_cuboids = rospy.Publisher("/robot0/cuboid_measurements", StampedRvizMarkerArray, queue_size=10)
        self.pub_jackal1_cuboids = rospy.Publisher("/robot1/cuboid_measurements", StampedRvizMarkerArray, queue_size=10)

        # Publishers for RVIZ
        if (self.publish_for_rviz):
            self.pub_jackal0_rviz = rospy.Publisher("/relative_inter_robot_meas/rviz/jackal0_pose", PoseStamped, queue_size=10)
            self.pub_jackal1_rviz = rospy.Publisher("/relative_inter_robot_meas/rviz/jackal1_pose", PoseStamped, queue_size=10)
            self.pub_relative_rviz = rospy.Publisher("/relative_inter_robot_meas/rviz/relative_pose", PoseStamped, queue_size=10)

        # Keep track of messages sent
        self.sequence_number = 0

        # Frequency with which to publish relative inter-robot measurements
        self.publish_rate = 1 # Hz
        self.last_publish_time = rospy.Time.now()

        # Write the GT trajectory files
        with open(self.rospack.get_path('relative_meas_gen') + '/results/robot0trajectoryGT.txt', 'w') as f:
            pass
        with open(self.rospack.get_path('relative_meas_gen') + '/results/robot1trajectoryGT.txt', 'w') as f:
            pass

    # Helper method that publishes a PoseStamped message for RVIZ
    def publish_rviz(self, pose: Pose, frame_id: str, publisher: rospy.Publisher):
        rviz_msg = PoseStamped()
        rviz_msg.header.stamp = rospy.Time.now()
        rviz_msg.header.seq = self.sequence_number
        rviz_msg.header.frame_id = frame_id
        rviz_msg.pose = pose
        publisher.publish(rviz_msg)
    
    # Helper method that creates a Marker message for a cuboid
    def create_marker_for_cuboid(self, model: ET.Element, robot_name: str, robot_pose: Pose) -> Marker:
        cuboid = Marker()
        cuboid.header.stamp = rospy.Time.now()
        cuboid.header.seq = self.sequence_number
        cuboid.header.frame_id = robot_name + "/base_link"
        cuboid.ns = robot_name + "/cuboid"
        cuboid.id = int(model.get('name')[9])
        cuboid.type = Marker.CUBE
        cuboid.action = Marker.ADD

        # Extract the pose of the cuboid
        pose_vals = model.find('pose').text.strip().split(' ')
        pose = Pose()
        pose.position.x = float(pose_vals[0])
        pose.position.y = float(pose_vals[1])
        pose.position.z = float(pose_vals[2])
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        if float(pose_vals[3]) != 0 or float(pose_vals[4]) != 0 or float(pose_vals[5]) != 0:
            rospy.logerr_throttle(10, "Cuboid rotation matrix is not Identity! Program functionality for this!")
            return cuboid

        # Transform pose into the relative frame of the robots
        cuboid.pose = transforms.calculate_relative_pose(robot_pose, pose)

        # Extract the scale of the cuboid
        scale_vals = model.find('link').find('collision').find('geometry').find('box').find('size').text.strip().split(' ')
        cuboid.scale.x = float(scale_vals[0])
        cuboid.scale.y = float(scale_vals[1])
        cuboid.scale.z = float(scale_vals[2])

        # Set other values
        cuboid.lifetime = rospy.Duration(0) # 0 means forever
        cuboid.frame_locked = False # Should stay in relative position for this frame
        return cuboid

    # This method calculates a relative inter-robot measurement
    # from the ground truth poses of two robots. It also publishes
    # the relative positions of the cuboids relative to the robots.
    def gazebo_callback(self, msg: ModelStates):
        # Make sure the clock is publishing before we start
        while rospy.Time.now() == 0.0:
            rospy.loginfo_throttle(10, "Waiting for clock to start... ModelStates msg dropped...")
            return

        # Get the index of jackal0 and jackal1 robots
        try:
            jackal0_idx = msg.name.index('jackal0')
            jackal1_idx = msg.name.index('jackal1')
        except ValueError:
            rospy.logerr_throttle(10, "Could not find jackal0 or jackal1 in the ModelStates message")
            return
        
        # Get the pose of both robots
        jackal0_pose: Pose = msg.pose[jackal0_idx]
        jackal1_pose: Pose = msg.pose[jackal1_idx]

        # Calculate the relative measurement from jackal0 to jackal1
        relative_meas: Pose = transforms.calculate_relative_pose(jackal0_pose, jackal1_pose)

        # Create the header for the messages
        header = Header()
        header.stamp = rospy.Time.now()
        header.seq = self.sequence_number
        header.frame_id = "jackal0/base_link"

        # Publish a relative inter-robot measurement
        if (self.last_publish_time + rospy.Duration(1.0/self.publish_rate) < rospy.Time.now()):
            msg = RelativeInterRobotMeasurement()
            msg.header = header
            msg.relativePose = relative_meas
            msg.robotIdObserver = 0
            msg.robotIdObserved = 1
            self.pub_relative.publish(msg)
            self.last_publish_time = rospy.Time.now()

        # Publish for RVIZ
        if(self.publish_for_rviz):
            self.publish_rviz(jackal0_pose, "map", self.pub_jackal0_rviz)
            self.publish_rviz(jackal1_pose, "map", self.pub_jackal1_rviz)
            self.publish_rviz(relative_meas, "jackal0/base_link", self.pub_relative_rviz)

        # Increase the sequence number
        self.sequence_number += 1

        # Get path to .world file
        world_file = self.rospack.get_path('relative_meas_gen') + '/worlds/stuff.world'

        # Parse the XML file
        tree = ET.parse(world_file)
        root = tree.getroot()
        world = root.find('world')

        # Create the MarkerArrays for jackal0 and jackal1
        jackal0_cuboids = MarkerArray()
        jackal1_cuboids = MarkerArray()
        for model in world.findall('model'):
            name = model.get('name')
            if "_box_" in name:
                jackal0_cuboids.markers.append(self.create_marker_for_cuboid(model, "jackal0", jackal0_pose))
                jackal1_cuboids.markers.append(self.create_marker_for_cuboid(model, "jackal1", jackal1_pose))

        # Create the StampedRvizMarkerArray messages
        jackal0_array = StampedRvizMarkerArray()
        jackal0_array.header = header
        jackal0_array.cuboid_rviz_markers = jackal0_cuboids
        jackal1_array = StampedRvizMarkerArray()
        jackal1_array.header = header
        jackal1_array.cuboid_rviz_markers = jackal1_cuboids

        # Publish the cuboid measurements
        self.pub_jackal0_cuboids.publish(jackal0_array)
        self.pub_jackal1_cuboids.publish(jackal1_array)

        # Check the first GT trajectory file to see if its empty
        # Assume the second file is in the same state
        empty = False
        with open(self.rospack.get_path('relative_meas_gen') + '/results/robot0trajectoryGT.txt') as f:
            if not f.read(1):
                empty = True

        # Create the GT trajectory file for jackal0 and jackal1
        with open(self.rospack.get_path('relative_meas_gen') + '/results/robot0trajectoryGT.txt', 'a') as f:
            if not empty:
                f.write('\n')
            f.write(str(header.stamp.to_sec()) + ' ' +
                    str(jackal0_pose.position.x) + ' ' + 
                    str(jackal0_pose.position.y) + ' ' + 
                    str(jackal0_pose.position.z) + ' ' +
                    str(jackal0_pose.orientation.x) + ' ' +
                    str(jackal0_pose.orientation.y) + ' ' +
                    str(jackal0_pose.orientation.z) + ' ' +
                    str(jackal0_pose.orientation.w))
        with open(self.rospack.get_path('relative_meas_gen') + '/results/robot1trajectoryGT.txt', 'a') as f:
            if not empty:
                f.write('\n')
            f.write(str(header.stamp.to_sec()) + ' ' +
                    str(jackal1_pose.position.x) + ' ' + 
                    str(jackal1_pose.position.y) + ' ' + 
                    str(jackal1_pose.position.z) + ' ' +
                    str(jackal1_pose.orientation.x) + ' ' +
                    str(jackal1_pose.orientation.y) + ' ' +
                    str(jackal1_pose.orientation.z) + ' ' +
                    str(jackal1_pose.orientation.w))
                    
if __name__ == '__main__':
    # Initialize the node with given name
    node_name = rospy.get_param("multi_UGV_to_SlideSLAM_node_name", default="multi_UGV_pub")
    rospy.init_node(node_name)
    meas_gen = MultiUGVPub()

    # Run until shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))