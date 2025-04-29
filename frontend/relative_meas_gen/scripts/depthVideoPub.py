#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class DepthVideoPub:
    def __init__(self):
        # Get parameters
        self.video_file = rospy.get_param("video_file", default="/opt/slideslam_docker_ws/src/SLIDE_SLAM/bags/CoPeD/FOREST/FOREST_wanda_depth_k.mp4")
        self.camera_topic = rospy.get_param("camera_topic", default="/wanda/forward/color/image_rect_color/compressed")

        # Subscribers
        self.camera_sub = rospy.Subscriber(self.camera_topic, CompressedImage, self.publish_depth_frame)

        # Publishers
        self.pub_depth = rospy.Publisher("/wanda/forward/color/image_rect_color/depth", Image, queue_size=10)

        # CV Bridge
        self.bridge = CvBridge()

        # Open the video file
        self.video_capture = cv2.VideoCapture(self.video_file)
        print("Ready to publish...")

        # The first frame we recieve should be this
        self.first_frame_num = 124488
        self.first_frame = True

    def publish_depth_frame(self, msg: CompressedImage):
        # If first frame, make sure we haven't missed a frame yet
        if self.first_frame:
            assert msg.header.seq == self.first_frame_num, "First frame number does not match the expected value."
            self.first_frame = False

        # Publish corresponding depth image
        if self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
            # if frame is read correctly ret is True
            if not ret:
                print("Hit last video frame...")
                self.video_capture.release()

            # Create a new depth Image to publish
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            image_message.header.seq = msg.header.seq
            image_message.header.stamp = msg.header.stamp
            image_message.header.frame_id = msg.header.frame_id
            self.pub_depth.publish(image_message)

if __name__ == '__main__':

    # Initialize tnode
    node_name = ("depthVideoPub")
    rospy.init_node(node_name)
    node = DepthVideoPub()

    # Run until shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))