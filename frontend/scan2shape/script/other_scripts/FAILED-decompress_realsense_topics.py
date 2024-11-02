#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class DepthImageUncompressor:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribe to the compressed depth image topic
        self.sub = rospy.Subscriber("/camera/depth/image_rect_raw/compressed", CompressedImage, self.callback)

        # Publisher for the uncompressed depth image
        self.pub = rospy.Publisher("/camera/depth/image_rect_raw", Image, queue_size=10)

    def callback(self, data):
        try:
            # Convert the compressed image to a CV2 image with the specified encoding
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="rgb8")
            
            # # Assuming all channels of RGB have the same value, take one channel and upscale to 16-bit
            # depth_img_16 = (cv2_img[:,:,0] * 256).astype('uint16')

            # # Convert the 16-bit depth image to a ROS image and publish
            # img_msg = self.bridge.cv2_to_imgmsg(depth_img_16, "16UC1")
            # self.pub.publish(img_msg)

            # # Convert RGB to BGR
            # cv2_img_bgr = cv2_img_rgb[...,::-1]

            # # Convert the BGR image to a ROS image and publish
            # img_msg = self.bridge.cv2_to_imgmsg(cv2_img_bgr, "bgr8")

            # Convert the CV2 depth image back to a ROS image and publish

            # multiple cv2_image with 1000 to get depth in mm
            img_msg = self.bridge.cv2_to_imgmsg(cv2_img, "rgb8") 
            self.pub.publish(img_msg)
            
        except CvBridgeError as e:
            rospy.logerr(e)


# bridge = CvBridge()
# depth_image = bridge.compressed_imgmsg_to_cv2(res)
# res.format  %yields: '16UC1; jpeg compressed '
# depth_image.dtype   %yields: dtype('uint8')
# depth_image = CvBridge().compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

if __name__ == "__main__":
    rospy.init_node('depth_image_uncompressor', anonymous=True)
    ic = DepthImageUncompressor()
    rospy.spin()
