#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import PIL.Image as ImagePIL
import io

class CompressedDepthToDepth:
    def __init__(self):
        # Get parameters
        self.compressed_depth_topic = rospy.get_param("compressed_depth_topic", default="/camera/image_raw/compressedDepth")
        self.uncompressed_depth_topic = rospy.get_param("uncompressed_depth_topic", default="/camera/image_raw/depth")
        print(self.compressed_depth_topic)

        # Subscribers 
        self.sub_gazebo = rospy.Subscriber(self.compressed_depth_topic, CompressedImage, self.compressed_image_callback)

        # Publishers
        # self.pub_relative = rospy.Publisher(self.uncompressed_depth_topic, Image, queue_size=10)

        # Create CVBridge object
        self.bridge = CvBridge()


    def uncompress_jpeg(self, jpeg_data):
        """
        Uncompresses JPEG data and returns a PIL Image object.

        Args:
            jpeg_data: Raw JPEG data as bytes.

        Returns:
            A PIL Image object representing the uncompressed image.
        """
        try:
            image = ImagePIL.open(io.BytesIO(jpeg_data))
            return image
        except Exception as e:
            print(f"Error uncompressing JPEG data: {e}")
            return None

    def compressed_image_callback(self, msg: CompressedImage):
        #print(msg)
        image: ImagePIL = self.uncompress_jpeg(msg.data)

        # try:
        #     cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "passthrough")
        # except CvBridgeError as e:
        #     print(e)
        # else:
        print(image.width)
        print(image.height)
        image.save("/opt/slideslam_docker_ws/src/roman_ros/roman_ros/temp.jpg")
        # for row in cv_image:
        #     for pixel in row:
        #         print(pixel, end="")
        # Scale the image to 0-255
        #cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        #cv2.imshow("Image window", cv_image, ) 
        #cv2.waitKey(1)

                    
if __name__ == '__main__':

    # Initialize tnode
    node_name = ("CompressedDepthToDepth")
    rospy.init_node(node_name)
    node = CompressedDepthToDepth()

    # Run until shutdown
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("Shutting down {}...".format(node_name))