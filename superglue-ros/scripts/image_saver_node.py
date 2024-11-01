#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode:
    def __init__(self):
        rospy.init_node('image_saver_node', anonymous=True)
        rospy.loginfo("Started!")
        # Set up a subscriber to the image topic
        self.image_subscriber = rospy.Subscriber("/realsense/color/image_raw", Image, self.image_callback)

        # Create a CvBridge instance
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting ROS Image to OpenCV image: %s" % str(e))
            return

        # Save the image using OpenCV
        filename = os.path.join('/home','karlym','catkin_ws','src','test.jpg')  # Change the filename as needed
        try:
            cv2.imwrite(filename, cv_image)
            rospy.loginfo("Image saved successfully as %s" % filename)
        except Exception as e:
            rospy.logerr("Error saving image: %s" % str(e))

def main():
    ImageSaverNode()
    rospy.spin()

if __name__ == '__main__':
    main()