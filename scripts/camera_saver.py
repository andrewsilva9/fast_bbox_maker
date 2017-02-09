#!/usr/bin/env python
# This file is responsible for bridging ROS to the FaceDetector class (built with Caffe)

from __future__ import division

import sys

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CameraSaver(object):
    """
    This class takes in image data and finds / annotates faces within the image
    """

    def __init__(self):
        rospy.init_node('camera_saver_node')
        self.bridge = CvBridge()
        self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/kinect2/qhd/image_color')


    def _parse_image(self, image_msg):
        """
        Take in an image and draw a bounding box within it
        :param image_msg: Image data
        :return: None
        """

        header = image_msg.header

        try:
            image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print e
            return

    def run(self):
        rospy.Subscriber(self.image_sub_topic_name, Image, self._parse_image) # subscribe to sub_image_topic and callback parse
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = CameraSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
