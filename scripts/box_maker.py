#!/usr/bin/env python
# This file is responsible for running a node that saves incoming images in a directory named "JPEGImages" and their
# boxes in a directory named "labels"

from __future__ import division

import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CameraSaver(object):
    """
    This class takes in image data and finds / annotates faces within the image
    """

    def __init__(self):
        rospy.init_node('box_maker')
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

        # Object ID in yolo data
        object_id = 21
        # number of pixels to scale bounding box by (left, up, down, and right)
        scaler = 3

        # Read in image, resize, blur, binarize / threshold, erode, dilate
        image_resize = cv2.resize(image_cv, dsize=(960, 540))
        image_blur = cv2.GaussianBlur(image_resize, ksize=(7, 7), sigmaX=3)
        ret, thresh = cv2.threshold(image_blur, 127, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # Find lowest left corner, lowest top, highest right, and keep making a detected edge the bottom
        left = len(opened[0])
        top = -1
        bottom = 0
        right = 0

        for index, row in enumerate(opened):
            ones = np.where(row < 125)[0]
            if len(ones) < 1:
                continue
            if top < 0:
                top = index
            if ones[0] < left:
                left = ones[0]
            if ones[len(ones) - 1] > right:
                right = ones[len(ones) - 1]
            bottom = index

        left -= scaler
        top -= scaler
        bottom += scaler
        right += scaler
        left /= float(len(image_resize[0]))
        top /= float(len(image_resize))
        bottom /= float(len(image_resize))
        right /= float(len(image_resize[0]))
        width = right - left
        height = bottom - top
        data = np.array([[object_id, left, top, width, height]]).astype(float)
        np.savetxt('test.txt', data, fmt="%i %.8f %.8f %.8f %.8f", newline=' ')
        # cv2.rectangle(image_resize, (int(left*len(image_resize[0])), int(top*len(image_resize))),
        #               (int((left+width)*len(image_resize[0])), int((top+height)*len(image_resize))), (0, 255, 0), thickness=2)
        # cv2.imshow("test", image_resize)
        # cv2.waitKey(0)

    def run(self):
        rospy.Subscriber(self.image_sub_topic_name, Image, self._parse_image) # subscribe to sub_image_topic and callback parse
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = CameraSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
