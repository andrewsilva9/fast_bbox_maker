#!/usr/bin/env python
# This file is responsible for running a node that saves incoming images in a directory named "JPEGImages" and their
# boxes in a directory named "labels"
###### WARNINGS:
###### Directories must be changed to reflect your own machine (mine are absolute paths and hard coded)
###### And the image_count is set to whatever the last one I used. You probably want to set it to 0.
###### Be sure to set the right object name and ID# so you don't overwrite stuff
from __future__ import division

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

class BoxMaker(object):
    """
    This class takes in image data and finds / annotates boxes
    See warnings above to fix two issues
    """

    def __init__(self):
        rospy.init_node('box_maker')
        self.bridge = CvBridge()
        self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/kinect2/qhd/image_color')
        self.image_count = 0
        # Amount to scale bounding box left up down and right
        self.scaler = 3
        # Object ID in YOLO data
        self.object_id = 000
        self.object_name = 'blank'
        self.last_time = time.time()
        self.delay_between_images = 5


    def _parse_image(self, image_msg):
        """
        Take in an image and draw a bounding box within it
        :param image_msg: Image data
        :return: None
        """
        if (time.time() - self.last_time) < self.delay_between_images:
            return
        header = image_msg.header

        try:
            image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print e
            return


        # Read in image, resize, blur, binarize / threshold, erode, dilate
        # cv2.imshow('unscaled', image_cv)
        # cv2.waitKey(0)
        image_resize = cv2.resize(image_cv, dsize=(960, 540))
        # cv2.imshow('scaled', image_resize)
        # cv2.waitKey(0)
        image_resize = image_resize[100:400, 300:700]
        image_blur = cv2.GaussianBlur(image_resize, ksize=(7, 7), sigmaX=3)
        # cv2.imshow('blurred', image_blur)
        # cv2.waitKey(0)
        ret, thresh = cv2.threshold(image_blur, 127, 255, cv2.THRESH_BINARY)
        # cv2.imshow('binary', thresh)
        # cv2.waitKey(0)
        kernel = np.ones((5, 5), np.uint8)
        opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        # cv2.imshow('open', opened)
        # cv2.waitKey(0)

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

        left -= self.scaler
        top -= self.scaler
        bottom += self.scaler
        right += self.scaler
        # cv2.rectangle(opened, (left, top), (right, bottom), color=(0, 255, 0), thickness=2)
        # cv2.imshow('rect', opened)
        # cv2.waitKey(0)
        # Avoid going out of bounds
        left = max(0, left)
        top = max(0, top)
        bottom = min(bottom, len(image_resize))
        right = min(right, len(image_resize[100]))
        left /= float(len(image_resize[0]))
        top /= float(len(image_resize))
        bottom /= float(len(image_resize))
        right /= float(len(image_resize[0]))
        width = right - left
        height = bottom - top
        data = np.array([[self.object_id, left, top, width, height]]).astype(float)
        image_file_path = '/home/asilva/ws/src/fast_bbox_maker/JPEGImages/'+self.object_name+'{:05d}'.format(self.image_count)+'.jpg'
        text_file_path = '/home/asilva/ws/src/fast_bbox_maker/labels/'+self.object_name+'{:05d}'.format(self.image_count)+'.txt'
        debug_image_file_path = '/home/asilva/ws/src/fast_bbox_maker/debugImages/'+self.object_name+'{:05d}'.format(self.image_count)+'.jpg'
        cv2.imwrite(image_file_path, image_resize)
        outfile = open(text_file_path, 'w')
        np.savetxt(outfile, data, fmt="%i %.8f %.8f %.8f %.8f", newline=' ')
        outfile.close()
        self.image_count+=1
        self.last_time = time.time()
        cv2.rectangle(image_resize, (int(left*len(image_resize[0])), int(top*len(image_resize))),
                      (int((left+width)*len(image_resize[0])), int((top+height)*len(image_resize))), (0, 255, 0), thickness=2)
        # cv2.imshow('rect', image_resize)
        # cv2.waitKey(0)
        cv2.imwrite(debug_image_file_path, image_resize)


    def run(self):
        rospy.Subscriber(self.image_sub_topic_name, Image, self._parse_image, queue_size=2) # subscribe to sub_image_topic and callback parse
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = BoxMaker()
        saver.run()
    except rospy.ROSInterruptException:
        pass
