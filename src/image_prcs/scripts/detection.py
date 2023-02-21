#! /usr/bin/env python3
#
# This is the image processing node for the comet simulation.


import rospy
import cv2
import numpy as np
import time

import threading  # for threading

# modules for tracking model

from sensor_msgs.msg import Image
from shape_msgs.msg import Plane
# from geometry_msgs.msg import
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from DetectionClass import Detector

#Make it work only on tracking mode 


class ImageProcessor(Detector):

    def __init__(self):

        # select the publishers and subscribers

        self.locking_rect_pub = rospy.Publisher(
            'locking_rect', Plane, queue_size=10)
        self.locking_state_pub = rospy.Publisher(
            'locking_state', Bool, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "talon/usb_cam/image_raw", Image, self.image_callback)
        rospy.init_node('image_processor', anonymous=True)
        rospy.loginfo("Image processor node started")
    
    def image_callback(self, data):

        # rospy.loginfo("Image received")
        try:
            #Reviev
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # this is the image debug window for cv2 to see if it is taking images correctly
            # cv2.imshow("Image window", self.cv_image)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)



    
if __name__ == '__main__':
    image_processor = ImageProcessor()
    # image_processor.init_image()
    image_processor.init_detector()
    image_processor.start_detection_thread()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
