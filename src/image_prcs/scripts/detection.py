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
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError
from DetectionClass import Detector

# Make it work only on tracking mode


class ImageProcessor(Detector):

    def __init__(self):

        # select the publishers and subscribers

        self.locking_rect_pub = rospy.Publisher(
            'locking_rect', Plane, queue_size=10)
        self.locking_state_pub = rospy.Publisher(
            'locking_state', Bool, queue_size=10)
        self.mode_sub = rospy.Subscriber(
            'mode_select', String, self.mode_callback)
        self.detector_initilized = False
        self.mode = String("None")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "talon/usb_cam/image_raw", Image, self.image_callback)
        rospy.init_node('image_processor', anonymous=True)
        rospy.loginfo("Image processor node started")
    
    def mode_callback(self, mode_msg):
        # rospy.loginfo("Mode received")
        self.mode_old = self.mode
        self.mode = mode_msg
        if self.mode_old != self.mode:
            rospy.loginfo("Mode changed to: %s", self.mode.data)

            if self.mode.data == "tracking":
                if not self.detector_initilized:
                    self.init_detector()
                    self.detector_initilized = True
                self.start_detection_thread()
                time.sleep(3)
                self.start_detection_logger_thread()

            elif self.mode.data == "stop_tracking" and self.detecting :
                self.detecting = False
                self.stop_detection_thread()
                self.stop_detection_logger_thread()
        else :
            pass

    def image_callback(self, data):

        # rospy.loginfo("Image received")
        try:
            # Reviev
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # this is the image debug window for cv2 to see if it is taking images correctly
            # cv2.imshow("Image window", self.cv_image)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def detection_logger(self):
        # send self.rect to the plane message and publish it
        # send locking state for server and detection as true or false
        # send the locking timestamp of the detection
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            try:
                if self.isLocked_server:
                    
                    self.locking_state_pub.publish(True)
                    self.locking_rect_pub.publish(Plane(self.rect))
                else:
                    self.locking_state_pub.publish(False)
                    self.locking_rect_pub.publish(Plane(self.rect))

                rate.sleep()
            except Exception as e:
                print(e)
                rate.sleep()
                continue

    def start_detection_logger_thread(self):
        # start the detection logger thread
        self.logger_thread = threading.Thread(target=self.detection_logger)
        self.logger_thread.daemon = True
        self.logger_thread.start()
    def stop_detection_logger_thread(self):
        # stop the detection logger thread
        try:
            self.logger_thread.join(1)
        except Exception as e:
            print(e)
            print("Detection logger thread not running or already stopped")
            pass

if __name__ == '__main__':
    image_processor = ImageProcessor()
    # image_processor.init_image()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
