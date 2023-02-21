#! /usr/bin/env python3

# The imports
import rospy
import time
import glob
import cv2
import torch
from cv2 import rectangle
import numpy as np
from os.path import realpath, dirname, join


# Rospy.loginfo is used to print out the information in the terminal which works better in roslaunch
# modules for tracking model
from net import SiamRPNvot
from run_SiamRPN import SiamRPN_init, SiamRPN_track
from utils import get_axis_aligned_bbox, cxy_wh_2_rect, im_to_numpy, area, in_locking_rect, large_enough, fsm

from sensor_msgs.msg import Image
from shape_msgs.msg import Plane
# from geometry_msgs.msg import
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
import copy

class Detector:

    def __init__(self):

        rospy.loginfo("Detection class initiated")

    def init_detector(self):

        # device for the detection
        self.device = torch.device(
            "cuda:0" if torch.cuda.is_available() else "cpu")
        rospy.loginfo("Algorithm runs on {}".format(self.device))

        # load net for Tracker
        self.tracking_net = SiamRPNvot()
        self.tracking_net.load_state_dict(torch.load(
            self.real_path_name("SiamRPNVOT.model"), map_location=self.device))
        self.tracking_net.eval().to(self.device)
        
        


        rospy.loginfo("detection model loaded")

        # Opencv DNN for YOLOv4_tiny
        self.detection_net = cv2.dnn.readNet(self.real_path_name("yolov4_tiny_custom_best.weights"),
                                             self.real_path_name("yolov4_tiny_custom_test.cfg"))

        # Create detection model
        self.detection_model = cv2.dnn_DetectionModel(self.detection_net)
        self.detection_model.setInputParams(size=(320, 320), scale=1 / 255)

        self.detection_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.detection_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        # initial states for fsm
        self.states = [False, False, False, False, False]
        # TODO make saving video work
        """
        # Saving video
        # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        # out = cv2.VideoWriter('output.avi', fourcc, 30.0, (1080, 720))
        """

        # Load class lists for detection
        # Relook here is it necessary to look at the classes since there is only one class
        self.classes = []
        with open(self.real_path_name("classes.txt"), "r") as file_object:
            for class_name in file_object.readlines():
                # print(class_name)
                class_name = class_name.strip()  # spaces between lines
                self.classes.append(class_name)

        # Initialize camera/ video
        # self.cap = cv2.VideoCapture("last version.mp4")

        self.cnt = 0  # frame counter
        self.detection_initiated = False  # False until the first detection, then True
        self.object_detected = False
        self.rect = [0, 0, 0, 0]  # bounding box of the object
        # timestamps of the locking and unlocking of the object
        self.locking_timestamps = [0, 0]
        self.start_time = time.time()  # start time of the iteration
        self.cv_image : np.ndarray
    def real_path_name(self, name):
        return join(realpath(dirname(__file__)), name)

    
    def start_detection_thread(self):
        #TODO threading code for detection
    # threading code for detection
        
        thread = Thread(target=self.detection, args=(5, 0.7, 0.35)) 
        thread.daemon = True
        thread.start()




    def detection(self, frame_per_detection: int = 5,confThreshold: float = 0.7, nmsThreshold: float = .35):

        tic = cv2.getTickCount()
        fps = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message("talon/usb_cam/image_raw", Image, timeout=1)
                im = self.cv_image
                isLocked = False
                # Get frames
                self.cnt += 1  # frame counter for detection

                duration_tic = time.time()
                print(duration_tic)
                im_x, im_y = im.shape[1], im.shape[0]
                # print("Current Frame is:{}".format(cnt))


                # number frame to skip detectionq
                if self.cnt % frame_per_detection == 0 or not self.Tracker_is_initilized :
                    tic_detection = cv2.getTickCount()
                    self.detection_initiated = True
                    # Object Detection
                    (class_ids, scores, bboxes) = self.detection_model.detect(
                        im, confThreshold, nmsThreshold)
                    bboxes = list(bboxes)


                    if len(bboxes) > 1:
                        bboxes.sort(key=area)
                        bboxes = bboxes[-1:]
                        print(bboxes)
                    toc_detection = cv2.getTickCount()
                    print("Detection FPS: {}".format(
                        1 / ((toc_detection - tic_detection) / cv2.getTickFrequency())))

                    for class_id, score, bbox in zip(class_ids, scores, bboxes):
                        self.object_detected = True
                        isLocked = True
                        (x, y, w, h) = bbox
                        self.rect = [x, y, x + w, y + h]
                        # cv2.rectangle(im, (x, y), (x + w, y + h), (0,0,255), 3)
                        # what does this do?
                        class_name = self.classes[class_id]

                        cv2.putText(im, class_name + "" + "%.2f" % score, (x, y - 10),
                                    cv2.FONT_HERSHEY_PLAIN, 3, (200, 0, 50), 1)

                        cx, cy = x + w / 2, y + h / 2
                        target_pos, target_sz = np.array(
                            [cx, cy]), np.array([w, h])
                        self.tracker = SiamRPN_init(im, target_pos, target_sz, self.tracking_net,
                                                    self.device)
                        self.Tracker_is_initilized = True

                elif self.Tracker_is_initilized:
                    tic_tracking = cv2.getTickCount()
                    # Track obejct
                    state = SiamRPN_track(self.tracker, im, self.device)
                    isLocked = True
                    # Draw bbox
                    cv2.putText(im, "Tracking Score: {:.2f}".format(state['score']),
                                (10, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 1)
                    # cv2.putText(im, str(state['score']), (20, 40), cv2.FONT_HERSHEY_PLAIN, 3, (0,0,100), 2)
                    res = cxy_wh_2_rect(
                        state['target_pos'], state['target_sz'])
                    res = [int(l) for l in res]
                    self.rect = [res[0], res[1], res[0] + res[2], res[1] + res[3]]
                    # cv2.rectangle(im, (res[0], res[1]), (res[0] + res[2], res[1] + res[3]), (0, 255, 255), 3)
                    if state['score'] < 0.5:
                        self.Tracker_is_initilized = False
                        # self.object_detected = False
                        # isLocked = False
                    toc_tracking = cv2.getTickCount()
                    print("Tracking FPS: {}".format(
                        1 / ((toc_tracking - tic_tracking) / cv2.getTickFrequency())))

                toc = cv2.getTickCount()
                isLocked_server = isLocked and large_enough(
                    self.rect[2] - self.rect[0], self.rect[3] - self.rect[1], im_x, im_y) and in_locking_rect(
                        self.rect, im_x, im_y)
                # isLocked_server = isLocked

                # Display frame

                cv2.putText(im, "FPS: %.2f" % (1 / ((toc - tic) / cv2.getTickFrequency())),
                            (10, 70), cv2.FONT_HERSHEY_PLAIN, 2, (200, 0, 50), 2)
                cv2.putText(im, "Frame: {}".format(self.cnt), (900, 40), cv2.FONT_HERSHEY_PLAIN,
                            3, (200, 0, 50), 2)
                
                tic = cv2.getTickCount()
                isLocked_server, self.states = fsm(isLocked_server, self.states)
                print("states: ", self.states)

                if isLocked:
                    cv2.rectangle(im, (self.rect[0], self.rect[1]), (self.rect[2], self.rect[3]), (0, 0, 255),
                                  3)
                    cv2.putText(im, "Locked", (450, 40), cv2.FONT_HERSHEY_PLAIN, 3,
                                (0, 0, 255), 3)
                    print("area: ", (self.rect[2] - self.rect[0])
                          * (self.rect[3] - self.rect[1]))
                else:
                    cv2.putText(im, "NOTLocked", (450, 40), cv2.FONT_HERSHEY_PLAIN, 3,
                                (5, 110, 5), 1)
                # if isLocked_server:
                if isLocked_server:
                    if self.locking_timestamps == [0, 0]:
                        self.locking_timestamps[0] = duration_tic    # type: ignore 
                        print("Locking started at: ", self.locking_timestamps[0])
     
                    else:
                        cv2.putText(
                            im, "Locking time: {:.2f}".format(duration_tic -
                                                              self.locking_timestamps[0]),
                            (10, 100), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 3)
                    cv2.putText(im, "Locked_for_server", (450, 80), cv2.FONT_HERSHEY_PLAIN,
                                2, (0, 0, 255), 3)
                    cv2.rectangle(im, (self.rect[0], self.rect[1]), (self.rect[2], self.rect[3]), (0, 0, 255),
                                  3)
                    print("Locked and the bounding box is large enough")
                    print("Hedef_merkez_X: {}".format(
                        (self.rect[0] + self.rect[2]) / 2))
                    print("Hedef_merkez_Y: {}".format(
                        (self.rect[1] + self.rect[3]) / 2))
                    print("Hedef_genislik: {}".format(self.rect[2] - self.rect[0]))
                    print("Hedef_yukseklik: {}".format(
                        self.rect[3] - self.rect[1]))
                else:
                
                    if not self.locking_timestamps == [0, 0]:
                        self.locking_timestamps[1] = duration_tic #type: ignore
                        """
                        print("Locking finished")
                        print("Locking_duration: {}".format(locking_timestamps[1]-locking_timestamps[0]))
                        """
                        if self.locking_timestamps[1] - self.locking_timestamps[0] > 5:
                            print("Locking finished at {}".format(duration_tic) +
                                  " with duraiton more than 5 seconds")
                            print("Locking_duration: {}".format(self.locking_timestamps[1] -
                                                                self.locking_timestamps[0]))
                        else:
                            # this wont be published to the server, just for the user to see
                            print("Locking finished at {}".format(duration_tic) +
                                  " with duraiton less than 5 seconds")
                            print("Locking_duration: {}".format(self.locking_timestamps[1] -
                                                                self.locking_timestamps[0]))
                        self.locking_timestamps = [0, 0]


                # exp
                cv2.putText(im, "states: {}".format(self.states), (10, 180),
                            cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 1)

                # Locking rectangle
                cv2.rectangle(im, (im_x // 4, im_y // 10),
                              (im_x * 3 // 4, im_y * 9 // 10), (130, 0, 75), 3)
                # clock
                cv2.putText(im, "Clock: {:.2f}".format(duration_tic - self.start_time),
                            (10, 140), cv2.FONT_HERSHEY_PLAIN, 2, (150, 250, 250), 2)

                # resize for display
                # frame = cv2.resize(im, (1600, 900))
                cv2.imshow("Frame", im)

                print("Frame: {}".format(self.cnt))
                # print("------------")
            
                
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                else:
                    fps.sleep()

            except Exception as e:
                print(e)