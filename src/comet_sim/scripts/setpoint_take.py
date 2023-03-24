#!/usr/bin/env python3

# Node written for the rival UAV's setpoints of Astrotech UAV

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from threading import Thread
import numpy as np

class RivalCoordinates:

    def __init__(self):
        self.pub = rospy.Publisher('setpoint_take', Float32MultiArray, queue_size=10)
        rospy.init_node('setpoint_take')
        self.input_rate = rospy.Rate(1)  # 1hz rate of reaching Rival UAV's coordinates
        self.pub_hz = 10
        self.pub_rate = rospy.Rate(self.pub_hz)  # 10hz rate of mode publishing
        self.pastcoordinates = Float32MultiArray()
        self.AllCoordinates = list()
        rospy.loginfo("Setpoint take node initialized")

    def rival_pos_take(self):
        while not rospy.is_shutdown():
            rivalposx, rivalposy, rivalposz = input("Enter the coordinates in terms of x,y,z: ").split()

            self.pastcoordinates.data = [float(rivalposx),float(rivalposy),float(rivalposz)]

            rospy.loginfo("The coordinates are %f, %f , %f , the publishing rate is %dhz", float(rivalposx) , float(rivalposy) , float(rivalposz) , self.pub_hz )

            self.AllCoordinates.append(self.pastcoordinates.data)
            rospy.loginfo(self.AllCoordinates) #Display the position array

            self.input_rate.sleep()

    def rival_pos_pub(self):
        
        while not rospy.is_shutdown():
            self.pub.publish(self.pastcoordinates)
            try:  # prevent garbage in console output when thread is killed
                self.pub_rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def start_pub_thread(self):
        self.pub_thread = Thread(target=self.rival_pos_pub, args=())
        self.pub_thread.daemon = True
        self.pub_thread.start()

if __name__ == '__main__':
    RivalAt = RivalCoordinates()

    RivalAt.start_pub_thread()
    try:
        RivalAt.rival_pos_take()
    except rospy.ROSInterruptException:
        pass

    
