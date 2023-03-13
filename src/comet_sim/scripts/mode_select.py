#!/usr/bin/env python3

# Node written for the mode selection of Astrotech Uav

import rospy
from std_msgs.msg import String
from threading import Thread


class ModeSelect:

    def __init__(self):
        self.pub = rospy.Publisher('mode_select', String, queue_size=10)
        rospy.init_node('mode_select')
        self.input_rate = rospy.Rate(1)  # 1hz rate of mode selection prompt
        self.pub_hz = 10
        self.pub_rate = rospy.Rate(self.pub_hz)  # 10hz rate of mode publishing
        self.mode = String("None")
        rospy.loginfo("Mode select node initialized")

    def mode_select(self):
        while not rospy.is_shutdown():
            modes = ["attctl", "posctl", "takeoff", "land", "loiter", "rtl",
                     "offboard", "disarm", "arm", "stop_tracking", "tracking"]
            mode = input("Enter mode: ").strip()  # strip whitespace

            if mode not in modes:
                rospy.logerr(
                    "Invalid mode: %s /n selected mode must be one of these : ", mode)
                rospy.logerr(modes)
                continue

            self.mode = String(mode)
            rospy.loginfo(
                "Mode selected: %s and publishing with rate %dhz", mode, self.pub_hz)
            self.input_rate.sleep()

    def set_mode(self):

        while not rospy.is_shutdown():
            self.pub.publish(self.mode)
            try:  # prevent garbage in console output when thread is killed
                self.pub_rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def start_pub_thread(self):
        self.pub_thread = Thread(target=self.set_mode, args=())
        self.pub_thread.daemon = True
        self.pub_thread.start()


if __name__ == '__main__':
    modeselect = ModeSelect()
    # modeselect.__init__()
    modeselect.start_pub_thread()
    try:
        modeselect.mode_select()
    except rospy.ROSInterruptException:
        pass
