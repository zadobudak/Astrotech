#!/usr/bin/env python3
from mavros_msgs.msg import ParamValue, AttitudeTarget
from tf.transformations import quaternion_from_euler
from threading import Thread
from std_msgs.msg import Header
from pymavlink import mavutil
from mavros_test_common import MavrosTestCommon
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
import numpy as np
import math
import rospy


class MavrosOffboard(MavrosTestCommon):

    def setUp(self):
        super(MavrosOffboard, self).setUp()

        self.att_setpoint_sub = rospy.Subscriber(
            "att_setpoint", AttitudeTarget, self.get_att)  # no callback for this yet
        self.pos_setpoint_sub = rospy.Subscriber(
            "pos_setpoint", PoseStamped, self.get_pos)  # no callback for this yet

        self.pos = PoseStamped()
        self.radius = 10  # the setpoint parameter for taking setpoints in posctl

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        # self.pos_thread = Thread(target=self.send_pos, args=())
        # self.pos_thread.daemon = True
        # self.pos_thread.start()

        self.att = AttitudeTarget()

        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe

        rospy.loginfo(str(rospy.Time.now()))

    def tearDown(self):
        super(MavrosOffboard, self).tearDown()


    """not sure if there is a better solution for callbacks"""

    def get_att(self, msg: AttitudeTarget):
        self.att = msg

    def get_pos(self, msg: PoseStamped):
        self.pos = msg

    def start_att_thread(self):
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        rospy.loginfo("att_thread" + str(rospy.Time.now()))
        self.att_thread.start()

    def start_pos_thread(self):
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        rospy.loginfo("pos_thread" + str(rospy.Time.now()))
        self.pos_thread.start()

    
   # POSCTL 
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in range(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    def send_att(self):
        rate = rospy.Rate(10)  # Hz
        # self.att.body_rate = Vector3()
        # self.att.header = Header()
        # self.att.header.frame_id = "base_footprint"
        # self.att.orientation = Quaternion(*quaternion_from_euler(0.25, -0.1,
        #                                                          0))
        # self.att.thrust = 0.25
        # self.att.type_mask = 7  # ignore body rate

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    #
    # Test method
    #

    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1 << 2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)

        self.set_arm(True, 5)
        self.set_takeoff(min_pitch=15, altitude=35)
        rospy.sleep(10)
        self.set_mode("OFFBOARD", 5)

        rospy.loginfo("run mission")
        # positions = ( (50, 50, 20), (50, -50, 20), (-50, -50, 20),
        #              (0, 0, 20))
        positions = ((50, 50, 50), (50, -50, 20))

        for i in range(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 60)

        # self.set_mode("AUTO.LAND", 5)
        self.set_land(mode="CURRENT")
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)

    def test_attctl(self):
        """Test offboard attitude control"""
        # boundary to cross
        boundary_x = 200
        boundary_y = 100
        boundary_z = 20

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_takeoff(min_pitch=15, altitude=35)

        self.set_arm(True, 5)
        rospy.sleep(5)
        self.wait_for_landed_state(
            mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, 10, -1)
        self.set_mode("OFFBOARD", 5)

        rospy.loginfo("run mission")
        rospy.loginfo("attempting to cross boundary | x: {0}, y: {1}, z: {2}".
                      format(boundary_x, boundary_y, boundary_z))
        # does it cross expected boundaries in 'timeout' seconds?
        timeout = 90  # (int) seconds
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        crossed = False
        for i in range(timeout * loop_freq):
            if (self.local_position.pose.position.x > boundary_x and
                    self.local_position.pose.position.y > boundary_y and
                    self.local_position.pose.position.z > boundary_z):
                rospy.loginfo("boundary crossed | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                crossed = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(crossed, (
            "took too long to cross boundaries | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   90, 0)
        self.set_arm(False, 5)

        def cmd_takeoff(self, alt, lat, lon, min_pitch=15):
            """Takeoff to a specified altitude"""
            """TODO make the takeoff relative to the curent position"""
            self.set_takeoff(min_pitch=min_pitch, altitude=alt,
                             latitude=lat, longitude=lon)

        def cmd_land(self, mode="CURRENT", lat=0, lon=0):
            """Land"""
            # diffrenct modes for landing are possible
            # change the mode parameter CURRENT, HOME, CUSTOM
            # uses gps coordinates for custom mode
            self.set_land(mode=mode, latitude=lat, longitude=lon)

        def cmd_arm(self, arm=True):
            """Arm or disarm"""
            self.set_arm(arm, 5)


if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)

    mavrosOffboard = MavrosOffboard()
    mavrosOffboard.setUp()
    mavrosOffboard.start_att_thread()
    mavrosOffboard.test_attctl()
    while not rospy.is_shutdown():
        rospy.spin()
