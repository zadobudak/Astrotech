#! /usr/bin/env python3

# The imports 
import rospy

if __name__ == "__main__":
    
    rospy.init_node("test_node", anonymous = True)
    
    rospy.loginfo("The node has been initialized.")
    
    rate = rospy.Rate(10) 
    
    rospy.loginfo(rate.sleep_dur)

    
    while not rospy.is_shutdown():
        
        rospy.loginfo("The node is still running."+ str(rospy.get_time()))
        rate.sleep()
        