#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def status_callback(msg):
    """Print navigation status updates"""
    rospy.loginfo(f"Mall Navigation Status: {msg.data}")

def listen_for_status():
    """Listen for status updates from mall navigator"""
    rospy.init_node('status_listener', anonymous=True)
    
    rospy.Subscriber('/mall_navigation/status', String, status_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        listen_for_status()
    except rospy.ROSInterruptException:
        pass