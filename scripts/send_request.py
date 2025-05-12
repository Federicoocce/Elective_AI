#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

def send_request(request_text):
    """Send a request to the mall navigation system"""
    rospy.init_node('request_sender', anonymous=True)
    
    pub = rospy.Publisher('/mall_navigation/mother_request', String, queue_size=10)
    
    # Wait for publisher to connect
    rospy.sleep(1.0)
    
    # Send the request
    pub.publish(request_text)
    rospy.loginfo(f"Sent request: {request_text}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: send_request.py \"request text\"")
        print("Examples:")
        print("  send_request.py \"Take me to Footwear Palace\"")
        print("  send_request.py \"I need red shoes for my son\"")
        print("  send_request.py \"Where can I find a blue dress?\"")
        sys.exit(1)
    
    request_text = " ".join(sys.argv[1:])
    
    try:
        send_request(request_text)
    except rospy.ROSInterruptException:
        pass