#!/usr/bin/env python

import roslib
roslib.load_manifest('drv_user')
import sys
import rospy
import cv2
from std_msgs.msg import Int64
from std_msgs.msg import UInt16MultiArray

from drv_msgs.msg import target_info
from drv_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError

def handle_user_select(req):
    num = req.select_num

    selected = 0
    if num == 1:
        selected = raw_input("Is the one you want? If so, enter 1, else enter 0: ")
    else:
        selected = raw_input("Which one do you want? If no target, enter 0: ")

    rsp = user_selectResponse()
    rsp.selected_id = int(selected)

    # Stop target publisher
    # rospy.set_param("/status/target", 0)
    return rsp

def user_select_server():
    rospy.init_node('user_select_server')
    s = rospy.Service('drv_user', user_select, handle_user_select)
    print "Ready to get user selection."
    rospy.spin()

if __name__ == "__main__":
    user_select_server()