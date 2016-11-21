#!/usr/bin/env python

import roslib

roslib.load_manifest('drv_user')
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt32

from drv_msgs.srv import *


pubSR = rospy.Publisher('/comm/msg/vision/select_request', UInt32, queue_size=1)
pubInfo = rospy.Publisher('/comm/msg/vision/info', String, queue_size=1)


def handle_user_select(req):
    param_control_user_selected = '/comm/param/control/target/selected'
    num = req.select_num
    selected = -1

    info = "Please choose the one you want, if no target, enter 0."
    print info
    info_msg = String()
    info_msg.data = "CHOOSE"
    pubInfo.publish(info_msg)

    # Make sure the last select is clear
    if rospy.has_param(param_control_user_selected):
        rospy.set_param(param_control_user_selected, -1)

    # broadcast the request to select the target
    sr_msg = UInt32()
    sr_msg.data = num
    pubSR.publish(sr_msg)

    while selected < 0:
        if rospy.has_param(param_control_user_selected):
            selected = rospy.get_param(param_control_user_selected)
        if num >= selected >= 0:
            break
        else:
            selected = -1

    rsp = user_selectResponse()
    rsp.selected_id = int(selected)

    rospy.set_param(param_control_user_selected, -1)

    return rsp


def user_select_server():
    rospy.init_node('user_select_server')
    s = rospy.Service('drv_user', user_select, handle_user_select)
    print "Ready to get user selection."
    rospy.spin()


if __name__ == "__main__":
    user_select_server()
