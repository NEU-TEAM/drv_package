#!/usr/bin/env python

import roslib
import rospy
import cv2
import process as ps

from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray
from drv_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError

roslib.load_manifest('drv_face')


def handle_face_recognize(req):
    rsp = face_recognizeResponse()

    bridge = CvBridge()
    ro_msg = UInt8MultiArray()
    try:
        for im in req.images_in:
            cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
            # Leave image preprocess to client
            result = ps.process(cv_image)

            name_id_msg = UInt8
            if result[1] > 0.9:
                name_id_msg.data = result[0] + 1  # known face id start with 1
            else:
                name_id_msg.data = 0  # unknown face has id 0
            ro_msg.data.append(name_id_msg)

        # establish a dict to find the name refereed by the id
        rsp.face_label_ids = ro_msg
        return rsp

    except CvBridgeError as e:
        print(e)
        return rsp


def face_recognize_server():
    rospy.init_node('face_recognize_server')
    s = rospy.Service('drv_face', face_recognize, handle_face_recognize)
    print "Ready to recognize."
    rospy.spin()


if __name__ == "__main__":
    face_recognize_server()
