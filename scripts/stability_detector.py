#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy

import message_filters
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import ClassificationResult

try:
    from rect_projector_msgs.msg import Scored2DBox, Scored2DBoxArray
except:
    rospy.logerr("please install aero-ros-pkg-private")
    exit()

rect_x_list = []
rect_y_list = []

def callbackTimer(event):
    print(rect_x_list)
    print(rect_y_list)
    print(np.var(np.array(rect_x_list)) + np.var(np.array(rect_y_list)))
    print("--------")
    del rect_x_list[:]
    del rect_y_list[:]


def callback(rect_msg):
    for rect in rect_msg.rects:
        rect_x_list.append(rect.x)
        rect_y_list.append(rect.y)

def listener():
    rospy.init_node('stability_detector', anonymous=True)
    rospy.Subscriber("/ssd_donbe_detector/output/rect", RectArray, callback)
    rospy.Timer(rospy.Duration(1), callbackTimer)
    rospy.spin()

if __name__ == '__main__':
    listener()
