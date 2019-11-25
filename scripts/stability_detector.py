#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy

import message_filters
from jsk_recognition_msgs.msg import Rect, RectArray
from jsk_recognition_msgs.msg import ClassificationResult
from std_msgs.msg import Float32

try:
    from rect_projector_msgs.msg import Scored2DBox, Scored2DBoxArray
except:
    rospy.logerr("please install aero-ros-pkg-private")
    exit()

class Rect2LabeledArray():

    rect_x_list = []
    rect_y_list = []
    rects_x_list = []
    rects_y_list = []
    count = 0

    def __init__(self):
        self.pub = rospy.Publisher("~output/stability", Float32, queue_size=1)
        self.subscribe()
        rospy.Timer(rospy.Duration(0.2), self.callbackTimer)

    def subscribe(self):
        queue_size = rospy.get_param('/kaida/rect/queue', 100)

        sub_box = message_filters.Subscriber(
            '~input_rect', RectArray, queue_size=queue_size)
        sub_class = message_filters.Subscriber(
            '~input_class', ClassificationResult, queue_size=queue_size)

        self.subs = [sub_box, sub_class]
        sync = message_filters.TimeSynchronizer(
            fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self.printMsg)

    def printMsg(self, rect_msg, class_msg):
        for rect, proba in zip(rect_msg.rects, class_msg.label_proba):
            if(proba > 0.97):
                self.rect_x_list.append(rect.x)
                self.rect_y_list.append(rect.y)

    def callbackTimer(self, event):
        self.rects_x_list.append(self.rect_x_list)
        self.rects_y_list.append(self.rect_y_list)
        self.rect_x_list = []
        self.rect_y_list = []
        if sum([len(v) for v in self.rects_x_list[-5:]]) != 0:
            rect_x_list= [flatten for inner in self.rects_x_list[-5:] for flatten in inner]
            rect_y_list= [flatten for inner in self.rects_y_list[-5:] for flatten in inner]
            stability = (np.var(np.array(rect_x_list)) + np.var(np.array(rect_y_list))) / len(rect_x_list) * 10
            print(sum([len(v) for v in self.rects_x_list[-5:]]))
            self.pub.publish(stability)
            del self.rects_x_list[:-5]
            del self.rects_y_list[:-5]
        else:
            self.pub.publish(0.0)
            print("No donbe")
        print("--------")

if __name__ == '__main__':
    rospy.init_node("rect_to_labeledarray")
    r2la = Rect2LabeledArray()
    rospy.spin()
