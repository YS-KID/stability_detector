#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import matplotlib.pyplot as plt

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
    stability_list = []

    def __init__(self):
        self.pub = rospy.Publisher("~output/stability", Float32, queue_size=1)
        self.subscribe()
        rospy.Timer(rospy.Duration(1), self.callbackTimer)

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
        #print(class_msg.label_proba)
        for rect, proba in zip(rect_msg.rects, class_msg.label_proba):
            if(proba > 0.97):
                self.rect_x_list.append(rect.x)
                self.rect_y_list.append(rect.y)

    def callbackTimer(self, event):
        print(len(self.rect_x_list))
        if len(self.rect_x_list) != 0:
            self.stability = (np.var(np.array(self.rect_x_list)) + np.var(np.array(self.rect_y_list))) / len(self.rect_x_list) * 10 
            self.pub.publish(self.stability)
            self.stability_list.append(self.stability)
            plt.plot(self.stability_list)
            plt.pause(0.01)
            plt.cla()
        else:
            self.pub.publish(0.0)
            print("No donbe")
        print("--------")
        del self.rect_x_list[:]
        del self.rect_y_list[:]

if __name__ == '__main__':
    rospy.init_node("rect_to_labeledarray")
    r2la = Rect2LabeledArray()
    rospy.spin()
