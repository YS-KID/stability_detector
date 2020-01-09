#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import time
import actionlib

from stability_detector_msgs.srv import JobState, JobStateResponse
from std_msgs.msg import String
from jsk_recognition_msgs.msg import BoundingBoxArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *

jobstate = "init"
boxes = BoundingBoxArray()

class subscribe_jobstate:

    def __init__(self):
        rospy.Subscriber("/jobstate", String, self.subscribe)

    def subscribe(self, msg):
        global jobstate
        jobstate = msg.data

class subscribe_boxes:

    def __init__(self):
        rospy.Subscriber("/ssd/multi_euclidean_cluster_point_indices_decomposer/boxes", BoundingBoxArray, self.subscribe)

    def subscribe(self, msg):
        global boxes
        boxes = msg

class observe_server:

    def __init__(self):
        rospy.Timer(rospy.Duration(0.2), self.observe)
        self.emergency_service_call = rospy.ServiceProxy("/jobstate",JobState)
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ac.wait_for_server()
        self.flag = 0

    def observe(self, event):
        global jobstate, boxes
        if jobstate == "move trolley" and self.flag == 0:
            if not(boxes.boxes):
                return
            self.box =  boxes.boxes[0].pose.position
            self.box = np.array([self.box.x, self.box.y, self.box.z*5])
            self.flag = 1
        if jobstate == "move trolley" and self.flag == 1:
            if not(boxes.boxes):
                return
            self.distance_list = []
            for b in boxes.boxes:
                b = b.pose.position
                b = np.array([b.x, b.y, b.z*5])
                self.distance_list.append(np.linalg.norm(self.box-b))
            if min(self.distance_list) > 0.2:
                self.ac.cancel_all_goals()
                self.emergency_service_call("collapse")
                print "collapse"
                self.flag = 0
                rospy.sleep(1)
                self.ac.cancel_all_goals()
                rospy.sleep(1)
                self.ac.cancel_all_goals()
                rospy.sleep(1)
                self.ac.cancel_all_goals()
                rospy.sleep(1)
        if jobstate != "move trolley":
            box = BoundingBoxArray()


if  __name__ == "__main__":
    rospy.init_node("observe_server", anonymous = True)
    a = subscribe_jobstate()
    b = subscribe_boxes()
    c = observe_server()
    rospy.spin()
