#!/usr/bin/env python

import json
import matplotlib.pyplot as plt
import roslib.packages as rp
import numpy as np
import seaborn as sns
import pandas as pd
import matplotlib as mpl
import rospy
import os

from stability_detector_msgs.srv import JobState, JobStateResponse
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

def get_max_index(array, max_num):
    array = np.copy(array)
    index = 0
    for i in range(max_num):
        index = array.argmax()
        array[index] = 0
    return index

def get_max_index_in_convex(array, max_num):
    array = np.copy(array)
    convex_list = get_convex_index(array)
    r_array = np.zeros(len(array))
    for i in convex_list:
        r_array[i] = array[i]
    index = get_max_index(r_array, max_num)
    return index

def get_convex_index(array):
    array = np.copy(array)
    gradient = []
    for i in range(len(array)-1):
        gradient.append(array[i+1]-array[i])
    index = []
    for i in range(len(gradient)-1):
        if gradient[i] > 0 and gradient[i+1] <= 0:
            index.append(i+1)
    return index

class HazardMapServer():

    def __init__(self):
        self.create_map()
        self.server = rospy.Service("/hazard_map_server", JobState, self.callback)
        self.sub_amcl = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_subscribe)
        self.sub_jobstate = rospy.Subscriber("/jobstate", String, self.jobstate_subscribe)
        self.pub_state = rospy.Publisher("/hazard_map_state", String, queue_size=10)
        rospy.Timer(rospy.Duration(1), self.callbackTimer)

    def create_map(self):
        self.Map = np.zeros(1000*1000).reshape([1000,1000])

    def update_map(self):
        self.read_json()
        for l in range(len(self.stability)):
           coo_amcl_x = (self.amcl[l][:,0]+50) * 10
           coo_amcl_y = (self.amcl[l][:,1]+50) * 10
           coo_amcl = np.array([coo_amcl_x, coo_amcl_y]).T.astype(int)
           for k in range(50):
               index = get_max_index_in_convex(np.copy(self.stability[l]), k+1)
               x = coo_amcl[index][0]
               y = coo_amcl[index][1]
               for i in range(-4,5):
                   for j in range(-4,5):
                       self.Map[x+i,y+j] += (5-max(abs(i),abs(j)))

    def plot_map(self):
        x_min = 0
        x_max = 0
        for i in range(1000):
            if np.max(self.Map[:,i]) != 0.0:
                x_min = i
                break
        for i in reversed(range(1000)):
            if np.max(self.Map[:,i]) != 0.0:
                x_max = i
                break
        y_min = 0
        y_max = 0
        for i in range(1000):
            if np.max(self.Map[i,:]) != 0.0:
                y_min = i
                break
        for i in reversed(range(1000)):
            if np.max(self.Map[i,:]) != 0.0:
                y_max = i
                break
        x_range = x_max - x_min
        y_range = y_max - y_min

        view_Map = self.Map[y_min-10:y_max+10,x_min-10:x_max+10]
        plt.figure()
        sns.heatmap(view_Map, square=True)
        plt.show("Map")

    def read_json(self):
        self.stability = []
        self.amcl = []
        dir_pass = rp.get_pkg_dir("stability_detector") + "/log"

        for curDir, dirs, files in os.walk(dir_pass):
            for n in range(len(files)):
                sta = []
                amc = []
                path = curDir + "/log" + str(n)
                log = json.load(open(path))
                if log[-1]["success"] == "success":
                    for i in range(len(log)-1):
                        if log[i]["stability"] < 1000:
                            sta.append(log[i]["stability"])
                        else:
                            sta.append(1000)
                        amc.append([log[i]["amcl"]["pose_x"],
                                    log[i]["amcl"]["pose_y"]])
                    self.stability.append(np.array(sta))
                    self.amcl.append(np.array(amc))

    def callback(self, trigger):
        req = JobStateResponse()
        req.success = True
        if trigger.trigger == "plot":
            self.plot_map()
        else:
            self.update_map()
            print "update"
        req.state = "init"
        return req

    def amcl_subscribe(self, msg):
        self.now_amcl = msg

    def jobstate_subscribe(self, msg):
        self.jobstate = msg

    def callbackTimer(self, event):
        if self.jobstate.data == "move trolley":
            index = np.unravel_index(np.argmax(self.Map), self.Map.shape)
            max_score = self.Map[index[0], index[1]]
            #print self.now_amcl
            amc = [self.now_amcl.pose.pose.position.x, self.now_amcl.pose.pose.position.y]
            coo_amcl = map(lambda x: (x+50)*10, amc)
            offset = self.Map[int(coo_amcl[0]), int(coo_amcl[1])] / max_score
            if str(offset) != "nan":
                print offset
                self.pub_state.publish(str(offset))
            else:
                self.pub_state.publish(str(0.0))
        else:
            self.pub_state.publish(str(0.0))

if __name__ == "__main__":
    rospy.init_node("hazard_map")
    hazard_map = HazardMapServer()
    rospy.spin()
