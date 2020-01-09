#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import time
import actionlib
import dynamic_reconfigure.client
import rospkg
import os
import json

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

class Adjust():

    def __init__(self):
        self.dynamic = { "max_vel_x": 10.0,
                         "max_vel_x_backwards": 10.0,
                         "max_vel_theta": 10.0,
                         "acc_lim_x": 10.0,
                         "acc_lim_theta": 10.0,
                         "max_vel_y": 10.0,
                         "acc_lim_y": 10.0}
        self.o_dynamic = { "max_vel_x": 0.0,
                           "max_vel_x_backwards": 0.0,
                           "max_vel_theta": 0.0,
                           "acc_lim_x": 0.0,
                           "acc_lim_theta": 0.0,
                           "max_vel_y": 0.0,
                           "acc_lim_y": 0.0}
        self.client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout=5, config_callback=self.dynamic_callback)
        self.server = rospy.Service("/adjust_server", Trigger, self.server_callback)
        self.sub_hazardstate = rospy.Subscriber("/hazard_map_state", String, self.hazard)
        #debug
        #self.update_logs()
        #self.adjust_speed()
        #finish debug

    def server_callback(self, _):
        self.update_logs()
        self.adjust_speed()
        self.offset_dynamic()
        self.set_dynamic()
        #response service
        req = TriggerResponse()
        req.success = True
        req.message = "adjust vel and acc"
        return req

    def update_logs(self):
        self.logs = []
        rospack = rospkg.RosPack()
        dir_path=rospack.get_path("stability_detector") + "/log"
        for curDir, dirs, files in os.walk(dir_path):
            for n in range(len(files)):
                path = curDir + "/log" + str(n)
                self.logs.append(json.load(open(path)))
            num = len(files)
            self.f_param = { "max_vel_x": 10.0,
                             "max_vel_x_backwards": 10.0,
                             "max_vel_theta": 10.0,
                             "acc_lim_x": 10.0,
                             "acc_lim_theta": 10.0,
                             "max_vel_y": 10.0,
                             "acc_lim_y": 10.0,
                             "success": "false" }
            self.fail_log = []
            self.success_log = []
            for i in range(num):
                if self.logs[i][-1]["success"] == "false":
                    self.fail_log = self.logs[i][-1]
                elif  self.logs[i][-1]["success"] == "success":
                    self.success_log = self.logs[i]

        #print self.fail_log
        #print self.success_log[-1]
        self.log = self.logs[-1]
        if self.success_log != []:
            self.dynamic = { "max_vel_x": self.success_log[-1]["max_vel_x"],
                             "max_vel_x_backwards": self.success_log[-1]["max_vel_x_backwards"],
                             "max_vel_theta": self.success_log[-1]["max_vel_theta"],
                             "acc_lim_x": self.success_log[-1]["acc_lim_x"],
                             "acc_lim_theta": self.success_log[-1]["acc_lim_theta"],
                             "max_vel_y": self.success_log[-1]["max_vel_y"],
                             "acc_lim_y": self.success_log[-1]["acc_lim_y"]}

    def adjust_speed(self):
        if self.fail_log == []:
            print "all success"
            self.adjust_at_success()
        else:
            print "fail exist"
            self.adjust_at_false()

    def offset_dynamic(self):
        offset = float(self.hazard) * 2.0
        if offset > 1.0:
            self.o_dynamic["max_vel_x"] = 0.5 * self.dynamic["max_vel_x"]
            self.o_dynamic["max_vel_y"] = 0.5 * self.dynamic["max_vel_y"]
            self.o_dynamic["max_vel_theta"] = 0.5 * self.dynamic["max_vel_theta"]
            self.o_dynamic["acc_lim_x"] = 0.5 * self.dynamic["acc_lim_x"]
            self.o_dynamic["acc_lim_y"] = 0.5 * self.dynamic["acc_lim_y"]
            self.o_dynamic["acc_lim_theta"] = 0.5 * self.dynamic["acc_lim_theta"]
        else:
            self.o_dynamic["max_vel_x"] = (1 - (0.5*offset))*self.dynamic["max_vel_x"]
            self.o_dynamic["max_vel_y"] = (1 - (0.5*offset))*self.dynamic["max_vel_y"]
            self.o_dynamic["max_vel_theta"] = (1 - (0.5*offset))*self.dynamic["max_vel_theta"]
            self.o_dynamic["acc_lim_x"] = (1 - (0.5*offset))*self.dynamic["acc_lim_x"]
            self.o_dynamic["acc_lim_y"] = (1 - (0.5*offset))*self.dynamic["acc_lim_y"]
            self.o_dynamic["acc_lim_theta"] = (1 - (0.5*offset))*self.dynamic["acc_lim_theta"]
        self.o_dynamic["max_vel_x_backwards"] = self.o_dynamic["max_vel_x"] * 0.8

    def get_dynamic(self):
        params = self.client.get_configuration()
        self.dynamic = { u"max_vel_x": params["max_vel_x"],
                         u"max_vel_x_backwards": params["max_vel_x_backwards"],
                         u"max_vel_theta": params["max_vel_theta"],
                         u"acc_lim_x": params["acc_lim_x"],
                         u"acc_lim_theta": params["acc_lim_theta"],
                         u"max_vel_y": params["max_vel_y"],
                         u"acc_lim_y": params["acc_lim_y"]}

    def set_dynamic(self):
        self.client.update_configuration(self.o_dynamic)

    def adjust_at_success(self):
        linear_x = []
        acc_x = []
        linear_y = []
        acc_y = []
        angular_z = []
        acc_z = []
        for i in range(len(self.success_log)-1):
            linear_x.append(self.success_log[i]["cmd_vel"]["linear_x"])
            linear_y.append(self.success_log[i]["cmd_vel"]["linear_y"])
            angular_z.append(self.success_log[i]["cmd_vel"]["angular_z"])
        for i in range(len(self.success_log)-2):
            acc_x.append((linear_x[i+1]-linear_x[i])*10)
            acc_y.append((linear_y[i+1]-linear_y[i])*10)
            acc_z.append((angular_z[i+1]-angular_z[i])*10)
        max_vel_x = self.success_log[-1]["max_vel_x"]
        max_vel_y = self.success_log[-1]["max_vel_y"]
        max_vel_theta = self.success_log[-1]["max_vel_theta"]
        acc_lim_x = self.success_log[-1]["acc_lim_x"]
        acc_lim_y = self.success_log[-1]["acc_lim_y"]
        acc_lim_theta = self.success_log[-1]["acc_lim_theta"]
        #adjust x vel and acc
        if(max(abs(np.asarray(linear_x))) > max_vel_x*0.99):
            if max_vel_x < self.f_param["max_vel_x"]:
                self.dynamic["max_vel_x"] = max_vel_x + 0.1
                rospy.loginfo("change max_vel_x {} -> {}".format(max_vel_x, self.dynamic["max_vel_x"]))
        elif(max(abs(np.asarray(acc_x))) > acc_lim_x*4):
            if acc_lim_x < self.f_param["acc_lim_x"]:
                self.dynamic["acc_lim_x"] = acc_lim_x + 0.02
                rospy.loginfo("change acc_lim_x {} -> {}".format(acc_lim_x, self.dynamic["acc_lim_x"]))
        #adjust y vel and acc
        if(max(abs(np.asarray(linear_y))) > max_vel_y*0.99):
            self.dynamic["max_vel_y"] = max_vel_y + 0.1
            rospy.loginfo("change max_vel_y {} -> {}".format(max_vel_y, self.dynamic["max_vel_y"]))
        elif(max(abs(np.asarray(acc_y))) > acc_lim_y*4):
            self.dynamic["acc_lim_y"] = acc_lim_y + 0.02
            rospy.loginfo("change acc_lim_y {} -> {}".format(acc_lim_y, self.dynamic["acc_lim_y"]))
        #adjust theta vel and acc
        if(max(abs(np.asarray(angular_z))) > max_vel_theta*0.99):
            self.dynamic["max_vel_theta"] = max_vel_theta + 0.1
            rospy.loginfo("change max_vel_theta {} -> {}".format(max_vel_theta, self.dynamic["max_vel_theta"]))
        elif(max(abs(np.asarray(acc_z))) > acc_lim_theta*4):
            self.dynamic["acc_lim_theta"] = acc_lim_theta + 0.02
            rospy.loginfo("change acc_lim_theta {} -> {}".format(acc_lim_theta, self.dynamic["acc_lim_theta"]))

    def adjust_at_false(self):
        x = np.linspace(0.1,1.0,10)
        linear_x = []
        linear_y = []
        angular_z = []
        #print success_log
        for i in range(-11, -1):
            linear_x.append(self.success_log[i]["cmd_vel"]["linear_x"])
            linear_y.append(self.success_log[i]["cmd_vel"]["linear_y"])
            angular_z.append(self.success_log[i]["cmd_vel"]["angular_z"])
        acc_x, _ = np.polyfit(x, linear_x, 1)
        acc_y, _ = np.polyfit(x, linear_y, 1)
        acc_z, _ = np.polyfit(x, angular_z, 1)
        max_vel_x = self.dynamic["max_vel_x"]
        max_vel_y = self.dynamic["max_vel_y"]
        max_vel_theta = self.dynamic["max_vel_theta"]
        acc_lim_x = self.dynamic["acc_lim_x"]
        acc_lim_y = self.dynamic["acc_lim_y"]
        acc_lim_theta = self.dynamic["acc_lim_theta"]
        #adjust x vel and acc
        if(abs(acc_x) > acc_lim_x*0.8):
            if self.success_log[-1]["acc_lim_x"] != self.fail_log["acc_lim_x"]:
                self.dynamic["acc_lim_x"] = (self.success_log[-1]["acc_lim_x"] + self.fail_log["acc_lim_x"]) / 2.0
                rospy.loginfo("change acc_lim_x {} -> {}".format(acc_lim_x, self.dynamic["acc_lim_x"]))
        elif(max(abs(np.asarray(linear_x))) > max_vel_x*0.8):
            if self.success_log[-1]["max_vel_x"] != self.fail_log["max_vel_x"]:
                self.dynamic["max_vel_x"] =  (self.success_log[-1]["max_vel_x"] + self.fail_log["max_vel_x"]) / 2.0
                rospy.loginfo("change max_vel_x {} -> {}".format(max_vel_x, self.dynamic["max_vel_x"]))
        #adjust y vel and acc
        if(abs(acc_y) > acc_lim_y*0.8):
            if self.success_log[-1]["acc_lim_y"] != self.fail_log["acc_lim_y"]:
                self.dynamic["acc_lim_y"] = (self.success_log[-1]["acc_lim_y"] + self.fail_log["acc_lim_y"]) / 2.0
                rospy.loginfo("change acc_lim_y {} -> {}".format(acc_lim_y, self.dynamic["acc_lim_y"]))
        elif(max(abs(np.asarray(linear_y))) > max_vel_y*0.8):
            if self.success_log[-1]["max_vel_y"] != self.fail_log["max_vel_y"]:
                self.dynamic["max_vel_y"] = (self.success_log[-1]["max_vel_y"] + self.fail_log["max_vel_y"]) / 2.0
                rospy.loginfo("change max_vel_y {} -> {}".format(max_vel_y, self.dynamic["max_vel_y"]))
        #adjust theta vel and acc
        if(abs(acc_z) > acc_lim_theta*0.8):
            if self.success_log[-1]["acc_lim_theta"] != self.fail_log["acc_lim_theta"]:
                self.dynamic["acc_lim_theta"] = (self.success_log[-1]["acc_lim_theta"] + self.fail_log["acc_lim_theta"]) / 2.0
                rospy.loginfo("change acc_lim_theta {} -> {}".format(acc_lim_theta, self.dynamic["acc_lim_theta"]))
        elif(max(abs(np.asarray(angular_z))) > max_vel_theta*0.8):
            if self.success_log[-1]["max_vel_theta"] != self.fail_log["max_vel_theta"]:
                self.dynamic["max_vel_theta"] = (self.success_log[-1]["max_vel_theta"] + self.fail_log["max_vel_theta"]) / 2.0
                rospy.loginfo("change max_vel_theta {} -> {}".format(max_vel_theta, self.dynamic["max_vel_theta"]))

    def hazard(self, msg):
        self.hazard = str(msg.data)

    def dynamic_callback(self, config):
        rospy.loginfo("Config set to max_vel_x: {max_vel_x}, max_vel_x_backwards: {max_vel_x_backwards}, max_vel_theta: {max_vel_theta}, acc_lim_x: {acc_lim_x}, acc_lim_theta: {acc_lim_theta}, max_vel_y: {max_vel_y}, acc_lim_y: {acc_lim_y}".format(**config))

if __name__ == "__main__":
    rospy.init_node("adjust_speed_server")
    adj = Adjust()
    rospy.spin()
