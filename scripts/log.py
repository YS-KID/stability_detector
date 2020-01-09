#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import json
import signal
import sys
import numpy as np
import rospy
import rospkg
import matplotlib.pyplot as plt
import dynamic_reconfigure.client

from std_msgs.msg import Float32
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

global_stability = 0
global_jobstate = "init"
global_odom = {}
global_cmd_vel = [0.0, 0.0, 0.0]
global_amcl = {}
global_log = []
global_success = "success"
global_dynamic = {}

rospack = rospkg.RosPack()
dir_path = rospack.get_path("stability_detector") + "/log"

num = 0
for curDir, dirs, files in os.walk(dir_path):
    num = len(files)

file_name = "log" + str(num)
file_path = dir_path + "/" + file_name

fw = open(file_path, 'w')

class get_stability():

    def __init__(self):
        rospy.Subscriber("/donbe_stability_detector/output/stability", Float32, self.subscribe)

    def subscribe(self, stability):
        global global_stability
        global_stability = stability.data


class get_state():

    def __init__(self):
        rospy.Subscriber("/jobstate", String, self.subscribe)

    def subscribe(self, jobstate):
        global global_jobstate
        global_jobstate = jobstate.data


class get_odom():

    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.subscribe)

    def subscribe(self, odom):
        global global_odom
        global_odom = odom.twist

class get_cmd_vel():

    def __init__(self):
        rospy.Subscriber("/cmd_vel", Twist, self.subscribe)

    def subscribe(self, cmd_vel):
        global global_cmd_vel
        global_cmd_vel[0] = cmd_vel.linear.x
        global_cmd_vel[1] = cmd_vel.linear.y
        global_cmd_vel[2] = cmd_vel.angular.z


class get_amcl():

    def __init__(self):
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.subscribe)

    def subscribe(self, amcl):
        global global_amcl
        global_amcl = amcl.pose


class save_log():

    def __init__(self):
        rospy.Timer(rospy.Duration(0.1), self.save)
        self.section_num = 0

    def save(self, _):
        global global_stability, global_jobstate, global_odom, global_amcl, global_log, num, global_success, global_cmd_vel
        if global_odom == {}:
            print("odom nil")
            return
        if global_amcl == {}:
            print("amcl nil")
            return
        if global_jobstate == "move trolley":
            name = str(num) + "." + str(self.section_num)
            stability = global_stability
            jobstate = global_jobstate
            odom = { u"linear_x": global_odom.twist.linear.x,
                     u"linear_y": global_odom.twist.linear.y,
                     u"angular": global_odom.twist.angular.z}
            cmd_vel = { u"linear_x": global_cmd_vel[0],
                        u"linear_y": global_cmd_vel[1],
                        u"angular_z": global_cmd_vel[2]}
            amcl = { u"pose_x": global_amcl.pose.position.x,
                     u"pose_y": global_amcl.pose.position.y,
                     u"orientation_x": global_amcl.pose.orientation.x,
                     u"orientation_y": global_amcl.pose.orientation.y,
                     u"orientation_z": global_amcl.pose.orientation.z,
                     u"orientation_w": global_amcl.pose.orientation.w}
            log = { u"section": name,
                    u"stability": stability,
                    u"jobstate": jobstate,
                    u"odom": odom,
                    u"cmd_vel":cmd_vel,
                    u"amcl": amcl}
            global_log.append(log)
            print(self.section_num)
            self.section_num += 1

        if global_jobstate == "collapse":
            global_success = "false"

class get_dynamic():
    def __init__(self):
        self.client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS",timeout=5, config_callback=self.dynamic_callback)

    def get_params(self):
        global global_dynamic
        global_dynamic = self.client.get_configuration()

    def dynamic_callback(self, _):
        pass

def Exit_gracefully(signal, frame):
    global global_log, fw, global_success, global_dynamic
    param = { u"success": global_success,
              u"max_vel_x": global_dynamic["max_vel_x"],
              u"max_vel_x_backwards": global_dynamic["max_vel_x_backwards"],
              u"max_vel_theta": global_dynamic["max_vel_theta"],
              u"acc_lim_x": global_dynamic["acc_lim_x"],
              u"acc_lim_theta": global_dynamic["acc_lim_theta"],
              u"max_vel_y": global_dynamic["max_vel_y"],
              u"acc_lim_y": global_dynamic["acc_lim_y"]}
    global_log.append(param)
    json.dump(global_log, fw, indent=4)
    print("exit")
    sys.exit(0)

if __name__== "__main__":
    signal.signal(signal.SIGINT, Exit_gracefully)

    rospy.init_node("stability_logger")
    sta = get_stability()
    sta = get_state()
    odo = get_odom()
    amc = get_amcl()
    cmd = get_cmd_vel()
    dyn = get_dynamic()
    dyn.get_params()
    sav = save_log()
    rospy.spin()


