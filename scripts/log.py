#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import json
import numpy as np
import rospy
import rospkg
import matplotlib.pyplot as plt

from std_msgs.msg import Float32
from std_msgs.msg import String
from stability_detector_msgs.srv import JobState, JobStateResponse, JobStateRequest

rospy.init_node("stability_logger")

rospack = rospkg.RosPack()
dir_path = rospack.get_path("stability_detector") + "/log"

num = 0
for curDir, dirs, files in os.walk(dir_path):
    num = len(files)

file_name = "log" + str(num+1)
file_path = dir_path + "/" + file_name

a = {}

fw = open(a,file_path, 'w')

