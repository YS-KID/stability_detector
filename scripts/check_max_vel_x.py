#!/usr/bin/env python

import json
import matplotlib.pyplot as plt
import roslib.packages as rp

import os

pkg_pass = rp.get_pkg_dir("stability_detector")
log_pass = pkg_pass + "/log"

acc = []

for curDir, dirs, files in os.walk(log_pass):
    for n in range(len(files)):
        file_pass = log_pass + "/log"  + str(n)
        a = open(file_pass)
        b = json.load(a)
        acc.append(b[-1]["max_vel_x"])

plt.plot(acc)
plt.show()
