#!/usr/bin/env python

import time
import json
import matplotlib.pyplot as plt
import roslib.packages as rp

num = raw_input()

pkg_pass = rp.get_pkg_dir("stability_detector")
log_pass = pkg_pass + "/log/log" + num

a = open(log_pass)
b = json.load(a)

linear_x = []
linear_y = []
angular_z = []

for i in range(len(b)-1):
  linear_x.append(b[i]["cmd_vel"]["linear_x"])
  linear_y.append(b[i]["cmd_vel"]["linear_y"])
  angular_z.append(b[i]["cmd_vel"]["angular_z"])

fig = plt.figure()

ax1 = fig.add_subplot(3, 1, 1)
ax1.plot(linear_x)

ax2 = fig.add_subplot(3, 1, 2)
ax2.plot(linear_y)

ax3 = fig.add_subplot(3, 1, 3)
ax3.plot(angular_z)

fig.show()
time.sleep(4)
