#!/usr/bin/env python

import json
import matplotlib.pyplot as plt
import roslib.packages as rp

num = raw_input()

pkg_pass = rp.get_pkg_dir("stability_detector")
log_pass = pkg_pass + "/log/log" + num

a = open(log_pass)
b = json.load(a)

loss = []

for i in range(len(b)-1):
  c = b[i]["acc_lim_x"]
  if(c > 1000):
    loss.append(1000)
  else:
    loss.append(c)

plt.plot(loss)
plt.show()
