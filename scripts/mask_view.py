#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from jsk_recognition_msgs.msg import ClusterPointIndices

class mask_view:

  def __init__(self):
    self.bridge = CvBridge()
    self.indices_sub = rospy.Subscriber("/ssd_donbe_detector/output/cluster_indices", ClusterPointIndices, self.callback)

  def callback(self,msg):
    mask = np.zeros((480, 640))
    indices = msg.cluster_indices
    for index in indices:
        mask.reshape(-1)[np.array(index.indices)] = 255
    cv2.imshow("mask", mask.reshape(480, 640))
    cv2.waitKey(1)

def main(args):
  ic = mask_view()
  rospy.init_node('mask_view', anonymous=True)
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
