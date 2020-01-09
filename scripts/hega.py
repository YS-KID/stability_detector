#! /usr/bin/env python

import rospy
import tf
import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *

class Hega():

    def __init__(self):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        client.cancel_all_goals()
        rospy.loginfo("cancel!!!")

if __name__ == '__main__':
    rospy.init_node('hega')
    hega = Hega()
    rospy.spin()
