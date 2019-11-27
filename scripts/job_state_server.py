#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import time

from stability_detector_msgs.srv import JobState, JobStateResponse

class JobStateServer:

    state = "init"

    def __init__(self):
        rospy.init_node("job_state_server")
        self.job_state_server = rospy.Service("/jobstate", JobState, self.change_state)
        rospy.spin()

    def change_state(self, trigger):
        req = JobStateResponse()
        req.success = True
        if trigger.trigger == "finish grab trolley":
            self.state = "move trolley"
        elif trigger.trigger == "finish move trolley":
            self.state = "release trolley"
        elif trigger.trigger == "finish release trolley":
            self.state = "finish"

        req.state = self.state
        return req

if __name__ == "__main__":
  srv = JobStateServer()

