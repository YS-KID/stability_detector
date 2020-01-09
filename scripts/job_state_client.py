#!/usr/bin/env python

import rospy
from stability_detector_msgs.srv import JobState, JobStateRequest, JobStateResponse

def get_state():
    job_state = rospy.ServiceProxy('/jobstate', JobState)
    req = JobStateRequest(trigger="get state")
    res = job_state(req)
    return res

if __name__ == "__main__":
    a = get_state()
    print(a.success)
    print(a.state)

