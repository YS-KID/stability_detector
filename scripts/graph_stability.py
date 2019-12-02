#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import matplotlib.pyplot as plt

from std_msgs.msg import Float32
from std_msgs.msg import String
from stability_detector_msgs.srv import JobState, JobStateResponse, JobStateRequest

def one_shot_subscribe(topic_name, mclass=None,
                       timeout=None, after_stamp=None,
                       condition=None,
                       frequency=10):
    """
    Subscribe message, just for once
    """
    if mclass is None:
        import rostopic
        import importlib
        import rosgraph
        master = rosgraph.masterapi.Master('/rostopic')
        topic_types = dict(rostopic._master_get_topic_types(master))
        topic_type = topic_types[topic_name].split('/')
        pkg = importlib.import_module('{}.msg'.format(topic_type[0]))
        mclass = getattr(pkg, topic_type[1])

    if after_stamp is not None and isinstance(after_stamp, rospy.Time):
        raise TypeError('after_stamp should be rospy.Time, but get {}'
                        .format(type(after_stamp)))

    class OneShotSubscriber(object):

        def __init__(self):
            self.msg = None
            self.sub = rospy.Subscriber(
                topic_name, mclass,
                self.one_shot_subscribe)

        def unsubscribe(self):
            self.sub.unregister()

        def one_shot_subscribe(self, msg):

            if after_stamp is not None:
                if msg.header.stamp.to_sec() > after_stamp.to_sec():
                    self.msg = msg
            else:
                self.msg = msg

    oss = OneShotSubscriber()

    finish_time = None
    if timeout is not None:
        finish_time = rospy.Time.now()
    if finish_time is not None:
        finish_time = finish_time + rospy.Duration(timeout / 1000.0)

    r = rospy.Rate(frequency)
    while not rospy.is_shutdown() and \
        not (finish_time is not None and
             (finish_time - rospy.Time.now()).to_sec < 0.0):
        r.sleep()
        if oss.msg is not None:
            if (condition is not None):
                if callable(condition) and \
                   condition(oss.msg):
                    break
            else:
                break
    oss.unsubscribe()
    return oss.msg

state = "init"
stability_list = [0.0]

class get_stability():

    def __init__(self):
        rospy.Subscriber("/donbe_stability_detector/output/stability", Float32, self.subscribe)

    def subscribe(self, stability):
        global state, stability_list
        if state == "move trolley":
            print(stability.data)
            stability_list.append(stability.data)

class get_state():

    def __init__(self):
        rospy.Subscriber("/jobstate", String, self.subscribe)

    def subscribe(self, jobstate):
        global state
        state = jobstate.data

class graph():

    def __init__(self):
        rospy.Timer(rospy.Duration(1), self.graph)

    def graph(self, event):
        global stability_list
        plt.plot(stability_list, label="stability")
        plt.pause(0.001)
        plt.cla()

if __name__ == "__main__":
   rospy.init_node("graph_stability", anonymous = True)
   a = get_stability()
   b = get_state()
   c = graph()
   rospy.spin()
