#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

a = {}

def callback(config):
    global a
    a = config
    rospy.loginfo(config)

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout=5, config_callback=callback)

    r = rospy.Rate(0.1)
    x = 0
    b = False
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0
        b = not b
        client.get_parameter_descriptions()
        #client.update_configuration()
        r.sleep()
