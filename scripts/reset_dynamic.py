#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

class Reset():

    def __init__(self):
        client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout=5, config_callback=self.callback)
        params = { "max_vel_x": 0.25,
                   "max_vel_y": 0.2,
                   "max_vel_x_backwards": 0.2,
                   "max_vel_theta": 0.25,
                   "acc_lim_x": 0.2,
                   "acc_lim_y": 0.2,
                   "acc_lim_theta": 0.2}
        client.update_configuration(params)

    def callback(self, config):
        pass

if __name__ == "__main__":
    rospy.init_node("dynamic_client")
    reset = Reset()

