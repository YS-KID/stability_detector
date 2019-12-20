#!/usr/bin/env python

import json
import matplotlib.pyplot as plt
import roslib.packages as rp
import numpy as np
import seaborn as sns
import pandas as pd
import matplotlib as mpl

def get_max_index(array, max_num):
    array = np.copy(array)
    index = 0
    for i in range(max_num):
        index = array.argmax()
        array[index] = 0
    return index

def get_max_index_in_convex(array, max_num):
    array = np.copy(array)
    convex_list = get_convex_index(array)
    r_array = np.zeros(len(array))
    for i in convex_list:
        r_array[i] = array[i]
    index = get_max_index(r_array, max_num)
    return index

def get_convex_index(array):
    array = np.copy(array)
    gradient = []
    for i in range(len(array)-1):
        gradient.append(array[i+1]-array[i])
    index = []
    for i in range(len(gradient)-1):
        if gradient[i] > 0 and gradient[i+1] <= 0:
            index.append(i+1)
    return index

if __name__ == "__main__":
    pkg_pass = rp.get_pkg_dir("stability_detector")
    log_pass = pkg_pass + "/log/log7"

    a = open(log_pass)
    b = json.load(a)

    stability = []
    amcl = []

    for i in range(len(b)):
        sta = b[i]["stability"]
        if(sta > 1000):
            stability.append(1000)
        else:
            stability.append(sta)
        amcl.append([b[i]["amcl"]["pose_x"], b[i]["amcl"]["pose_y"]])

    stability = np.array(stability)
    amcl = np.array(amcl)

    x_min = np.min(amcl[:,0])
    y_min = np.min(amcl[:,1])
    coo_amcl_x = (amcl[:,0] - x_min)*10
    coo_amcl_y = (amcl[:,1] - y_min)*10
    coo_amcl = np.array([coo_amcl_x, coo_amcl_y]).T.astype(int)

    x_max = np.max(coo_amcl[:,0])
    x_range = x_max + 1
    y_max = np.max(coo_amcl[:,1])
    y_range = y_max + 1

    #Map = np.zeros(x_range*y_range).reshape([y_range, x_range])
    Map = np.zeros(1000*1000).reshape([1000,1000])
    for i in coo_amcl:
        Map[i[0]][i[1]] = 1

    n_index = get_max_index_in_convex(stability, 1)

    plt.figure()
    sns.heatmap(Map, square=True)
    plt.show("Map")


