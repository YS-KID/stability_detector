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
    #load log
    num = raw_input()
    pkg_pass = rp.get_pkg_dir("stability_detector")
    log_pass = pkg_pass + "/log/log" + num

    a = open(log_pass)
    b = json.load(a)

    stability = []
    amcl = []

    for i in range(len(b)-1):
        sta = b[i]["stability"]
        if(sta > 1000):
            stability.append(1000)
        else:
            stability.append(sta)
        amcl.append([b[i]["amcl"]["pose_x"], b[i]["amcl"]["pose_y"]])

    stability = np.array(stability)
    amcl = np.array(amcl)

    #create map
    Map = np.zeros(1000*1000).reshape([1000,1000])

    coo_amcl_x = (amcl[:,0]+50) * 10
    coo_amcl_y = (amcl[:,1]+50) * 10
    coo_amcl = np.array([coo_amcl_x, coo_amcl_y]).T.astype(int)
    for k in range(50):
        index = get_max_index_in_convex(stability, k+1)
        x = coo_amcl[index][0]
        y = coo_amcl[index][1]
        for i in range(-4,5):
            for j in range(-4,5):
                Map[x+i,y+j] += (5-max(abs(i),abs(j)))

    #plot Map
    x_max = np.max(coo_amcl[:,0])
    x_min = np.min(coo_amcl[:,0])
    x_range = x_max - x_min
    y_max = np.max(coo_amcl[:,1])
    y_min = np.min(coo_amcl[:,1])
    y_range = y_max - y_min

    view_Map = Map[x_min-10:x_max+10,y_min-10:y_max+10]
    plt.figure()
    sns.heatmap(view_Map, square=True)
    plt.show("Map")

    from mpl_toolkits.mplot3d import Axes3D

    fig = plt
