#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from PP_tree import PP_TREE
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


SA = 0.4
pptree = PP_TREE()
samples = np.random.rand(500,3)*5
goal = np.array([5,5,4])
dists = np.linalg.norm(samples,axis=1)
index = np.argsort(dists)
dists = dists[index]
samples = samples[index]
pcl = np.r_[np.random.rand(200,3)*np.random.rand(1)*2+np.random.rand(3)*5,
            np.random.rand(10,3)*np.random.rand(1)*2+np.random.rand(3)*4,
            np.random.rand(50,3)*np.random.rand(1)*2+np.random.rand(3)*3,
            np.random.rand(200,3)*np.random.rand(1)*2+np.random.rand(3)*5]
# pcl = np.random.rand(100,3)*np.random.rand(1)*1+np.random.rand(3)*4+1
t1 = time.time()
pptree.kd_tree(pcl, 4)
samples = pptree.kd_colcheck(pcl, samples, SA)
# for i in range(len(samples)):
a=pptree.insert_node(samples, SA*1.5)
# goal = a[-1]
path = np.array(pptree.get_path(goal))
t2 = time.time()
fig = plt.figure()
ax1 = plt.axes(projection='3d')
ax1.scatter3D(a[:,0],a[:,1],a[:,2], cmap='Blues')
ax1.scatter3D(pcl[:,0],pcl[:,1],pcl[:,2], color='Green')
ax1.scatter3D(goal[0],goal[1],goal[2], color='Orange',s=25)
ax1.plot3D(path[:,0],path[:,1],path[:,2],'Red')
plt.show()
# print(a)
print("path:",path)
print("time:",t2-t1)