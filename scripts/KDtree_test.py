#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 14 10:48:04 2021

@author: chen
"""

import numpy as np
from sklearn.neighbors import KDTree
import time


# print(X)
build_tree = time_cost = 0
test_times = 100
for i in range(test_times):
    rng = np.random.RandomState(0)
    X = rng.random_sample((200, 3))  # 10 points in 3 dimensions
    t1 = time.time()
    tree = KDTree(X, leaf_size=3)
    build_tree += time.time()-t1
    t2 = time.time()
    dist, ind = tree.query(np.random.rand(1,3), k=1)     
    time_cost += time.time()- t2    
print(ind)  # indices of 3 closest neighbors
print(dist)
print(time_cost+build_tree/test_times)