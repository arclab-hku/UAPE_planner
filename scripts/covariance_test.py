#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 19 17:19:26 2021

@author: chen
"""
import numpy as np
from scipy.linalg import logm
import time

pcl1=np.random.rand(50,3)*5

pcl2 = pcl1*1.1 + 2
pcl3 = np.r_[pcl2,np.array([np.mean(pcl2[2:4],axis=0)])]
t1 = time.time()
for i in range(1000):
    # d_cov = np.linalg.norm(logm(np.cov(pcl1.T))-logm(np.cov(pcl2.T)),ord='fro')
    d_cov = np.linalg.norm(np.cov(pcl1.T)-np.cov(pcl3.T),ord='fro')
t2 = time.time()-t1
d_cov1 = np.linalg.norm(logm(np.cov(pcl1.T))-logm(np.cov(pcl3.T)),ord='fro')
print(t2,d_cov,d_cov**0.3,d_cov1)
