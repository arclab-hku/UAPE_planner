#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 18 11:44:00 2021

@author: chen
"""

import numpy as np
from sklearn.neighbors import KDTree
import time
# import cython
# import numba as nb
# import ellipse

def exau_test(X,x):
    # rng = np.random.RandomState(0)
    # X = rng.random_sample((500, 3))  # 10 points in 3 dimensions
    t_exh = time.time()
    xx=x[0]
    index = np.where((xx[0]+0.2 > X[:,0]) & (xx[0]-0.2 < X[:,0]) &(xx[1]+0.2 > X[:,1]) & (xx[1]-0.2 < X[:,1]) &
                      (xx[2]+0.2 > X[:,2]) &(xx[2]-0.2 < X[:,2]))
    if_collide = len(index[0])==0
    t1 = time.time()-t_exh
    dists = np.linalg.norm(X[index]-x,axis=1)
    min_dist = np.argsort(dists)
    t2 = time.time()-t_exh
    return np.array([t1,t2]), min_dist, dists

# @nb.jit()
# def ellipse_test(X,x):
#     x1=x[0].copy()
#     x2=x[0]+0.2
#     if_collide=0
#     # l1s = np.linalg.norm(X-x1,axis=1)
#     t_exh = time.time()
#     L=np.linalg.norm(x2-x1)
#     SA=0.05
#     UB=(SA+(SA**2+L**2)**0.5)
#     xx1=np.array([min(x1[0],x2[0]),min(x1[1],x2[1]),min(x1[2],x2[2])])
#     xx2=np.array([max(x1[0],x2[0]),max(x1[1],x2[1]),max(x1[2],x2[2])])
#     index = np.where((xx2[0]+SA > X[:,0]) & (xx1[0]-SA < X[:,0]) &(xx2[1]+SA > X[:,1]) & (xx1[1]-SA < X[:,1]) &
#                       (xx2[2]+SA > X[:,2]) &(xx1[2]-SA < X[:,2]))
#     t3=time.time()-t_exh
#     X = X[index]
#     # l1s = np.linalg.norm(X-x1,axis=1)
#     # l2s = np.linalg.norm(X-x2,axis=1)
#     if len(X):
#         aa = (X-x1)**2
#         bb= (X-x2)**2
#         l1s = np.sum(aa,axis=1) #a^2
#         l2s = np.sum(bb,axis=1) #b^2
#         jt = 2*(L**2)*(l1s+l2s)-(l1s-l2s)**2-L**4
       
#         # aa = np.sort(l1s+l2s)
#         aa = np.where((l1s**0.5+l2s**0.5<UB)&(jt<4*(L*SA)**2))
#         t2=time.time()-t_exh
#         if len(aa[0]):# and aa[0]<UB:
#             if_collide=1
#         return np.array([time.time()-t_exh,t2,t3])
#     else:
#         return np.array([time.time()-t_exh,0,t3])

def cross_test(X,x):
    x1=x[0].copy()
    x2=x[0]+0.2
    SA=0.05
    if_collide=0

    # l1s = np.linalg.norm(X-x1,axis=1)
    t_exh = time.time()
    L=np.linalg.norm(x2-x1)
    # UB=(SA+(SA**2+L**2)**0.5)
    # L=np.linalg.norm(x2-x1)
    xx1=np.array([min(x1[0],x2[0]),min(x1[1],x2[1]),min(x1[2],x2[2])])
    xx2=np.array([max(x1[0],x2[0]),max(x1[1],x2[1]),max(x1[2],x2[2])])
    index = np.where((xx2[0]+SA > X[:,0]) & (xx1[0]-SA < X[:,0]) &(xx2[1]+SA > X[:,1]) & (xx1[1]-SA < X[:,1]) &
                      (xx2[2]+SA > X[:,2]) &(xx1[2]-SA < X[:,2]))
    X = X[index]  
    x_x1 = X-x1
    x_x2 = X-x2
    # X = X[index]  
    # index = np.where(np.linalg.norm(X-x1,axis=1)+np.linalg.norm(X-x2,axis=1)<UB)
    t3=time.time()-t_exh
    
    t4=time.time()-t_exh
    cross_norms = np.sort(np.linalg.norm(np.cross(X-x1,X-x2),axis=1))
    # cross_norms = np.cross(x_x1,x_x2)
    t2=time.time()-t_exh
    # if len(cross_norms) : #and cross_norms[0]/L < SA:
    #     if_collide=1
    return np.array([time.time()-t_exh,t2,t3,t4])
# print(X)
build_tree = time_cost = t_exau = 0
test_times = 1
for i in range(test_times):
    rng = np.random.RandomState(0)
    X = rng.random_sample((700, 3))  # 10 points in 3 dimensions
    t1 = time.time()
    tree = KDTree(X, leaf_size=4)
    build_tree += time.time()-t1
    t2 = time.time()
    x = np.random.rand(100,3)
    dist, ind = tree.query(x, k=1)     
    time_cost += time.time()- t2    
    # a,b,c= exau_test(X,x)
    # t_exau += a
# print(ind,b[0])  # indices of 3 closest neighbors
# print(dist,c[b[0]])
print(X[ind[np.where(dist>0.05)]],dist)
print(time_cost+build_tree/test_times)
# print(t_exau)
# for i in range(test_times):
#     rng = np.random.RandomState(0)
#     X = rng.random_sample((500, 3))  # 10 points in 3 dimensions
#     x = np.random.rand(1,3)
#     t_exau += ellipse.ellipse_test(X, x)
    
    # t1 = time.time()
    # if_cld = ellipse_test.ellipse_test(X, x)
    # t_exau += time.time()-t1
    # t_exau += cross_test(X, x)
# rng = np.random.RandomState(0)
# X = rng.random_sample((500, 3))  # 10 points in 3 dimensions
# x = np.random.rand(1,3)
# t_exau += ellipse.ellipse_test(X, x)
# print(t_exau)