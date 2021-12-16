#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  2 18:19:39 2021

@author: chen
"""
import numpy as np
import time
cimport numpy as np
cimport cython

DTYPE = np.int

ctypedef np.int_t DTYPE_t

# @cython.boundscheck(False)
# @cython.wraparound(False)
# cdef np.ndarray[np.float32_t, ndim=1] _ellipse_ck(np.ndarray[double, ndim=2] X, np.ndarray[double, ndim=2] x):
#     cdef int if_collide
#     cdef double t_exh,SA,t3,L,UB,t2
#     cdef np.ndarray[DTYPE_t, ndim=1] ids
#     cdef np.ndarray[double, ndim=2] aa,bb
#     cdef np.ndarray[double, ndim=1] l1s,l2s,jt,x1,x2
#     cdef np.ndarray[np.float32_t, ndim=1] rt = np.zeros(3, dtype=np.float32)
#     cdef np.ndarray[np.float32_t, ndim=1] t_ad = np.zeros(3, dtype=np.float32)
#     x1=x[0].copy()
#     x2=x[0]+0.2
#     if_collide=0
#     # t_exh = 0
#     # l1s = np.linalg.norm(X-x1,axis=1)
#     for i in range (200):
#         t_exh = time.time()
#         L=sum((x2-x1)**2)**0.5
#         SA=0.05
#         UB=(SA+(SA**2+L**2)**0.5)
#         # xx1=np.array([min(x1[0],x2[0]),min(x1[1],x2[1]),min(x1[2],x2[2])])
#         # xx2=np.array([max(x1[0],x2[0]),max(x1[1],x2[1]),max(x1[2],x2[2])])
#         # index = np.where((xx2[0]+SA > X[:,0]) & (xx1[0]-SA < X[:,0]) &(xx2[1]+SA > X[:,1]) & (xx1[1]-SA < X[:,1]) &
#         #                   (xx2[2]+SA > X[:,2]) &(xx1[2]-SA < X[:,2]))
#         t3=time.time()-t_exh
#         # X = X[index]
#         # l1s = np.linalg.norm(X-x1,axis=1)
#         # l2s = np.linalg.norm(X-x2,axis=1)
#         if len(X):
#             aa = (X-x1)**2
#             bb= (X-x2)**2
#             l1s = np.sum(aa,axis=1) #a^2
#             l2s = np.sum(bb,axis=1) #b^2
#             jt = 2*(L**2)*(l1s+l2s)-(l1s-l2s)**2-L**4
           
#             # aa = np.sort(l1s+l2s)
#             ids = np.where((l1s**0.5+l2s**0.5<UB)&(jt<4*(L*SA)**2))[0]
#             t2=time.time()-t_exh
#             if len(ids):# and aa[0]<UB:
#                 if_collide=1
#             rt[0] = time.time()-t_exh
#             rt[1] = t2
#             rt[2] = t3
#             # return rt
#         else:
#             rt[0] = time.time()-t_exh
#             rt[1] = 0.0
#             rt[2] = t3
#             # rt2 = np.array([time.time()-t_exh,0,t3])
#             # return rt2
#         t_ad += rt
#         return t_ad
@cython.boundscheck(False)
@cython.wraparound(False)
cdef _ellipse_ck(X,x):
    x1=x[0].copy()
    x2=x[0]+0.2
    if_collide=0
    rt = np.zeros(3, dtype=np.float32)
    t_ad = np.zeros(3, dtype=np.float32)
    # t_exh = 0
    # l1s = np.linalg.norm(X-x1,axis=1)
    # l1s = np.zeros(len(X))
    # l2s = np.zeros(len(X))
    for i in range (200):
        t_exh = time.time()
        L=sum((x2-x1)**2)**0.5
        SA=0.05
        UB=(SA+(SA**2+L**2)**0.5)
        # xx1=np.array([min(x1[0],x2[0]),min(x1[1],x2[1]),min(x1[2],x2[2])])
        # xx2=np.array([max(x1[0],x2[0]),max(x1[1],x2[1]),max(x1[2],x2[2])])
        # index = np.where((xx2[0]+SA > X[:,0]) & (xx1[0]-SA < X[:,0]) &(xx2[1]+SA > X[:,1]) & (xx1[1]-SA < X[:,1]) &
        #                   (xx2[2]+SA > X[:,2]) &(xx1[2]-SA < X[:,2]))
        t3=time.time()-t_exh
        # X = X[index]
        # l1s = np.linalg.norm(X-x1,axis=1)
        # l2s = np.linalg.norm(X-x2,axis=1)
        if len(X):
            aa = (X-x1)**2
            bb= (X-x2)**2
            l1s = np.sum(aa,axis=1) #a^2
            l2s = np.sum(bb,axis=1) #b^2
            # for aai in range(len(aa)):
            #     l1s[aai] = aa[aai,0]+aa[aai,1]+aa[aai,2]
            #     l2s[aai] = bb[aai,0]+bb[aai,1]+bb[aai,2]
            jt = 2*(L**2)*(l1s+l2s)-(l1s-l2s)**2-L**4
           
            # aa = np.sort(l1s+l2s)
            ids = np.where((l1s**0.5+l2s**0.5<UB)&(jt<4*(L*SA)**2))[0]
            # for aai in range(len(aa)):
            #     if (l1s[aai]**0.5+l2s[aai]**0.5<UB) and (jt[aai]<4*(L*SA)**2):
            #         if_collide=1
            #         return t_ad
            t2=time.time()-t_exh
            if len(ids):# and aa[0]<UB:
                if_collide=1
            rt[0] = time.time()-t_exh
            rt[1] = t2
            rt[2] = t3
            # return rt
        else:
            rt[0] = time.time()-t_exh
            rt[1] = 0.0
            rt[2] = t3
            # rt2 = np.array([time.time()-t_exh,0,t3])
            # return rt2
        t_ad += rt
        return t_ad
def ellipse_test(a, b):
    return _ellipse_ck(a,b)
    # return if_collide