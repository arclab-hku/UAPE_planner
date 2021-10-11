#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul  6 12:35:31 2021

@author: chen
"""
import numpy as np

#pythran export line_sf(float64[:,:], float64[:], float64[:], float64)
def line_sf (pcl,p1,p2,SA): #
    if_collide=0
    # l1s = np.linalg.norm(X-p1,axis=1)
   
    L=np.linalg.norm(p2-p1)
    UB=(SA+(SA**2+L**2)**0.5)
    # l1s = np.linalg.norm(pcl-p1,axis=1)
    # l2s = np.linalg.norm(pcl-p2,axis=1)
    if len(pcl):
        aa = (pcl-p1)**2
        bb= (pcl-p2)**2
        l1s = np.sum(aa,axis=1) #a^2
        l2s = np.sum(bb,axis=1) #b^2
        jt = 2*(L**2)*(l1s+l2s)-(l1s-l2s)**2-L**4

        aa = np.where((l1s**0.5+l2s**0.5<UB)&(jt<4*(L*SA)**2))
        if len(aa[0]):# and aa[0]<UB:
            if_collide=1
        # cld_pts = pcl[aa]
    return L,if_collide

#pythran export line_sf(float64[], float64[], float64[], float64)
# def check_fov(np.ndarray[double, ndim=1] qry_pt,rtree,list cam_param,list cam_ori,np.ndarray[double, ndim=1] uav_pose):
#     cdef list idx = []
#     cdef list ids
#     cdef int i = 0
#     cdef it
#     cdef float s_len,depth,h_ang,v_ang
#     cdef np.ndarray[double, ndim=1] body_qrypt
#     # cdef np.ndarray[double, ndim=2] e2b
#     cdef double[:,:] e2b
#     # cdef double[:] body_qrypt
#     depth,h_ang,v_ang = cam_param
#     ct_pos = uav_pose[0:3]
#     ct_ori = uav_pose[3::]
#     e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
#     body_qrypt = np.matmul(e2b,qry_pt).T
#     # print("body_qrypt",body_qrypt)
#     s_len = depth/1.732
#     # dis_qry = np.linalg.norm(qry_pt) #dis_qry<  depth and 
#     if abs(Angle2(body_qrypt[0:2], np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2], np.array([1,0])))<v_ang:
#         return 1
#     # else:
#     #     # print("dis_qry", dis_qry)
#     #     return 0
#     qry_pt = qry_pt+ct_pos
#     ids = list(rtree.intersection((qry_pt[0]-s_len, qry_pt[1]-s_len, qry_pt[2]-s_len, qry_pt[0]+s_len,qry_pt[1]+s_len,qry_pt[2]+s_len)))
    
#     for it in ids:
#         ct_pos = cam_ori[it][0:3]
#         ct_ori = cam_ori[it][3::]
#         e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
#         body_qrypt = np.matmul(e2b,qry_pt-ct_pos).T
#         if abs(Angle2(body_qrypt[0:2], np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2], np.array([1,0])))<v_ang:
#             return 1
#     return 0
   