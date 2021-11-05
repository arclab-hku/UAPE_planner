#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  3 12:53:00 2021

@author: chen
"""



cimport cython

from cython.parallel import prange, parallel

import numpy as np
# import math as mt
from time import time as time
# from utils import earth_to_body_frame
# import linesafe
cimport numpy as np
from itertools import chain


# DTYPE = np.int
PI = 3.14159265358979
ctypedef np.int_t ITYPE_t
ctypedef np.float64_t FTYPE_t

cdef extern from "math.h":
    # float cosf(float theta)
    double atan2(double theta1,double theta2)
    double acos(float theta)
    double cos(float theta)
    double sin(float theta)
    double asin(float theta)
    double sqrt(double x)
    # double abs(double a)
    
cdef inline body_to_earth_frame(double ii, double jj, double kk):
    Cii, Cjj, Ckk=cos(ii), cos(jj), cos(kk)
    Sii, Sjj, Skk=sin(ii), sin(jj), sin(kk)
    cdef double[:, :] R
    R = np.array([[Ckk * Cjj, Ckk * Sjj * Sii - Skk * Cii, Ckk * Sjj * Cii + Skk * Sii],
                [Skk * Cjj, Skk * Sjj * Sii + Ckk * Cii, Skk * Sjj * Cii - Ckk * Sii],
                [-Sjj, Cjj * Sii, Cjj * Cii]])
    return R

cdef inline earth_to_body_frame(double ii, double jj, double kk):
    return body_to_earth_frame(ii, jj, kk).T

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)
cdef inline double Angle2(np.ndarray[FTYPE_t, ndim=1] x, np.ndarray[long, ndim=1] y):
    cdef double x1,y1,x2,y2,angle
    x1 = x[0]
    y1 = x[1]
    x2 = y[0]
    y2 = y[1]

    if (x1 == 0 and y1 ==0)or(x2 == 0 and y2 ==0):
        angle =0.0
    else:
        angle=atan2(x1,y1)-atan2(x2,y2)
    if angle<0:
        angle = -angle
    if (2*PI>angle>PI):
        angle= 2*PI - angle
    else:
        if 2*PI<angle:
            angle= angle - 2*PI
    if y1<0:
        angle = -angle
    # elif angle<-pi:
    #     angle=angle+2*pi
    # print('angle:',-angle)
    return angle

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)
cdef inline line_sf (np.ndarray[FTYPE_t, ndim=2] pcl,np.ndarray[FTYPE_t, ndim=1] p1, np.ndarray[FTYPE_t, ndim=1] p2, double SA, int return_pts): #
    cdef int if_collide
    cdef double L,UB
    # cdef np.ndarray[DTYPE_t, ndim=1] ids
    cdef np.ndarray[FTYPE_t, ndim=2] aa,bb
    cdef np.ndarray[FTYPE_t, ndim=1] l1s,l2s,jt
    cdef tuple ids
    if_collide=0
    # l1s = np.linalg.norm(X-p1,axis=1)
   
    L=np.linalg.norm(p2-p1)
    UB=(SA+(SA**2+L**2)**0.5)
    # xp1=np.array([min(p1[0],p2[0]),min(p1[1],p2[1]),min(p1[2],p2[2])])
    # xp2=np.array([max(p1[0],p2[0]),max(p1[1],p2[1]),max(p1[2],p2[2])])
    # index = np.where((xp2[0]+SA > pcl[:,0]) & (xp1[0]-SA < pcl[:,0]) &(xp2[1]+SA > pcl[:,1]) & (xp1[1]-SA < pcl[:,1]) &
    #                   (xp2[2]+SA > pcl[:,2]) &(xp1[2]-SA < pcl[:,2]))
   
    # pcl = pcl[index]

    if len(pcl):
        aa = (pcl-p1)**2
        bb= (pcl-p2)**2
        l1s = np.sum(aa,axis=1) #a^2
        l2s = np.sum(bb,axis=1) #b^2
        jt = 2*(L**2)*(l1s+l2s)-(l1s-l2s)**2-L**4
       
        ids = np.where((l1s**0.5+l2s**0.5<UB)&(jt<4*(L*SA)**2))
        if len(ids[0]):# and aa[0]<UB:
            if_collide=1
    if return_pts:
        return L,if_collide,pcl[ids]
    else:
        return L,if_collide

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)
cdef inline line_sfc (np.ndarray[FTYPE_t, ndim=2] pcl,np.ndarray[FTYPE_t, ndim=1] p1, np.ndarray[FTYPE_t, ndim=1] p2, double SA): #
    cdef int if_collide=0
    cdef double L,UB,jt,l1s,l2s
    # cdef np.ndarray[DTYPE_t, ndim=1] ids
    # cdef np.ndarray[FTYPE_t, ndim=2] aa,bb
    cdef np.ndarray[FTYPE_t, ndim=1] pt,aa,bb,p2_1
    # cdef tuple ids
    
    # l1s = np.linalg.norm(X-p1,axis=1)
    p2_1 = p2-p1
    L=sqrt(p2_1[0]*p2_1[0]+p2_1[1]*p2_1[1]+p2_1[2]*p2_1[2])
    UB=(SA+(SA**2+L**2)**0.5)
    # xp1=np.array([min(p1[0],p2[0]),min(p1[1],p2[1]),min(p1[2],p2[2])])
    # xp2=np.array([max(p1[0],p2[0]),max(p1[1],p2[1]),max(p1[2],p2[2])])
    # index = np.where((xp2[0]+SA > pcl[:,0]) & (xp1[0]-SA < pcl[:,0]) &(xp2[1]+SA > pcl[:,1]) & (xp1[1]-SA < pcl[:,1]) &
    #                   (xp2[2]+SA > pcl[:,2]) &(xp1[2]-SA < pcl[:,2]))
   
    # pcl = pcl[index]

    for pt in pcl:
        
        aa = (pt-p1)**2
        bb= (pt-p2)**2
        l1s = aa[0]+aa[1]+aa[2] #a^2
        l2s = bb[0]+bb[1]+bb[2] #b^2
        jt = 2*(L*L)*((l1s+l2s)-0.5*L*L)-(l1s-l2s)*(l1s-l2s)

        if (sqrt(l1s)+sqrt(l2s)<UB) and (jt<4*(L*SA)*(L*SA)):
            if_collide=1
            break

    return L,if_collide
        
@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True) 
cdef gen_samples (np.ndarray[FTYPE_t, ndim=1] goal,int r_num,double dr,list angle_seeds,int circle_num):  #r_list: the index of radius of the collide part of the line uav-goal
    # samples = []
    # r_num = max(1,r_num)
    l=max(dr,dr*r_num)
    ang_g = np.array([Angle2(goal[0:2], np.array([1,0])),Angle2(np.array([np.linalg.norm(goal[0:2]),goal[2]]), np.array([1,0]))])
    print("r_num,circle_num",ang_g,r_num,circle_num,len(angle_seeds),len(angle_seeds[r_num]))
    angles = angle_seeds[r_num][circle_num]+ang_g
    samples = np.array([l*np.cos(angles[:,0])*np.cos(angles[:,1]),
                        l*np.sin(angles[:,0])*np.cos(angles[:,1]),
                        l*np.sin(angles[:,1])]).T
    print("samples:",samples)
    return samples

def gen_angle_seed (double dis,double dr,int segs):
    seeds = []
    for i in range(segs):
        alpha = 2*asin(dis/(2*max(1,i)*dr))
        itnum = int(max(1,np.rint(PI/1.1/alpha))+1)
        seed = []
        for j in range(1,itnum):
            # line_1= np.arange(0,-j-0.1,-1)*alpha
            line_2= np.arange(0,j+0.1)*alpha
            line_1 = -line_2
            line = np.array(list(chain(*zip(line_1, line_2))))[1::]
            line1 = np.c_[line,np.ones(2*j+1)*(j*alpha)]
            line2 = np.c_[np.ones(2*j-1)*(j*alpha),line[1:-1]]
            
            line3 = np.c_[line,-np.ones(2*j+1)*(j*alpha)]
            line4 = np.c_[-np.ones(2*j-1)*(j*alpha),line[1:-1]]
            line24 = np.array(list(chain(*zip(line2, line4))))
            seed.append(np.r_[line24,line1,line3])
        seeds.append(seed)
    return seeds

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)
cdef get_node(np.ndarray[FTYPE_t, ndim=3] wpts,np.ndarray[FTYPE_t, ndim=1] new_node, double dis_layer):
    cdef list idx =[]
    cdef int i,index,len_w
    # cdef np.ndarray[FTYPE_t, ndim=2] last_nodes
    cdef np.ndarray[FTYPE_t, ndim=1] dists
    len_w =len(wpts)
    # print("wpts",wpts,len_w)
    # if len(wpts) >1:
    #     for i in range(len(wpts[len_w-1])):
    #         # print("i",i)
    #         if Angle1(new_node-wpts[len_w-1,i],wpts[len_w-2,i]-wpts[len_w-1,i]) > PI/2:
    #             idx.append(i)
    #     if len(idx)==0:
    #         return -1,[]
    #     last_nodes = wpts[len_w-1,idx]
    # else:
    #     last_nodes = wpts[len_w-1,:]
    # if len(last_nodes)==1:
    #     return 0,last_nodes[0]
    # else:
    last_nodes = wpts[len_w-1]
    dists = np.linalg.norm(last_nodes-new_node,axis=1)
    index = np.argmin(dists)
  #  if dists[index] > dis_layer/cos(PI/3):
  #      return -1,last_nodes[0]
    return index, last_nodes[index]

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)    
cpdef int check_path (np.ndarray[FTYPE_t, ndim=2] pcl,np.ndarray[FTYPE_t, ndim=2] waypoints,double SA):
    cdef int if_safe = 1
    cdef np.ndarray[FTYPE_t, ndim=1] p1,p2
    for i in range(len(waypoints)-1):
        p1,p2=waypoints[i:i+2]
        if line_sf (pcl,p1,p2,SA,0)[1]:
            if_safe = 0
            break
    return if_safe

# @cython.boundscheck(False)
# @cython.wraparound(False)
# @cython.nonecheck(False)
# @cython.cdivision(True) 
# @cython.profile(True)    
# cpdef int check_path (np.ndarray[FTYPE_t, ndim=2] pcl,np.ndarray[FTYPE_t, ndim=2] waypoints,double SA):
#     cdef int if_safe = 1
#     cdef np.ndarray[FTYPE_t, ndim=1] p1,p2
#     for i in range(len(waypoints)-1):
#         p1,p2=waypoints[i:i+2]
#         if line_sf (pcl,p1,p2,SA,0)[1]:
#             if_safe = 0
#             break
#     return if_safe

# def check_path(a,b,c):
#     return  _check_path(a,b,c)
# cdef list check_fov(np.ndarray[FTYPE_t, ndim=1] qry_pt,rtree,list cam_param,list cam_ori,np.ndarray[FTYPE_t, ndim=1] uav_pose):
#     depth,h_ang,v_ang = cam_param
#     ct_pos = uav_pose[0:3]
#     ct_ori = uav_pose[3::]
#     e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
#     body_qrypt = np.matmul(e2b,qry_pt).T
#     s_len = depth/1.732
#     idx = []
#     # dis_qry = np.linalg.norm(qry_pt) #dis_qry<  depth and 
#     if abs(Angle2(body_qrypt[0:2],  np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2],  np.array([1,0])))<v_ang:
#         return 1
#     # else:
#     #     print("dis_qry", dis_qry)
#     #     return 0
#     qry_pt = qry_pt+ct_pos
#     ids = list(rtree.intersection((qry_pt[0]-s_len, qry_pt[1]-s_len, qry_pt[2]-s_len, qry_pt[0]+s_len,qry_pt[1]+s_len,qry_pt[2]+s_len)))
    
#     for it in ids:
#         ct_pos = cam_ori[it][0:3]
#         ct_ori = cam_ori[it][3::]
#         e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
#         body_qrypt = np.matmul(e2b,qry_pt-ct_pos).T
#         if abs(Angle2(body_qrypt[0:2],  np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2], np.array([1,0])))<v_ang:
#             return 1
#     return 0
@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)
cdef list check_fov(np.ndarray[FTYPE_t, ndim=2] qry_pt_list,rtree,list cam_param,list cam_ori,np.ndarray[FTYPE_t, ndim=1] uav_pose):

    cdef list idx = []
    cdef list ids
    cdef int i = 0
    cdef it
    cdef float s_len
    cdef np.ndarray[FTYPE_t, ndim=1] qry_pt,body_qrypt
    cdef double[:,:] e2b
    depth,h_ang,v_ang = cam_param
    ct_pos = uav_pose[0:3]
    ct_ori = uav_pose[3::]
    e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
    for qry_pt in qry_pt_list:
        body_qrypt = np.matmul(e2b,qry_pt).T
        s_len = depth/1.732
        
        # dis_qry = np.linalg.norm(qry_pt) #dis_qry<  depth and 
        if abs(Angle2(body_qrypt[0:2],  np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2],  np.array([1,0])))<v_ang:
            idx.append(i)
            i+=1
            continue
            # return 1
        # else:
        #     print("dis_qry", dis_qry)
        #     return 0
        qry_pt = qry_pt+ct_pos
        ids = list(rtree.intersection((qry_pt[0]-s_len, qry_pt[1]-s_len, qry_pt[2]-s_len, qry_pt[0]+s_len,qry_pt[1]+s_len,qry_pt[2]+s_len)))
        
        for it in ids:
            ct_pos = cam_ori[it][0:3]
            ct_ori = cam_ori[it][3::]
            e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
            body_qrypt = np.matmul(e2b,qry_pt-ct_pos).T
            if abs(Angle2(body_qrypt[0:2],  np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2], np.array([1,0])))<v_ang:
                idx.append(i)
                break
        i+=1
    return idx
        # return 0
@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)
cdef list check_fov3(np.ndarray[FTYPE_t, ndim=2] qry_pt_list,list ids,list cam_param,list cam_ori,np.ndarray[FTYPE_t, ndim=1] uav_pose):

    cdef list idx = []
    # cdef list ids
    cdef int i = 0
    cdef it
    cdef float s_len
    cdef np.ndarray[FTYPE_t, ndim=1] qry_pt,body_qrypt
    cdef double[:,:] e2b
    depth,h_ang,v_ang = cam_param[0:3]
    ct_pos = uav_pose[0:3]
    ct_ori = uav_pose[3::]
    e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
    for qry_pt in qry_pt_list:
        body_qrypt = np.matmul(e2b,qry_pt).T
        s_len = depth/1.732
        
        # dis_qry = np.linalg.norm(qry_pt) #dis_qry<  depth and 
        if abs(Angle2(body_qrypt[0:2],  np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2],  np.array([1,0])))<v_ang:
            idx.append(i)
            i+=1
            continue
            # return 1
        # else:
        #     print("dis_qry", dis_qry)
        #     return 0
        qry_pt = qry_pt+ct_pos
        # ids = list(rtree.intersection((qry_pt[0]-s_len, qry_pt[1]-s_len, qry_pt[2]-s_len, qry_pt[0]+s_len,qry_pt[1]+s_len,qry_pt[2]+s_len)))
        
        for it in ids:
            ct_pos = cam_ori[it][0:3]
            ct_ori = cam_ori[it][3::]
            e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
            body_qrypt = np.matmul(e2b,qry_pt-ct_pos).T
            if np.linalg.norm(qry_pt-ct_pos) > depth:
                continue
            if abs(Angle2(body_qrypt[0:2],  np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2], np.array([1,0])))<v_ang:
                idx.append(i)
                break
        i+=1
    return idx

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True) 
cdef double cross_dot(np.ndarray[FTYPE_t, ndim=1] a,np.ndarray[FTYPE_t, ndim=1] b,np.ndarray[FTYPE_t, ndim=1] c):
    cdef list cs=[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    cdef double vi = cs[0]*c[0]+cs[1]*c[1]+cs[2]*c[2]
    if vi < 0:
        vi = -vi
    # print("v in fuction:",cs,c,vi,cs[0]*c[0]+cs[1]*c[1]+cs[2]*c[2])
    return vi
    # return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True) 
cdef float dot(list a,np.ndarray[FTYPE_t, ndim=1] b):
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
    
@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True) 
cdef list check_fov2(np.ndarray[FTYPE_t, ndim=2] samples,list ids,list cam_param,list cam_rec,np.ndarray[FTYPE_t, ndim=1] uav_pose,float near_rat):
# use hybrid product of vectors for checking
    cdef list idx = []
    # cdef list c1,c2,c3,c4,c5,c6
    if near_rat < 0.5:
        return [list(np.arange(len(samples))),[]]
    cdef list failed = []
    cdef int i = 0
    cdef int it
    cdef double v1,v2,v3,v4,v5,v6
    cdef float s_len,v_cam,s_cam_pt
    cdef np.ndarray[FTYPE_t, ndim=1] qry_pt,body_qrypt
    cdef np.ndarray[FTYPE_t, ndim=2] fov_shape,reduce,qry_pt_list
    # cdef double[:,:] e2b
    qry_pt_list = samples + uav_pose[0:3]
    s_cam_pt = cam_param[3]
    v_cam = cam_param[4]
    for qry_pt in qry_pt_list:
        # body_qrypt = np.matmul(e2b,qry_pt).T
        # s_len = depth/1.732
        
        # # dis_qry = np.linalg.norm(qry_pt) #dis_qry<  depth and 
        # if abs(Angle2(body_qrypt[0:2],  np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2],  np.array([1,0])))<v_ang:
        #     idx.append(i)
        #     i+=1
        #     continue
            # return 1
        # else:
        #     print("dis_qry", dis_qry)
        #     return 0
        # qry_pt = qry_pt+uav_pos[0:3]
        # ids = list(rtree.intersection((qry_pt[0]-s_len, qry_pt[1]-s_len, qry_pt[2]-s_len, qry_pt[0]+s_len,qry_pt[1]+s_len,qry_pt[2]+s_len)))
        # print("ids",ids)
        for it in ids:
            fov_shape = cam_rec[it]
            reduce = qry_pt - fov_shape
            # v1 = abs(np.dot(np.cross(qry_pt - fov_shape[0], qry_pt - fov_shape[1]), qry_pt - fov_shape[2]))
            # v2 = abs(np.dot(np.cross(qry_pt - fov_shape[0], qry_pt - fov_shape[2]), qry_pt - fov_shape[3]))
            # v3 = abs(np.dot(np.cross(qry_pt - fov_shape[0], qry_pt - fov_shape[3]), qry_pt - fov_shape[4]))
            # v4 = abs(np.dot(np.cross(qry_pt - fov_shape[0], qry_pt - fov_shape[4]), qry_pt - fov_shape[1]))
            # v5 = abs(np.dot(np.cross(qry_pt - fov_shape[1], qry_pt - fov_shape[2]), qry_pt - fov_shape[3]))
            # v6 = abs(np.dot(np.cross(qry_pt - fov_shape[3], qry_pt - fov_shape[4]), qry_pt - fov_shape[1]))
            v1 = cross_dot(reduce[0],reduce[1],reduce[2])
            v2 = cross_dot(reduce[0],reduce[2],reduce[3])
            v3 = cross_dot(reduce[0],reduce[3],reduce[4])
            v4 = cross_dot(reduce[0],reduce[4],reduce[1])
            v5 = cross_dot(reduce[1],reduce[2],reduce[3])
            v6 = cross_dot(reduce[3],reduce[4],reduce[1])
            # v1 = abs(dot(c1,reduce[2]))
            # v1 = abs(dot(c2,reduce[3]))
            # v1 = abs(dot(c3,reduce[4]))
            # v1 = abs(dot(c4,reduce[1]))
            # v1 = abs(dot(c5,reduce[3]))
            # v1 = abs(dot(c6,reduce[1]))
            # print("fov_shape",reduce,"v1-v6",[v1,v2,v3,v4,v5,v6],"sum,v_cam:",[sum([v1,v2,v3,v4,v5,v6])/6,v_cam],"qry_pt:",qry_pt)
            if ((v1+v2+v3+v4+v5+v6)/6-v_cam)/v_cam < 0.005:
                idx.append(i)
                break
            # elif it == ids[-1]:
            #     failed.append(i)
        if len(idx)==0 or  i != idx[len(idx)-1]:
            failed.append(i)
        i=i+1
    return [idx,failed]

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)       
cpdef opt_path (np.ndarray[FTYPE_t, ndim=3] wpts,np.ndarray[FTYPE_t, ndim=2] pcl,double SA):
    cdef int i
    if len(wpts)>3:
        i=1
        while i < len(wpts)-1:
            while not line_sf (pcl,wpts[i-1,0,:] ,wpts[i+1,0,:],SA,0)[1]:
                wpts = np.delete(wpts,i,axis=0)
                print("delete one waypoint!",i)
                if i >= len(wpts)-2:
                    break
            i+=1
                
    return wpts

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)  
cdef get_dyn_node (np.ndarray[FTYPE_t, ndim=3] dyn_obs, np.ndarray[FTYPE_t, ndim=2] V_A, np.ndarray[FTYPE_t, ndim=1] last_pos, 
                   np.ndarray[FTYPE_t, ndim=1] it,double last_t):
    cdef np.ndarray[FTYPE_t, ndim=1] c1,c2,c3,ct_pos
    cdef np.ndarray[FTYPE_t, ndim=2] nun = np.array([[]])
    cdef double am = 6.0
    c1 = V_A[0]
    c2 = V_A[1]/2
    cdef list t = [],tt = []
    cdef double b_Ac1,b_Ac2
    # for i in range(3):
    #     t = []
    #     b_Ac1 = c1[i]**2 + (2/3*c2[i]+am/6)*(it[i]-last_pos[i])
    #     b_Ac2 = c1[i]**2 + (2/3*c2[i]-am/6)*(it[i]-last_pos[i])
    #     if b_Ac1 >0:
    #         t.append((-c1[i] + (b_Ac1)**0.5)/(2*(2/3*c2[i]+am/6)))
    #         t.append((-c1[i] - (b_Ac1)**0.5)/(2*(2/3*c2[i]+am/6)))
    #     if b_Ac2 >0:
    #         t.append((-c1[i] + (b_Ac2)**0.5)/(2*(2/3*c2[i]-am/6)))
    #         t.append((-c1[i] - (b_Ac2)**0.5)/(2*(2/3*c2[i]-am/6)))
    #     # t = np.array()
    #     tt.append(max(np.array(t)[np.where(np.array(t)>0)[0]]))
    #     print("tt,t:",tt,t)

        
    # cdef double T = max(tt)
    cdef double T = np.linalg.norm(it-last_pos)/am*2
    c3 = (it-last_pos - c1*T - c2*T**2)/(T**3)
    cdef double fac = max([((max(abs(V_A[1]+6*T*c3)))/am)**0.6,(max(abs(V_A[0]+2*T*c2+3*T**2*c3)))/am,1])
    print("C3-before:",c3,V_A[1]+6*T*c3,V_A[0],V_A[1],"fac:",fac)
    T = T*fac
    c3 = (it-last_pos - c1*T - c2*T**2)/(T**3)
    print("C3:",c3,V_A[1]+6*T*c3,V_A[0],V_A[1])
    cdef np.ndarray[FTYPE_t, ndim=2] ct_center
    cdef double ct

    return np.array([it,V_A[0]+2*T*c2+3*T**2*c3,V_A[1]+6*T*c3,[last_t+T]*3]),False


    if len(dyn_obs[0,0]) == 0:
       # return np.array([last_pos +  c1*T + c2*T**2 +  c3*T**3,V_A[0]+2*T*c2+3*T**2*c3,V_A[1]+6*T*c3,[last_t+T]*3]),False
       return np.array([it,V_A[0]+2*T*c2+3*T**2*c3,V_A[1]+6*T*c3,[last_t+T]*3]),False
    else:
        for i in range(1,int(T/0.1)+2):
            ct = 0.1*i
            ct_center = (ct+last_t)* dyn_obs[:,1,:] + dyn_obs[:,0,:]
            ct_pos = last_pos +  c1*ct + c2*ct**2 +  c3*ct**3
            if ((abs(ct_pos - ct_center) - dyn_obs[:,2,:]) < 0).all():
                print("dyn check fail!",dyn_obs, ct_center,ct,ct_pos)
                return nun,True
            if i == int(T/0.1)+1:
                #return np.array([ct_pos,V_A[0]+2*T*c2+3*T**2*c3,V_A[1]+6*T*c3,[last_t+T]*3]),False
                return np.array([it,V_A[0]+2*T*c2+3*T**2*c3,V_A[1]+6*T*c3,[last_t+T]*3]),False
@cython.boundscheck(False)
@cython.nonecheck(False)
@cython.cdivision(True) 
@cython.profile(True)
cpdef get_path (np.ndarray[FTYPE_t, ndim=2] pcl,np.ndarray[FTYPE_t, ndim=1] goal,double dr,double sense_range,double SA,
                list seeds,int path_num,rtree,list ids,list cam_param,list cam_ori,list cam_rec,
                np.ndarray[FTYPE_t, ndim=1]uav_pose, double max_height,np.ndarray[FTYPE_t, ndim=3] dyn_obs,
                np.ndarray[FTYPE_t, ndim=2] V_A):  # returns if fly straightly to goal, and the waypoints
    
    print("in function")
    cdef float segs = np.rint(sense_range/dr) 
    # st_pts = goal/np.linalg.norm(goal)*dr
    cdef list which_collide = []
    # cdef list samples  = []
    cdef list all_sample = [[],[]]
    cdef list ct_nodes,costs,last_index,idx_fov,idx_sf,failed
    cdef double L_goal,t1,t2,t01,t11,cost,t12,t13,t14,t3,dis_layer
    cdef int if_cld0,check_num,j,circle_num,jj,if_cld,index,iter_num,check_fov_num,which_len,ii,if_dyn_cld = 1
    cdef np.ndarray[FTYPE_t, ndim=2] pts,samples,cost_list,new_costs, dyn_node
    cdef np.ndarray[FTYPE_t, ndim=4] wpts,new_wpts
    cdef np.ndarray[FTYPE_t, ndim=3] last_nodes,final_waypoints
    cdef np.ndarray[float, ndim=1] time_check = np.zeros(5,dtype=np.float32)
    cdef np.ndarray[FTYPE_t, ndim=1] node,it
    cdef list final_list = []
    cdef list final_cost = []
    
    t1 = time()
    L_goal,if_cld0,pts = line_sf(pcl,np.array([0.0,0.0,0.0]),goal,SA,1)
    if if_cld0:
        for pt in pts:
            which_collide.append(int(np.dot(pt,goal)/L_goal/dr))  
    which_collide = list(np.unique(which_collide))
    print("collide:",which_collide)
    # for i in range(int(segs)):
    #     p1 = st_pts*i
    #     p2 = st_pts*(i+1)
    #     if line_sf (pcl,p1,p2,SA)[1]:
    #         which_collide.append(i)
    
    
    if not len(which_collide):
       return 1,[],[],0,0  #0 for fly curve, 1 for straight, 2 for no waypoint can be found
    elif len(which_collide)>=2 and which_collide[0]==0 and which_collide[1]==1:
        del which_collide[0]
    if len(which_collide) and which_collide[-1] == int(segs):
        del which_collide[-1]
    if len(which_collide)==0 or which_collide[-1] != int(segs)-1:
        which_collide.append(int(segs)-1)
    print("all the collide segs:",segs,which_collide)
    iter_num = 0        
    
    # wpts = []
    
   
    check_num = 0
    check_fov_num = 0
    t2 = time()
    # which_len = len(which_collide)
    # with nogil, parallel(num_threads=4):

    for j in which_collide:
        if iter_num==0:
            last_nodes = np.array([[[0,0,0],V_A[0],V_A[1],[0,0,0]]]).astype(float)
            # wpts = np.array([[np.array([0,0,0]).astype(float)]*path_num])
            wpts = np.array([[np.array([[0,0,0],V_A[0],V_A[1],[0,0,0]]).astype(float)]*path_num])
            cost_list = np.array([[0]*path_num]).astype(float)
            dis_layer = j*dr
        else:
            dis_layer = (which_collide[iter_num]-which_collide[iter_num-1])*dr
        # elif len(ct_nodes)==0:
        #     break
        # if len(samples):
        #     last_samples = samples.copy()
        circle_num=0  # 0 is for the first square-edge search
        # print("j1",j)
        
        ct_nodes=[]
        costs=[]
        last_index = []
        
        while len(ct_nodes) < path_num and circle_num<len(seeds[j]):
            t01=time()
            samples = gen_samples(goal, j, dr, seeds, circle_num)  #n*3 array for x,y,z local corridinate
            print("used goal in global",goal+uav_pose[0:3])
            print("samples size:",[np.size(samples,axis=0),np.size(samples,axis=1)])
            print("len(ct_nodes)",len(ct_nodes),"circle_num",circle_num,"len(seeds[j])",len(seeds[j]),"j",j)
            
            # ct_nodes=[]
            # costs=[]
            # last_index = []
            jj = 0
          
            time_check[0] += time()-t01
            
            t11=time()
            samples = samples[np.where(samples[:,2]<max_height)]
            # idx_fov = check_fov(samples,rtree,cam_param,cam_ori,uav_pose)
            idx_fov,failed= check_fov2(samples,ids,cam_param,cam_rec,uav_pose,j/segs)
            # samples = samples - uav_pose[0:3]
            print("failed number:",len(failed))
            # idx_fov = check_fov3(samples,ids,cam_param,cam_ori,uav_pose)
            check_fov_num += len(samples)
            all_sample[1].append(samples[idx_fov])
            all_sample[0].append(samples[failed])
            time_check[1]+= time()-t11
            if len(idx_fov)==0:
                break
            if len(idx_fov) < len(samples)*0.2:
                circle_num = int(1e5)
            # idx_sf = line_sf (pcl,node,it,SA,0)
            # time_check[2]+=time()-t12
            # print("samples used for collision check:", samples[idx_fov],uav_pose[0:3],goal)
            for it in samples[idx_fov]:
                # t11=time()
                # if not check_fov(it,rtree,cam_param,cam_ori,uav_pose):
                #     print("FOV check fail",jj,it)
                #     all_sample[0].append(it)
                #     jj+=1
                   
                #     time_check[1]+= time()-t11
                #     continue
                # all_sample[1].append(it)
                t12 = time()
                
                index,node = get_node(wpts[:,:,0,:],it,dis_layer)
                if index<0:
                    continue
                
                # time_check[1]+= t12-t11
                cost,if_cld = line_sf (pcl,node,it,SA,0)
                
                
                print("node:",node,it,if_cld)
                if not if_cld : 
                    print("wpts:",wpts,wpts[:,:,0,:])
                    dyn_node, if_dyn_cld = get_dyn_node (dyn_obs,wpts[-1,index,1:3,:],node,it,wpts[-1,index,-1,0])  #V_A should be the state of the last node
                    print("dyn_node:",dyn_node,if_dyn_cld)
                # cost,if_cld = line_sfc (pcl,node,it,SA)
                t13=time()
                
                check_num +=1
                if not if_dyn_cld:
                    ct_nodes.append(dyn_node)
                    costs.append(cost)
                    last_index.append(index)
                if len(ct_nodes)>=path_num:
                    iter_num+=1
                    last_nodes = np.array(ct_nodes).copy()
                    new_wpts = wpts
                    new_costs = cost_list
                    for pt in range(path_num):
                        new_wpts[:,pt] = wpts[:,last_index[pt]]
                        new_costs[:,pt] = cost_list[:,last_index[pt]]
                    # print(new_wpts,np.array([ct_nodes]))
                    new_wpts = np.r_[new_wpts,np.array([ct_nodes])]
                    new_costs = np.r_[new_costs,np.array([costs])]
                    wpts = new_wpts
                    cost_list = new_costs
                    time_check[2:4]+= np.array([t13-t12,time()-t13])
                    break
                time_check[2:4]+= np.array([t13-t12,time()-t13])
            # if len(ct_nodes)>=path_num:
            #     break
            # else:
            circle_num += 1
            # if there are less nodes than path_num in this layer
        # print("j2",j)
        t14 = time()
        if len(ct_nodes) < path_num and len(ct_nodes):
            iter_num+=1
            print("nodes number is not enough!",path_num-len(ct_nodes))
            while len(ct_nodes) < path_num:
                ct_nodes += ct_nodes[-(path_num-len(ct_nodes))::]
                costs += costs[-(path_num-len(costs))::]
                last_index += last_index[-(path_num-len(last_index))::]
            if len(ct_nodes) > path_num:
                ct_nodes = ct_nodes[0:path_num]
                costs = costs[0:path_num]
                last_index = last_index[0:path_num]
            new_wpts = wpts
            new_costs = cost_list
            for pt in range(path_num):
                new_wpts[:,pt] = wpts[:,last_index[pt]]
                new_costs[:,pt] = cost_list[:,last_index[pt]]
            new_wpts = np.r_[new_wpts,np.array([ct_nodes])]
            new_costs = np.r_[new_costs,np.array([costs])]
            wpts = new_wpts
            cost_list = new_costs
          
        elif len(ct_nodes) ==0:
            return 2,[],[],check_num,check_fov_num
        time_check[-1] += time()-t14
        # print("j3",j)
    t3=time()
    
    cdef int i
    cdef int m=0
    cdef int if_goal_reachable = 0
   # print (len(wpts[:,-1,0,:]),len(cost_list[0]))
    while (not if_goal_reachable and m<3):
        i=0
        for lnd in wpts[-1,:,0,:]:
            #cost = np.linalg.norm(lnd-goal) 
            cost,if_cld = line_sf(pcl,lnd,goal,SA,0)
        
            if not if_cld:
                if_goal_reachable = 1
                final_list.append(i)
                final_cost.append(cost+sum(np.array(cost_list)[:,i]))
        if (not if_goal_reachable):
                
            goal = (np.linalg.norm(goal)-SA)*goal/np.linalg.norm(goal)
            if (m==2):
                i= np.argmin(np.linalg.norm(wpts[-1,:,0,:]-goal,axis=1))
                final_list.append(i)
                final_cost.append(cost+sum(np.array(cost_list)[:,i]))
                break
            i+=1
        m+=1
    
    print("path_ind,costs",path_ind,final_cost,len(wpts[-1]))
    
    cdef int path_ind1,path_ind
    cdef dyn_node_list = []
    path_ind1 = final_list[np.argmin(final_cost)]
    if_dyn_cld = True
    for fct in final_cost:
        path_ind = final_list[np.argmin(final_cost)]
        final_cost[np.argmin(final_cost)] = 1e5
        last_2nd = wpts[-1,path_ind]
        dyn_node, if_dyn_cld = get_dyn_node (dyn_obs,last_2nd[1:3,:],last_2nd[0,:],goal,last_2nd[-1,0])
        dyn_node_list.append(dyn_node)
        if not if_dyn_cld:
            final_waypoints = np.r_[wpts[:,path_ind],np.array([dyn_node])]
            break
    if if_dyn_cld:
        # print("wpts,node(size):",wpts[:,path_ind1],np.array([dyn_node_list[0]]))
        final_waypoints = np.r_[wpts[:,path_ind1],np.array([[goal,[0,0,0],[0,0,0],[0,0,0]]])]
    final_waypoints = opt_path(final_waypoints,pcl,SA)
    print("time cost 123:",time()-np.array([t1,t2,t3]),time_check,sum(time_check))
    return 0,final_waypoints,all_sample,check_num,check_fov_num
        