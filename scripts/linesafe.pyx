#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  2 19:03:58 2021

@author: chen
"""
import numpy as np
# import math as mt
from time import time as time
# from utils import earth_to_body_frame
# import linesafe
cimport numpy as np
cimport cython

DTYPE = np.int
PI = 3.14159265358979
ctypedef np.int_t DTYPE_t

# cython: initializedcheck=False
# cython: cdivision=True
# cython: boundscheck=False
# cython: wraparound=False

cdef extern from "math.h":
    # float cosf(float theta)
    double atan2(double theta1,double theta2)
    double acos(float theta)
    double cos(float theta)
    double sin(float theta)
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

cdef inline double Angle2(np.ndarray[double, ndim=1] x, np.ndarray[long, ndim=1] y):
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



cpdef line_sf (np.ndarray[double, ndim=2] pcl,np.ndarray[double, ndim=1] p1, np.ndarray[double, ndim=1] p2, double SA, int return_pts): #
    cdef int if_collide
    cdef double L,UB
    # cdef np.ndarray[DTYPE_t, ndim=1] ids
    cdef np.ndarray[double, ndim=2] aa,bb
    # cdef np.ndarray[double, ndim=1] p1,p2
    # cdef double[:, ::1] aa,bb
    cdef np.ndarray[double, ndim=1] l1s,l2s,jt
    cdef tuple ids
    if_collide=0
    # l1s = np.linalg.norm(X-p1,axis=1)
    # L=sum((p2-p1)**2)**0.5
    L=np.linalg.norm(p2-p1)
    UB=(SA+(SA**2+L**2)**0.5)
    
    # xp1=np.array([min(p1[0],p2[0]),min(p1[1],p2[1]),min(p1[2],p2[2])])
    # xp2=np.array([max(p1[0],p2[0]),max(p1[1],p2[1]),max(p1[2],p2[2])])
    # index = np.where((xp2[0]+SA > pcl[:,0]) & (xp1[0]-SA < pcl[:,0]) &(xp2[1]+SA > pcl[:,1]) & (xp1[1]-SA < pcl[:,1]) &
    #                   (xp2[2]+SA > pcl[:,2]) &(xp1[2]-SA < pcl[:,2]))
   
    # pcl = pcl[index]
    # print("L,UB",L,UB,p2,p1)
    if len(pcl):
        aa = (pcl-p1)**2
        bb= (pcl-p2)**2
        l1s = np.sum(aa,axis=1) #a^2
        l2s = np.sum(bb,axis=1) #b^2
        jt = 2*(L**2)*(l1s+l2s)-(l1s-l2s)**2-L**4
       
        ids = np.where((l1s**0.5+l2s**0.5<UB)&(jt<4*(L*SA)**2))
        if len(ids[0]):# and aa[0]<UB:
            if_collide=1
        # else:
        #     print("L1,UB1",L,UB)
    if return_pts:
        return L,if_collide,pcl[ids]
    else:
        return L,if_collide

    
cdef inline double Angle1 (np.ndarray[double, ndim=1] a,np.ndarray[double, ndim=1] b):
    cdef double angle
    angle = acos(a.dot(b)/(np.linalg.norm(a)*np.linalg.norm(b)))
    return angle


def gen_samples (np.ndarray[double, ndim=1] goal,int r_num,double dr,list angle_seeds,int circle_num):  #r_list: the index of radius of the collide part of the line uav-goal
    # samples = []
    # r_num = max(1,r_num)
    cdef double l
    cdef np.ndarray[double, ndim=1] ang_g
    cdef int i
    cdef double[:, ::1] angles
    cdef np.ndarray[double, ndim=2] samples
    # cdef double[:, :] samples
    l=max(dr,dr*r_num)
    ang_g = np.array([Angle2(goal[0:2], np.array([1,0])),Angle2(np.array([np.linalg.norm(goal[0:2]),goal[2]]), np.array([1,0]))])
    print("r_num,circle_num",r_num,circle_num,len(angle_seeds),len(angle_seeds[r_num]))
    angles = angle_seeds[r_num][circle_num]+ang_g
    # cdef int len_ang = len(angles)
    # cdef np.ndarray[double, ndim=2] samples = np.zeros([len_ang,3])
    # for i in range(len_ang):
    #     samples[i] = np.array([l*cos(angles[i,0])*cos(angles[i,1]),
    #                     l*sin(angles[i,0])*cos(angles[i,1]),
    #                     l*sin(angles[i,1])])
    # samples =  samples.T   
    samples = np.array([l*np.cos(angles[:,0])*np.cos(angles[:,1]),
                        l*np.sin(angles[:,0])*np.cos(angles[:,1]),
                        l*np.sin(angles[:,1])]).T
    # print("samples:",samples)
    return samples

# cpdef opt_path (double[:, :] wpts,double[:, :] pcl,double SA):np.ndarray[double, ndim=2]\
cpdef opt_path (np.ndarray[double, ndim=2] wpts,np.ndarray[double, ndim=2] pcl,double SA):
    cdef int i
    if len(wpts)>3:
        i=1
        while i < len(wpts)-1:
            while not line_sf (pcl,wpts[i-1] ,wpts[i+1],SA,0)[1]:
                wpts = np.delete(wpts,i,axis=0)
                print("delete one waypoint!",i)
                if i >= len(wpts)-2:
                    break
            i+=1
                
    return wpts
        
        

def get_node(np.ndarray[double, ndim=3] wpts,np.ndarray[double, ndim=1] new_node,double dis_layer):
    cdef list idx =[]
    cdef int i,index,len_w
    cdef np.ndarray[double, ndim=2] last_nodes
    cdef np.ndarray[double, ndim=1] dists
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
    last_nodes = wpts[len_w-1,:]
    dists = np.linalg.norm(last_nodes-new_node,axis=1)
    index = np.argmin(dists)
    if dists[index] > dis_layer/cos(PI/5):
        return -1,[]
    return index, last_nodes[index]


def check_fov(np.ndarray[double, ndim=1] qry_pt,rtree,list cam_param,list cam_ori,np.ndarray[double, ndim=1] uav_pose):
    cdef list idx = []
    cdef list ids
    cdef int i = 0
    cdef it
    cdef float s_len,depth,h_ang,v_ang
    cdef np.ndarray[double, ndim=1] body_qrypt
    # cdef np.ndarray[double, ndim=2] e2b
    cdef double[:,:] e2b
    # cdef double[:] body_qrypt
    depth,h_ang,v_ang = cam_param
    ct_pos = uav_pose[0:3]
    ct_ori = uav_pose[3::]
    e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
    body_qrypt = np.matmul(e2b,qry_pt).T
    # print("body_qrypt",body_qrypt)
    s_len = depth/1.732
    # dis_qry = np.linalg.norm(qry_pt) #dis_qry<  depth and 
    if abs(Angle2(body_qrypt[0:2], np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2], np.array([1,0])))<v_ang:
        return 1
    # else:
    #     # print("dis_qry", dis_qry)
    #     return 0
    qry_pt = qry_pt+ct_pos
    ids = list(rtree.intersection((qry_pt[0]-s_len, qry_pt[1]-s_len, qry_pt[2]-s_len, qry_pt[0]+s_len,qry_pt[1]+s_len,qry_pt[2]+s_len)))
    
    for it in ids:
        ct_pos = cam_ori[it][0:3]
        ct_ori = cam_ori[it][3::]
        e2b = earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
        body_qrypt = np.matmul(e2b,qry_pt-ct_pos).T
        if abs(Angle2(body_qrypt[0:2], np.array([1,0])))<h_ang and abs(Angle2(body_qrypt[::2], np.array([1,0])))<v_ang:
            return 1
    return 0

# def line_sf(a,b,c,d,e):
#     # print("b,c,d,e:",b,c,d,e)
#     return _line_sf(a,b,c,d,e)