#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 11 15:48:14 2021

@author: chen
"""
import numpy as np
# fov_shape = np.array([[0,0,0],[3,3,2],[3,-3,2],[3,-3,-2],[3,3,-2]])
fov_shape = np.array([[ 6.23106142e-03, -1.21656096e-01,  8.77866053e-01],
       [ 1.75679118e+00, -8.17884771e+00,  3.37917031e+00],
       [ 8.25352010e+00,  1.16651835e+00,  3.01378520e+00],
       [ 8.10415239e+00,  1.08182854e+00, -1.80813753e+00],
       [ 1.60742347e+00, -8.26353752e+00, -1.44275242e+00]])
# qry_pt = np.array([1,0,0.5])
qry_pt = np.array([ 3.73575205, -4.07135248,  0.91572107])
l1 = np.linalg.norm(fov_shape[1]-fov_shape[2])
l2 = np.linalg.norm(fov_shape[2]-fov_shape[3])
# s_cam_pt = abs(fov_shape[1,1]*fov_shape[1,2])*4
s_cam_pt = abs(l1*l2)
# v_cam = s_cam_pt*abs(fov_shape[1,0])/3
v_cam = s_cam_pt*abs(6)/3
v1 = abs(np.dot(np.cross(qry_pt - fov_shape[0], qry_pt - fov_shape[1]), qry_pt - fov_shape[2]))
v2 = abs(np.dot(np.cross(qry_pt - fov_shape[0], qry_pt - fov_shape[2]), qry_pt - fov_shape[3]))
v3 = abs(np.dot(np.cross(qry_pt - fov_shape[0], qry_pt - fov_shape[3]), qry_pt - fov_shape[4]))
v4 = abs(np.dot(np.cross(qry_pt - fov_shape[0], qry_pt - fov_shape[4]), qry_pt - fov_shape[1]))
v5 = abs(np.dot(np.cross(qry_pt - fov_shape[1], qry_pt - fov_shape[2]), qry_pt - fov_shape[3]))
v6 = abs(np.dot(np.cross(qry_pt - fov_shape[3], qry_pt - fov_shape[4]), qry_pt - fov_shape[1]))
v_test = sum([v1,v2,v3,v4,v5,v6])/6
if abs((v_test-v_cam)/v_cam) < 0.01:
    print("in fov!")
print(v_test,v_cam)