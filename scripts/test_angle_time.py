#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 27 10:44:39 2021

@author: chen
"""
from math import *
import time
import numpy as np
def Angle2( x,  y):
    angle = 0.0;
    x1 = x[0]
    y1 = x[1]
    x2 = y[0]
    y2 = y[1]

    if (x1 == 0 and y1 ==0)or(x2 == 0 and y2 ==0):
        angle =0.0
    else:
        angle=atan2(x1,y1)-atan2(x2,y2)
        # print(angle)
    if angle<0:
        angle = -angle
    if 2*pi>angle>pi:
        angle= 2*pi - angle
    elif 2*pi<angle:
        angle= angle - 2*pi
    # elif angle<-pi:
    #     angle=angle+2*pi
    # print('angle:',-angle)
    return angle
def Angle1 (x,y):
    aa = sum(x**2)**0.5
    bb = sum(y**2)**0.5
    cc2 = sum((y-x)**2)
    angle = acos((aa**2+bb**2-cc2)/(2*aa*bb))
    return angle

T1, T2 = 0,0
for i in range(1000):
    x = np.random.rand(2)-0.5
    y = np.random.rand(2)-0.5
    t1 = time.time()
    ang1 = Angle1 (x,y)
    t2 = time.time()
    T1+=t2-t1
    t3 = time.time()
    ang2 = Angle2 (x,y)
    t4 = time.time()
    T2+=t4-t3
    if i%100==0:
        print(ang1,ang2)
# ang1 = Angle2([3,0], [1,0])
# print(ang1)
print("T1,T2:",T1,T2)