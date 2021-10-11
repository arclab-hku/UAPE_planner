#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 15 10:27:35 2021

@author: chen
"""

from sympy import *
from scipy.optimize import fsolve
from time import time
p0 = [2.0,2.0,2.0]
pg = [2.5,2.3,2.0]
vn = [-0.4,0.2,0.0]
an = [0.0,0.0,0.0]
jm = 8.0
# t = symbols('t')
# jx = symbols('jx')
# jy = symbols('jy')
# jz = symbols('jz')

# y= solve([p0[0]+vn[0]*t+1/2*an[0]*t**2+1/6*jx*t**3-pg[0],p0[1]+vn[1]*t+1/2*an[1]*t**2+1/6*jy*t**3-pg[1],
#           p0[2]+vn[2]*t+1/2*an[2]*t**2+1/6*jz*t**3-pg[2],jx**2+jy**2+jz**2-jm**2], [t,jx,jy,jz])

def func(x,p0,pg,vn,an,jm):
    
    t,jx,jy,jz=x
    return [p0[0]+vn[0]*t+1/2*an[0]*t**2+1/6*jx*t**3-pg[0],p0[1]+vn[1]*t+1/2*an[1]*t**2+1/6*jy*t**3-pg[1],
          p0[2]+vn[2]*t+1/2*an[2]*t**2+1/6*jz*t**3-pg[2],jx**2+jy**2+jz**2-jm**2]

t1= time()
r = fsolve(func,[0, 0, 0,0],args=(p0,pg,vn,an,jm),xtol=1e-2,maxfev=100)
print(r,time()-t1)