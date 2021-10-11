#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May 16 17:30:21 2021

@author: chen
"""
# import matplotlib
# matplotlib.use('TKAgg')
import numpy as np
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
disps = []
t2=0

# space = np.zeros((10,10,10))
space = np.zeros((10,10,10,2))
pos1 = [3,3,3]
pos2 = [4,1,0]
disp_t = np.array(pos2)-np.array(pos1)

# obj1 = np.ones((3,4,8))

j=0
pks = []
neg_noise1 = 2*0.5/(1-0.5) #0-1.5
neg_noise2 = 2*0.5/(1-0.5) #0-1.5
add_p_num = 30
t_num = 5000
for i in range(t_num):
    # obj1[np.random.randint(10,size=add_p_num),np.random.randint(10,size=add_p_num),np.random.randint(10,size=add_p_num)] = 1
    obj1 = np.random.rand(3,4,6,2)*3+1
    fm1 = space.copy()
    t1 = time.time()
    fm1[pos1[0]:pos1[0]+len(obj1), pos1[1]:pos1[1]+len(obj1[0]),pos1[2]:pos1[2]+len(obj1[0,0])] = abs(np.rint(obj1-np.random.rand(3,4,6,2)*neg_noise1))
    # fm1[np.random.randint(10,size=add_p_num),np.random.randint(10,size=add_p_num),np.random.randint(10,size=add_p_num)] = 1
    fm2 = np.zeros((max(10,pos2[0]+len(obj1)),max(10,pos2[1]+len(obj1[0])),max(10,pos2[2]+len(obj1[0,0])),2))
    fm2[np.random.randint(10,size=add_p_num),np.random.randint(10,size=add_p_num),np.random.randint(10,size=add_p_num),:] = np.random.rand(2)*3+1
    fm2[pos2[0]:pos2[0]+len(obj1), pos2[1]:pos2[1]+len(obj1[0]),pos2[2]:pos2[2]+len(obj1[0,0])] = abs(np.rint(obj1-np.random.rand(3,4,6,2)*neg_noise2))
    fm2 = fm2[0:10,0:10,0:10]
    fft1 = np.fft.rfftn(fm1)
    fft2 = np.fft.rfftn(fm2)
    ifft = np.fft.irfftn(np.conj(fft1)*fft2)
    pk1 = np.max(ifft)
    pk1_m = np.mean(ifft[np.where(ifft==pk1)[0:3]])
    # pks.append(pk1)
    # for i in range(10):
    #     if abs(pks[-1]-pks[0])/pks[0] < 0.08:
            
    pk1 = np.max(ifft)
    disp  = np.where(ifft==pk1)
    # disp = np.rint(np.mean((np.where(ifft==pk1)[0:3]),axis=1)).astype(int)
    # disp[np.where(disp==0)] = 1
    # pks = np.mean(ifft[disp[0]-1:disp[0]+2,disp[1]-1:disp[1]+2,disp[2]-1:disp[2]+2,:],axis=3)
    # pks_index = np.where(abs(pks-pk1_m)/pk1_m < 0.02)
    # err = np.max(pks_index,axis=1)
    # disp = disp + err
    # disps.append(np.reshape(disp,3))
    
    ifft[disp] = 0
    pk2 = np.max(ifft)
    disp2 = np.where(ifft==pk2)
    t2 += time.time()-t1
    pks.append([pk1,pk2])

    if abs(pk2-pk1)/pk1 < 0.08:
        disp = np.array([max(np.r_[disp[0],disp2[0]],key=abs),max(np.r_[disp[1],disp2[1]],key=abs),max(np.r_[disp[2],disp2[2]],key=abs)])
        # disp = np.array([np.mean(np.r_[disp[0],disp2[0]]),np.mean(np.r_[disp[1],disp2[1]]),np.mean(np.r_[disp[2],disp2[2]])])
       
        conv_dms = np.where(np.array(pos1)+disp>=np.array(np.shape(space)[0:3]))
        disp[conv_dms] = disp[conv_dms] - np.array(np.shape(space)[0:3])[conv_dms]
        disps.append(np.reshape(disp,3))
    else:
        # disp = np.array([max(disp[0],key=abs),max(disp[1],key=abs),max(disp[2],key=abs)])
        disp = np.array([np.mean(disp[0]),np.mean(disp[1]),np.mean(disp[2])])
        conv_dms = np.where(np.array(pos1)+disp>=np.array(np.shape(space)[0:3]))
        disp[conv_dms] = disp[conv_dms] - np.array(np.shape(space)[0:3])[conv_dms]
        disps.append(np.reshape(disp,3))
    
    if (disps[-1] != disp_t).any():
        j+=1
disp_tt = disp_t.copy()
disp_tt[2] = max(disp_tt[2],1)
print(np.mean(disps,axis=0),abs((np.mean(disps,axis=0)-disp_t)/disp_tt),t2/t_num,j/t_num)

plt.figure()
ax = plt.subplot(111,projection='3d')
ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')
idx1 = np.where(fm1!=0)[0:3]
pcl1 = np.reshape(idx1,[3,len(idx1[0])])
idx2 = np.where(fm2!=0)[0:3]
pcl2 = np.reshape(idx2,[3,len(idx2[0])])
ax.scatter(pcl1[0,:],pcl1[1,:],pcl1[2,:],c='r')
ax.scatter(pcl2[0,:],pcl2[1,:],pcl2[2,:],c='g')
# ax.scatter(pcl2[:,0],pcl2[:,1],pcl2[:,2],c='g')
# plt.show()

plt.figure()
plt.hist(ifft.flatten(), bins=40, facecolor="blue", edgecolor="black", alpha=0.7)
plt.show()
# t2=0
# t3 = 0
# for i in range(10000):
#     a = np.random.rand(10,10,10)
#     t1 = time.time()
#     b= np.fft.rfftn(a)
    
#     t2 += time.time()-t1
#     tt2 = time.time()
#     c = np.fft.irfftn(b)
#     t3+=time.time()-tt2
# print(t2,t3)