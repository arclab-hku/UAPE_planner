#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon May 31 16:45:18 2021

@author: chen
"""

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import time
img1 = cv.imread('box.png',cv.IMREAD_GRAYSCALE)[40:183,40:264]          # queryImage
img2 = cv.imread('box_in_scene.png',cv.IMREAD_GRAYSCALE)[100:284,100:412] # trainImage
# Initiate SIFT detector
tt = 0
for i in range(100):
    t1 = time.time()
    orb = cv.ORB_create()
    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)
    # FLANN parameters
    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 12, # 12
                   key_size =20,     # 20
                   multi_probe_level = 2) #2
    search_params = dict(checks=50)   # or pass empty dictionary
    flann = cv.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    # Need to draw only good matches, so create a mask
    # matchesMask = [[0,0] for i in range(len(matches))]
    # # ratio test as per Lowe's paper
    # for i,(m,n) in enumerate(matches):
    #     if m.distance < 0.7*n.distance:
    #         matchesMask[i]=[1,0]
    good = []
    for m_n in matches:
      if len(m_n) != 2:
        continue
      (m,n) = m_n
      if m.distance < 0.6*n.distance:
        good.append(m)
    tt += time.time()-t1

print("time cost",tt/100,len(good))
# draw_params = dict(matchColor = (0,255,0),
#                    singlePointColor = (255,0,0),
#                    matchesMask = matchesMask,
#                    flags = cv.DrawMatchesFlags_DEFAULT)
img3 = cv.drawMatches(img1,kp1,img2,kp2,good[:30],None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
plt.imshow(img3,),plt.show()