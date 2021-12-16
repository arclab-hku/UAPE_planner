#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon May 31 17:46:48 2021

@author: chen
"""

import cv2 as cv
import time
from matplotlib import pyplot as plt

img1 = cv.imread('box.png',cv.IMREAD_GRAYSCALE)[50:173,80:244]          # queryImage
img2 = cv.imread('box_in_scene.png',cv.IMREAD_GRAYSCALE)[100:284,100:412] # trainImage
if img1 is None or img2 is None:
    print('Could not open or find the images!')
    exit(0)
tt = 0
for i in range(100):
# Initiate SIFT detector
    t1 = time.time()
    detector = cv.xfeatures2d.SIFT_create()
    
    keypoints1, descriptors1 = detector.detectAndCompute(img1, None)
    keypoints2, descriptors2 = detector.detectAndCompute(img2, None)
    
    # FLANN paramters
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 4)
    search_params = dict(check=50) # or pass dictory
    
    flann = cv.FlannBasedMatcher(index_params, search_params)
    
    matches = flann.knnMatch(descriptors1, descriptors2, k=2)
    
    # Need to draw only good matches, so create a mask
    matchesMask = [[0,0] for i in range(len(matches))] # python2.x for xrange()
    
    # ratio test as per Lowe's paper
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.5 * n.distance:
            matchesMask[i] = [1,0]
    tt += time.time()-t1

print("time cost",tt/100)
draw_params = dict(matchColor=(0, 0, 255), singlePointColor=(255, 0, 0),
                   matchesMask=matchesMask, flags=0)

img_matches = cv.drawMatchesKnn(img1, keypoints1, img2, keypoints2, matches, None, **draw_params)
plt.imshow(img_matches,),plt.show()