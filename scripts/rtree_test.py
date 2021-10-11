#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  2 13:44:59 2021

@author: chen
"""
from rtree import index
from time import time as time
from utils import earth_to_body_frame
import numpy as np

p = index.Property()
p.dimension = 3
p.dat_extension = 'data'
p.idx_extension = 'index'
idx = index.Index(properties=p)
key_id=0
s_len = 0.2
poss = []
for i in range(600):
    pos = np.random.rand(3)
    idx.insert(key_id,tuple(pos))
    key_id += 1
    poss.append(pos)
poss = np.array(poss)
t1=time()

for i in range(400):
    qry_pt =  np.random.rand(3)
    
    dis_qry = np.linalg.norm(qry_pt)
    e2b = earth_to_body_frame(0.8,0.7,0.4)
    body_qrypt = np.matmul(e2b,qry_pt).T
    ids = list(idx.intersection((qry_pt[0]-s_len, qry_pt[1]-s_len, qry_pt[2]-s_len, qry_pt[0]+s_len,qry_pt[1]+s_len,qry_pt[2]+s_len)))
    
t2 = time()
print("time cost:",t2-t1,ids)#,poss[ids],qry_pt)  