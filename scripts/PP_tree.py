#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 13 12:23:28 2021

@author: chen
"""
import numpy as np
import math as mt
from time import time as time
from rtree import index
from sklearn.neighbors import KDTree

class PP_TREE:
    def __init__(self):
        p = index.Property()
        p.dimension = 3
        p.dat_extension = 'data'
        p.idx_extension = 'index'
        self.tree = index.Index(properties=p)
        self.nd_list = np.array([[0,0,0]])
        self.parent_idx = [0]
        self.cost_idx = [0]
        self.node_num = 0
        self.tree.insert(self.node_num,tuple([0,0,0]))
    def get_parent (self,new_node,SA):
        
        ids = list(self.tree.intersection((new_node[0]-SA, new_node[1]-SA, new_node[2]-SA, new_node[0]+SA,new_node[1]+SA,new_node[2]+SA)))
        if len(ids)==0:
            return -1,-1
        costs = np.linalg.norm(self.nd_list[ids]-new_node,axis=1)
      
        p_id = np.argmin(costs)
        return ids[p_id],costs[p_id]
    def insert_node (self,new_nodes,SA):
        for new_node in new_nodes:
            print(self.node_num)
            parent_id,parent_cost = self.get_parent(new_node,SA)
            if parent_id <0:
                # return []
                continue
           
            self.node_num+=1
            self.nd_list = np.r_[self.nd_list,[new_node]]
            self.tree.insert(self.node_num,tuple(new_node))
            self.parent_idx.append(parent_id)
            self.cost_idx.append(self.cost_idx[parent_id] + parent_cost)
        
        return self.nd_list
    
    def fastinsert_node (self,new_node,parent_id,parent_cost):
        # parent_id,parent_cost = self.get_parent(new_node)
        # if parent_id <0:
        #     return []
        self.nd_list = np.r_[self.nd_list,[new_node]]
        # self.tree.insert(self.node_num,tuple(new_node))
        self.parent_idx.append(parent_id)
        self.cost_idx.append(self.cost_idx[parent_id] + parent_cost)
        # self.node_num+=1
        return self.nd_list   
    
    def get_path (self,goal):
        path = []
        all_id = list(np.arange(0,len(self.nd_list)))
        leaf_id1 = list(set(all_id).difference(set(self.parent_idx)))
        leaf_nodes = self.nd_list[leaf_id1]
        leaf_id = np.array(leaf_id1)
        costs_to_goal = np.linalg.norm(leaf_nodes-goal,axis=1)
        idx_close = np.where(costs_to_goal<1)
        try:
            if len(idx_close[0]) ==0:
                idx_close = np.argmin(costs_to_goal)
                leaf_id = leaf_id[idx_close]
                costs_to_goal1 = costs_to_goal[idx_close]
                print("final leaf node1,cost:",leaf_id,costs_to_goal1,leaf_nodes[idx_close],goal,min(costs_to_goal))
                costs = np.array(self.cost_idx)[leaf_id] + costs_to_goal1
                last_id = leaf_id
            else:
                leaf_id = leaf_id[idx_close]
                costs_to_goal = costs_to_goal[idx_close]
                costs = np.array(self.cost_idx)[leaf_id] + costs_to_goal
         
                last_id = leaf_id[np.argmin(costs)]
                print("final leaf node2,cost:",leaf_id,costs_to_goal,self.nd_list[last_id],goal)
        except:
            print(len(self.nd_list),leaf_id,leaf_id1,costs)

        
        pt_node_id = 1
        path.append(self.nd_list[last_id])
        while pt_node_id != 0:
            pt_node_id = self.parent_idx[last_id]
            path.append(self.nd_list[pt_node_id])
            last_id = pt_node_id
        return path
    def kd_tree(self,pcl,k):
        self.kdtree = KDTree(pcl, leaf_size=k)

    def kd_colcheck (self,pcl,samples,SA):
        dist, ind = self.kdtree.query(samples, k=1)
        free_pts = samples[list(np.where(dist>SA)[0])]
        return free_pts
    def fastget_path (self,goal):
        path = []

        last_id = -1
        pt_node_id = 1
        path.append(self.nd_list[last_id])
        while pt_node_id != 0:
            pt_node_id = self.parent_idx[last_id]
            path.append(self.nd_list[pt_node_id])
            last_id = pt_node_id
        return path