#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 21 14:19:42 2021

@author: chen
"""
import numpy as np
import math as mt
from time import time as time
# from utils import earth_to_body_frame
import linesafe
from line_sf_pytran import line_sf
PI = 3.14159265358979
class FET:
    
    def body_to_earth_frame(self,ii, jj, kk):
        Cii, Cjj, Ckk=np.cos(ii), np.cos(jj), np.cos(kk)
        Sii, Sjj, Skk=np.sin(ii), np.sin(jj), np.sin(kk)
    
        R = np.array([[Ckk * Cjj, Ckk * Sjj * Sii - Skk * Cii, Ckk * Sjj * Cii + Skk * Sii],
                    [Skk * Cjj, Skk * Sjj * Sii + Ckk * Cii, Skk * Sjj * Cii - Ckk * Sii],
                    [-Sjj, Cjj * Sii, Cjj * Cii]])
        return R

    def earth_to_body_frame(self,ii, jj, kk):
        return self.body_to_earth_frame(ii, jj, kk).T

    def Angle2(self, x,  y):
        angle = 0.0;
        x1 = x[0]
        y1 = x[1]
        x2 = y[0]
        y2 = y[1]
        # if_inv = 0
        if (x1 == 0 and y1 ==0)or(x2 == 0 and y2 ==0):
            angle =0.0
        else:
            angle=mt.atan2(x1,y1) - mt.atan2(x2,y2)
        if angle<0:
            angle = -angle
            # if_inv = 1
        if 2*PI>angle>PI:
            angle= 2*PI - angle
        elif 2*PI<angle:
            angle= angle - 2*PI
        if y1<0:
            angle = -angle
        # if if_inv:
        #     angle = -angle
        # elif angle<-pi:
        #     angle=angle+2*pi
        # print('angle:',-angle)
        return angle

    def line_sf (self,pcl,p1,p2,SA,return_pts=0): #
        if_collide=0
        # l1s = np.linalg.norm(X-p1,axis=1)
       
        L=np.linalg.norm(p2-p1)
        UB=(SA+(SA**2+L**2)**0.5)
        xp1=np.array([min(p1[0],p2[0]),min(p1[1],p2[1]),min(p1[2],p2[2])])
        xp2=np.array([max(p1[0],p2[0]),max(p1[1],p2[1]),max(p1[2],p2[2])])
        index = np.where((xp2[0]+SA > pcl[:,0]) & (xp1[0]-SA < pcl[:,0]) &(xp2[1]+SA > pcl[:,1]) & (xp1[1]-SA < pcl[:,1]) &
                          (xp2[2]+SA > pcl[:,2]) &(xp1[2]-SA < pcl[:,2]))
       
        pcl = pcl[index]
        # l1s = np.linalg.norm(pcl-p1,axis=1)
        # l2s = np.linalg.norm(pcl-p2,axis=1)
        if len(pcl):
            aa = (pcl-p1)**2
            bb= (pcl-p2)**2
            l1s = np.sum(aa,axis=1) #a^2
            l2s = np.sum(bb,axis=1) #b^2
            jt = 2*(L**2)*(l1s+l2s)-(l1s-l2s)**2-L**4
           
            # aa = np.sort(l1s+l2s)
            # list1 = np.where(l1s**0.5+l2s**0.5<UB)
            # l1s = l1s[list1]
            # l2s = l2s[list1]
            # pcl=pcl[list1]
            # jt = 2*(L**2)*(l1s+l2s)-(l1s-l2s)**2-L**4
            # aa = np.where(jt<4*(L*SA)**2)
            aa = np.where((l1s**0.5+l2s**0.5<UB)&(jt<4*(L*SA)**2))
            if len(aa[0]):# and aa[0]<UB:
                if_collide=1
        if return_pts:
            return L,if_collide,pcl[aa]
        else:
            return L,if_collide

            
           
        
    def gen_samples (self,goal,r_num,dr,angle_seeds,circle_num):  #r_list: the index of radius of the collide part of the line uav-goal
        # samples = []
        # r_num = max(1,r_num)
        l=max(dr,dr*r_num)
        # yaw=mt.atan2(goal[1],goal[0])

        ang_g = np.array([self.Angle2(goal[0:2], [1,0]),self.Angle2([np.linalg.norm(goal[0:2]),goal[2]], [1,0])])
        # ang_g = np.array([yaw,self.Angle2([np.linalg.norm(goal[0:2]),goal[2]], [1,0])])
        print("r_num,circle_num",r_num,circle_num,len(angle_seeds),len(angle_seeds[r_num]))
        print("ang_goal",ang_g,goal)
        angles = angle_seeds[r_num][circle_num]+ang_g
        samples = np.array([l*np.cos(angles[:,0])*np.cos(angles[:,1]),
                            l*np.sin(angles[:,0])*np.cos(angles[:,1]),
                            l*np.sin(angles[:,1])]).T
        # print("samples:",samples)
        return samples
    def gen_angle_seed (self,dis,dr,segs):
        seeds = []
        for i in range(segs):
            alpha = 2*mt.asin(dis/(2*max(1,i)*dr))
            if i<2:
                alpha = alpha/2
            itnum = int(max(1,np.rint(PI/2/alpha))+1)
            seed = []
            for j in range(1,itnum):
                line= np.arange(-j, j+0.1)*alpha
                line1 = np.c_[line,np.ones(2*j+1)*(j*alpha)]
                line2 = np.c_[np.ones(2*j-1)*(j*alpha),line[1:-1]]
                
                line3 = np.c_[line,-np.ones(2*j+1)*(j*alpha)]
                line4 = np.c_[-np.ones(2*j-1)*(j*alpha),line[1:-1]]
                seed.append(np.r_[line1,line2,line3,line4])
            seeds.append(seed)
        return seeds
    def Angle1 (self,a,b):
        angle = mt.acos(a.dot(b)/(np.linalg.norm(a)*np.linalg.norm(b)))
        return angle
    def get_node(self,wpts,new_node,dis_layer):
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
        if dists[index] > dis_layer/np.cos(PI/5):
            return -1,[]
        return index, last_nodes[index]
        
    def check_path (self,pcl,waypoints,SA):
        if_safe = 1
        for i in range(len(waypoints)-1):
            p1,p2=waypoints[i:i+2]
            if linesafe.line_sf (pcl,p1,p2,SA,0)[1]:
                if_safe = 0
                break
        return if_safe
    def opt_path (self,wpts,pcl,SA):
       
        if len(wpts)>3:
            i=1
            while i < len(wpts)-1:
                while not self.line_sf (pcl,wpts[i-1] ,wpts[i+1],SA,0)[1]:
                    wpts = np.delete(wpts,i,axis=0)
                    print("delete one waypoint!",i)
                    if i >= len(wpts)-2:
                        break
                i+=1
                    
        return wpts
    def check_fov(self,qry_pt,rtree,cam_param,cam_ori,uav_pose):
        depth,h_ang,v_ang = cam_param
        ct_pos = uav_pose[0:3]
        ct_ori = uav_pose[3::]
        e2b = self.earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
        body_qrypt = np.matmul(e2b,qry_pt).T
        s_len = depth/1.732
        # dis_qry = np.linalg.norm(qry_pt) #dis_qry<  depth and 
        if abs(self.Angle2(body_qrypt[0:2], [1,0]))<h_ang and abs(self.Angle2(body_qrypt[::2], [1,0]))<v_ang:
            return 1
        # else:
        #     print("dis_qry", dis_qry)
        #     return 0
        qry_pt = qry_pt+ct_pos
        ids = list(rtree.intersection((qry_pt[0]-s_len, qry_pt[1]-s_len, qry_pt[2]-s_len, qry_pt[0]+s_len,qry_pt[1]+s_len,qry_pt[2]+s_len)))
        
        for it in ids:
            ct_pos = cam_ori[it][0:3]
            ct_ori = cam_ori[it][3::]
            e2b = self.earth_to_body_frame(ct_ori[0],ct_ori[1],ct_ori[2])
            body_qrypt = np.matmul(e2b,qry_pt-ct_pos).T
            if abs(self.Angle2(body_qrypt[0:2], [1,0]))<h_ang and abs(self.Angle2(body_qrypt[::2], [1,0]))<v_ang:
                return 1
        return 0
    def get_path (self,pcl,goal,dr,sense_range,SA,seeds,path_num,rtree,cam_param,cam_ori,uav_pose,max_height):  # returns if fly straightly to goal, and the waypoints
        t1 = time()
        segs = np.rint(sense_range/dr)
       
        # st_pts = goal/np.linalg.norm(goal)*dr
        which_collide = []
        L_goal,if_cld0,pts = linesafe.line_sf(pcl,np.array([0.0,0.0,0.0]),goal,SA,1)
        if if_cld0:
            for pt in pts:
                which_collide.append(int(np.dot(pt,goal)/L_goal/dr))  
        which_collide = list(np.unique(which_collide))
        # for i in range(int(segs)):
        #     p1 = st_pts*i
        #     p2 = st_pts*(i+1)
        #     if linesafe.line_sf (pcl,p1,p2,SA)[1]:
        #         which_collide.append(i)
        
        
        if not len(which_collide):
           return 1,[],[]   #0 for fly curve, 1 for straight, 2 for no waypoint can be found
        elif len(which_collide)>=2 and which_collide[0]==0 and which_collide[1]==1:
            del which_collide[0]
        # if len(which_collide)>=2 and which_collide[0]==0 and which_collide[1]==1:
        if which_collide[-1] == int(segs):
            del which_collide[-1]
        if which_collide[-1] != int(segs)-1:
            which_collide.append(int(segs)-1)
        print("all the collide segs:",segs,which_collide)
        iter_num = 0        
        
        # wpts = []
        samples = []
        all_sample = [[],[]]
        self.check_num = 0
        self.check_fov_num = 0
        t2 = time()
        time_check = np.zeros(5)
        for j in which_collide:
            if iter_num==0:
                last_nodes = np.array([[0,0,0]]).astype(float)
                wpts = np.array([[np.array([0,0,0])]*path_num]).astype(float)
                cost_list = np.array([[0]*path_num]).astype(float)
                dis_layer = j*dr
            else:
                dis_layer = (which_collide[iter_num]-which_collide[iter_num-1])*dr
            print("dis_layer",dis_layer,j)
            # elif len(ct_nodes)==0:
            #     break
            # if len(samples):
            #     last_samples = samples.copy()
            circle_num=0  # 0 is for the first square-edge search
            # print("j1",j)
            
            ct_nodes=[]
            costs=[]
            last_index = []
            # print("len(seeds)",len(seeds),j)
            while len(ct_nodes) < path_num and j < len(seeds) and circle_num<len(seeds[j]):
                t01=time()
                samples = linesafe.gen_samples(goal, j, dr, seeds, circle_num) #n*3 array for x,y,z local corridinate
                print("samples size:",[np.size(samples,axis=0),np.size(samples,axis=1)])
                print("len(ct_nodes)",len(ct_nodes),"circle_num",circle_num,"len(seeds[j])",len(seeds[j]),"j",j)
                
                # ct_nodes=[]
                # costs=[]
                # last_index = []
                jj = 0
              
                time_check[0] += time()-t01
                for it in samples:
                    if it[2] > max_height:
                        continue
                    t11=time()
                    self.check_fov_num +=1
                    if not linesafe.check_fov(it,rtree,cam_param,cam_ori,uav_pose):
                    # if not self.check_fov(it,rtree,cam_param,cam_ori,uav_pose):
                        print("FOV check fail",jj,it)
                        all_sample[0].append(it)
                        jj+=1
                       
                        time_check[1]+= time()-t11
                        continue
                    # if jj > len(samples)*0.8 or jj>10:
                    if jj > len(samples)*0.95:
                        circle_num = float("Inf")
                    all_sample[1].append(it)
                    
                    t12 = time()
                    index,node = linesafe.get_node(wpts,it,dis_layer)
                    time_check[1]+= t12-t11
                    if index <0:
                        continue
                    # cost,if_cld = line_sf (pcl,node.astype(float),it.astype(float),SA)
                    cost,if_cld = linesafe.line_sf (pcl,node,it,SA,0)
                    t13=time()
                    
                    self.check_num +=1
                    if not if_cld:
                        ct_nodes.append(it)
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
                        time_check[2:4]+=np.array([t13-t12,time()-t13])
                        break
                    time_check[2:4]+=np.array([t13-t12,time()-t13])   
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
                return 2,[],[]
            time_check[-1] += time()-t14
            # print("j3",j)
        t3=time()
        
        final_list,final_cost = [],[]
        i=0
        for lnd in wpts[-1]:
            cost = np.linalg.norm(lnd-goal) #cost,if_cld = linesafe.line_sf (pcl,lnd,goal,SA)
            # if not if_cld:
            final_list.append(i)
            final_cost.append(cost+sum(np.array(cost_list)[:,i]))
            
            i+=1
        path_ind = final_list[np.argmin(final_cost)]
        print("path_ind,costs",path_ind,final_cost)
        final_waypoints = np.r_[wpts[:,path_ind],np.array([goal])]
        final_waypoints = linesafe.opt_path(final_waypoints,pcl,SA)
        print("time cost 123:",time()-np.array([t1,t2,t3]),time_check,sum(time_check))
        return 0,final_waypoints,all_sample
        