# -*- coding: utf-8 -*-
"""
Created on Sat Dec 28 11:36:44 2019

@author: chenhan
"""
import rospy
from utils import earth_to_body_frame,body_to_earth_frame
from math import *
import numpy as np
#import matplotlib.pyplot as plt
#from PIL import Image
import time
#from numba import autojit
from scipy.optimize import minimize
#from skimage import measure
#@autojit 


class control_method():
    """
    用来仿真的参数，
    """
    @staticmethod
    def init(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]  # max speed
        # self.min_speed = -0.5  # [m/s]  # min speed
        self.m_speed = self.max_speed  # [m/s]  # max speed
        self.min_speed = 0.4  # [m/s]  # min speed
#        self.max_accel = self.max_speed*0.5  # [m/ss]  # max accelerate
        self.max_accel = 3
        self.dt = 0.02  # [s]  # simulation period
        self.max_jerk = 3
        self.predict_coe = 1  # [s]  # predict coefficient
        self.control_gain = 0.6
        self.to_goal_cost_gain = 6.0  # goal_cost_gain
        self.to_obs_cost_gain = 0.0  # obs_cost_gain
        self.speed_cost_gain = 15     # speed_cost_gain
        # self.uav_r = 0.4  # [m]  # uav safe radius ,0.45 for static
        self.dis2wp = 0.4
        self.detect_l = 3.0 # [m]  # detectsion radius
        self.pcl_fl = 3
        self.det_ang = pi/12
        self.map_reso = 0.2 # [m]  # map resolution
        self.startpoint=np.array([3.0,3.0,0.0])    # start point
        self.endpoint=np.array([94.0,94.0,0.0])    #goal,assume the size is 100m*100m
        self.error=0.5             # if the uav reach the goal
        self.global_l = 100 
        self.x0 = np.append(self.startpoint,np.array([0,0,0,0,0,0])).astype(float)
        # self.dd = int(self.uav_r/self.map_reso)+1
        self.pcl_distance=3
        self.pred_coe = 0.6
        self.d3_detang_les=1.0 # for 3d better path search, how many angular grids less than 2d max angle
        self.upb=5
        self.downb=0.5
        self.vel_coe = 0.7
        self.iter=0
        self.p_num=70  #max number of points for collision check
        self.pnum_map=60
        self.acc_CD=2  #parameter of acceleration expanding coefficient for avoiding moving obstacle
        self.d3_fly_leth=0.5 # the fly length after the 3d path is found 
        self.wps=[]
        self.dmin=[]
        self.c_dyn=[]
        self.v_dyn=[]
        self.r_dyn=[]
        self.obs_v=0  #if we change the velocity for dynamic obs
        self.no_path=0
        self.apf_ang=np.array([0,0])
        # self.if_d3_goal = 0
    @staticmethod
    def Angle( x,  y):
        angle = 0.0;
        x1 = x[0]
        y1 = x[1]
        x2 = y[0]
        y2 = y[1]
    
        if (x1 == 0 and y1 ==0)or(x2 == 0 and y2 ==0):
            angle =0.0
        else:
            angle=atan2(x1,y1)-atan2(x2,y2)
        if angle>pi:
            angle=angle-2*pi
        elif angle<-pi:
            angle=angle+2*pi
        # print('angle:',-angle)
        return -angle
    # @staticmethod
    # def Angle (x,y):
    #     aa = sum(x**2)**0.5
    #     bb = sum(y**2)**0.5
    #     cc2 = sum((y-x)**2)
    #     angle = acos((aa**2+bb**2-cc2)/(2*aa*bb))
    @staticmethod
    def pp_vector(v,v_p):
        w=-(v_p[0]**2+v_p[1]**2+v_p[2]**2)/(v_p[0]*v[0]+v_p[1]*v[1]+v_p[2]*v[2])
        pp_v=(w*v+v_p)/np.linalg.norm(w*v+v_p)*np.linalg.norm(v)
        return pp_v
    @staticmethod
    def apf(self,loc_goal,local_pos,pcl,pred_pos,mass_a,pcl_w,max_det_ang,wpts0):
#        ppf_itv=15
        att_mag = 7
        att_mag_min = 2
        iner_mag = 0.5
        d_force_mag_min = 1
        if len(pcl) and len(pcl_w):
            # if np.linalg.norm(pcl[0])<0.5:
            #     z_co=1
            # else:
            #     z_co=0.25
            z_co = min(1,(0.6/np.linalg.norm(pcl[0])))
            d_rev=(1./np.linalg.norm(pcl_w-pred_pos,axis=1))**3.0*mass_a
            d_force=-4.0*np.sum((np.c_[d_rev,d_rev,d_rev]*(pcl_w-pred_pos)),axis=0)
            # d_force = -1.5*mass*(m_ct-pred_pos)/(np.linalg.norm(m_ct-pred_pos)**2.5)
            d_force[2] *= z_co  
            # d_force[0:2] *= 1+z_co
            if np.linalg.norm(d_force[0:2])<d_force_mag_min:
                d_force[0:2] = d_force_mag_min*d_force[0:2]/np.linalg.norm(d_force[0:2])
            if local_pos[2]<1:
                
                d_force[2] += 3/(local_pos[2]**3)
            print("static force-1:",d_force,z_co,d_rev)
        else:
            d_force=0
        
        # if wpts0 is not None:
        #     iner_force = wpts0[0]/np.linalg.norm(wpts0[0])*iner_mag
        #     iner_force[2] = 0
        # else:
        #     iner_force = 0
        v_mag = np.linalg.norm(self.velocity)
        if v_mag > self.min_speed:
            iner_force = self.velocity/v_mag*iner_mag
        else:
            iner_force = 0
        
        if len(self.c_dyn)>0:
            print("dynamic force applied!!!")
            v_rev=(1./np.linalg.norm(self.c_dyn,axis=1))**2*self.r_dyn*1
            v_force=-1.5*np.sum(np.c_[v_rev,v_rev,v_rev]*self.v_dyn,axis=0)
        else:
            v_force=0
        att_coe = max(att_mag_min,att_mag/np.linalg.norm(loc_goal-pred_pos))
        if np.linalg.norm(loc_goal-pred_pos)<1:
            att_coe = att_mag
        
        attract=(loc_goal-pred_pos)/(np.linalg.norm(loc_goal-pred_pos))*att_coe  # 7: param
        if attract[2]<0:
            attract[2] *= 1.5 
        if v_force is not 0:
            res_force=attract+v_force
            aa=np.linalg.norm(res_force)
            bb=np.linalg.norm(attract)
            cc=np.linalg.norm(attract-res_force)
            ang_gd=(-cc**2+aa**2+bb**2)/(2*aa*bb)
            if ang_gd<-0.6 :
#                self.cont_pv+=1
#                if self.cont_pv % ppf_itv==0:
                v_force=self.pp_vector(v_force,attract)
                print("dynamic verticle force!",v_force,attract)
                    
            else:
                self.cont_pv=-1
        else:
            self.cont_pv=-1
        if d_force is not 0: 
            res_force=attract+d_force+iner_force
            aa=np.linalg.norm(res_force)
            bb=np.linalg.norm(attract)
            cc=np.linalg.norm(attract-res_force)
            ang_gd=(-cc**2+aa**2+bb**2)/(2*aa*bb)
            if ang_gd< 0:
                if ang_gd< -0.7:
                    param1 = 1
                else:
                    param1 = -(ang_gd/(0.6)+1)**2+1
                param2 = 1-param1
            else:
                param1 = 0
                param2 = 1-ang_gd**2     
#                self.cont_pd+=1
#                if self.cont_pd % ppf_itv==0:
            d_force=self.pp_vector(d_force,attract)*param1 + d_force*param2
            print("static verticle force!",d_force,attract)
                    
        #     elif ang_gd<0.5:
        #         d_force=self.pp_vector(d_force,attract)*(-)
        #         self.cont_pd=-1
        # else:
        #     self.cont_pd=-1
        print('attract,loc_goal,pred_pos',attract,loc_goal,pred_pos)
        print("magnitudes for obs_f and att_f",np.linalg.norm(d_force),att_mag)
        res_force=attract+d_force+v_force+iner_force
        ang=np.array([self.Angle(res_force[0:2],[1,0]),self.Angle([np.linalg.norm(res_force[0:2]),res_force[2]],[1,0])])
        ang_goal=np.array([self.Angle((loc_goal-pred_pos)[0:2],[1,0]),self.Angle([np.linalg.norm((loc_goal-pred_pos)[0:2]),(loc_goal-pred_pos)[2]],[1,0])])
#        print("apf_ang:",ang)
        if ang[1]<-pi/6:
            ang[1]=0
        if local_pos[2]>self.upb:
            ang[1]=-0.3
        apf_ang0=self.apf_ang.copy()
        if ang_goal[0]*ang[0]>=0 :
            ang_dif=abs(ang_goal[0]-ang[0])
        
        else:
            ang_dif=abs(np.sign(ang[0])*(2*pi-abs(ang_goal[0]))-ang[0])
            if ang_dif>pi:
                ang_dif=2*pi-ang_dif
        if self.apf_ang[0] ==0 or ang_dif<pi/1.5:#or np.linalg.norm(pcl[0])<0.3:
            self.apf_ang=ang.copy()
            print("self.apf_ang=ang",ang)
        else:
            self.apf_ang[0] = ang_goal[0]+np.sign(ang[0]-ang_goal[0])*pi/1.5
            self.apf_ang[1] = ang[1]
        # print("self.apf_ang,apf_ang0,ang",self.apf_ang,apf_ang0,ang)
#        self.apf_ang=ang.copy()
        return self.apf_ang,apf_ang0
 
    @staticmethod
    def apf1(self,loc_goal,local_pos,pcl,pred_pos,mass_a,pcl_w,max_det_ang,wpts0):
#        ppf_itv=15
        att_mag = 7
        att_mag_min = 1
        iner_mag = 1
        d_force_mag_min = 1
        if len(pcl) and len(pcl_w):
            # if np.linalg.norm(pcl[0])<0.5:
            #     z_co=1
            # else:
            #     z_co=0.25
            z_co = min(1,(0.6/np.linalg.norm(pcl[0])))
            d_force = 0
            loc_goal = self.detect_l*loc_goal/np.linalg.norm(loc_goal)
            for i in range(3):
                pred_pos = loc_goal * i/2
                d_rev=(1./np.linalg.norm(pcl_w-pred_pos,axis=1))**3.0*mass_a
                d_force=-1.0*np.sum((np.c_[d_rev,d_rev,d_rev]*(pcl_w-pred_pos)),axis=0)+d_force
            # d_force = -1.5*mass*(m_ct-pred_pos)/(np.linalg.norm(m_ct-pred_pos)**2.5)
            # d_force = d_force/3
            d_force[2] *= z_co  
            # d_force[0:2] *= 1+z_co
            
            # if np.linalg.norm(d_force[0:2])<d_force_mag_min:
            #     d_force[0:2] = d_force_mag_min*d_force[0:2]/np.linalg.norm(d_force[0:2])
            if local_pos[2]<1:
                
                d_force[2] += 10/(local_pos[2]**3)
            print("static force-1:",d_force,z_co,d_rev)
        else:
            d_force=0
        
        # if wpts0 is not None:
        #     iner_force = wpts0[0]/np.linalg.norm(wpts0[0])*iner_mag
        #     iner_force[2] = 0
        # else:
        #     iner_force = 0
        v_mag = np.linalg.norm(self.velocity)
        if wpts0 is not None and len(wpts0):
            iner_force = wpts0[0]/np.linalg.norm(wpts0[0])*iner_mag
        else:
            iner_force = 0
        
        if len(self.c_dyn)>0:
            print("dynamic force applied!!!")
            v_rev=(1./np.linalg.norm(self.c_dyn,axis=1))**2*self.r_dyn*1
            v_force=-1.5*np.sum(np.c_[v_rev,v_rev,v_rev]*self.v_dyn,axis=0)
        else:
            v_force=0
        att_coe = max(att_mag_min,att_mag/np.linalg.norm(loc_goal))
        if np.linalg.norm(loc_goal)<1:
            att_coe = att_mag
        # att_coe = att_mag
        attract=(loc_goal)/(np.linalg.norm(loc_goal))*att_coe  # 7: param
        if attract[2]<0:
            attract[2] *= 1.5 
        if v_force is not 0:
            res_force=attract+v_force
            aa=np.linalg.norm(res_force)
            bb=np.linalg.norm(attract)
            cc=np.linalg.norm(attract-res_force)
            ang_gd=(-cc**2+aa**2+bb**2)/(2*aa*bb)
            if ang_gd<-0.6 :
#                self.cont_pv+=1
#                if self.cont_pv % ppf_itv==0:
                v_force=self.pp_vector(v_force,attract)
                print("dynamic verticle force!",v_force,attract)
                    
            else:
                self.cont_pv=-1
        else:
            self.cont_pv=-1
        if d_force is not 0: 
            res_force=attract+d_force+iner_force
            aa=np.linalg.norm(res_force)
            bb=np.linalg.norm(attract)
            cc=np.linalg.norm(attract-res_force)
            ang_gd=(-cc**2+aa**2+bb**2)/(2*aa*bb)
            if ang_gd< 0:
                if ang_gd< -0.8:
                    param1 = 1
                else:
                    param1 = -(ang_gd/(0.8)+1)**2+1
                param2 = 1-param1
            else:
                param1 = 0
                param2 = 1-ang_gd**2     
#                self.cont_pd+=1
#                if self.cont_pd % ppf_itv==0:
            d_force=self.pp_vector(d_force,attract)*param1 + d_force*param2
            print("static verticle force!",d_force,attract,param1)
                    
        print('attract,loc_goal,pred_pos',attract,loc_goal,pred_pos)
        print("magnitudes for obs_f and att_f",np.linalg.norm(d_force),att_mag)
        res_force=attract+d_force+v_force+iner_force
        ang=np.array([self.Angle(res_force[0:2],[1,0]),self.Angle([np.linalg.norm(res_force[0:2]),res_force[2]],[1,0])])
        ang_goal=np.array([self.Angle((loc_goal-pred_pos)[0:2],[1,0]),self.Angle([np.linalg.norm((loc_goal-pred_pos)[0:2]),(loc_goal-pred_pos)[2]],[1,0])])
#        print("apf_ang:",ang)
        if ang[1]<-pi/6:
            ang[1]=0
        if local_pos[2]>self.upb:
            ang[1]=-0.3
        apf_ang0=self.apf_ang.copy()
        if ang_goal[0]*ang[0]>=0 :
            ang_dif=abs(ang_goal[0]-ang[0])
        
        else:
            ang_dif=abs(np.sign(ang[0])*(2*pi-abs(ang_goal[0]))-ang[0])
            if ang_dif>pi:
                ang_dif=2*pi-ang_dif
        if self.apf_ang[0] ==0 or ang_dif<pi/1.5:#or np.linalg.norm(pcl[0])<0.3:
            self.apf_ang=ang.copy()
            print("self.apf_ang=ang",ang)
        else:
            self.apf_ang[0] = ang_goal[0]+np.sign(ang[0]-ang_goal[0])*pi/1.5
            self.apf_ang[1] = ang[1]
        # print("self.apf_ang,apf_ang0,ang",self.apf_ang,apf_ang0,ang)
#        self.apf_ang=ang.copy()
        return self.apf_ang,apf_ang0
    @staticmethod
    def get_wpts(self,local_pos,pcl,loc_goal_pl,loc_goal_old,state,no_path,wpts0,f_angle):
        max_det_ang = asin(min(self.max_accel/np.linalg.norm(self.velocity),0.8))
        # pcl = np.r_[pcl[0:int(len(pcl)/3)],pcl]
        # m_ct = np.mean(pcl,axis=0)
        # mass = len(pcl)**0.3
        p_id_0 = 0
        pcl_w = []
        mass_a = []
        pts_size = []
        angle_locgoal=np.array([self.Angle(loc_goal_pl[0:2],[1,0]),self.Angle([np.linalg.norm(loc_goal_pl[0:2]),loc_goal_pl[2]],[1,0])])       
        for mass in range(1,int(((1+8*len(pcl))**0.5-1)/2)+1):
            pts = pcl[p_id_0:p_id_0+mass]
            pts_mean = np.mean(pts,axis=0)
            size = max(np.linalg.norm(pts-pts_mean,axis=1))
            p_id_0 = p_id_0+mass
            if size > 1:
                 pts_size += [0]*mass
                 pcl_w += list(pts)
                 mass_a += [1]*mass
                 continue
           
            pts_size.append(size)
            # if abs(self.Angle(point[0:2],[1,0])-self.Angle(loc_goal[0:2],[1,0]))> pi*2/3:
            #     continue
                       
            pcl_w.append(pts_mean)
            mass_a.append(mass)
        if len(pcl) > p_id_0:
            pts_mean  = np.mean(pcl[p_id_0::],axis=0)
            size = max(np.linalg.norm(pcl[p_id_0::]-pts_mean,axis=1))
            mass = len(pcl)-p_id_0
            if size < 1:
                pcl_w.append(pts_mean)
                pts_size.append(size)
                mass_a.append(mass)
            # else:
            #     pts_size += [0]*mass
            #     pcl_w += list(pts)
            #     mass_a += [1]*mass
        # m_ct = sum(np.array(pcl_w))/mass_a

        dis2wp = min(0.5,self.dis2wp*max(1,np.linalg.norm(self.velocity)**0.5))
        
        # n_pred = int(self.detect_l/dis2wp)
        n_pred = 3
        d_dwp = (3-3*dis2wp)/3
        wpts=[]
        next_wp = np.zeros([3])
        # for i in range(n_pred):
            
        #     div_ang,_ = self.apf(self,loc_goal_pl,local_pos,pcl,next_wp,np.array(mass_a),np.array(pcl_w),max_det_ang,wpts0)
        #     # self.apf_ang=div_ang.copy()
            
        #     if i==3:
        #         div_ang = (div_ang+div_ang0)/2
        #         next_wp = wpts[1]+np.array([cos(div_ang[0])*dis2wp*cos(div_ang[1]),
        #                                 sin(div_ang[0])*dis2wp*cos(div_ang[1]),
        #                                 sin(div_ang[1])*dis2wp])
        #     else:
        #         dis2wp = dis2wp + d_dwp
        #         next_wp = next_wp+np.array([cos(div_ang[0])*dis2wp*cos(div_ang[1]),
        #                                 sin(div_ang[0])*dis2wp*cos(div_ang[1]),
        #                                 sin(div_ang[1])*dis2wp])
            
        #     if i!=2:    
        #         wpts.append(next_wp)
        #     div_ang0=div_ang.copy()
                
            # last_wp = next_wp.copy()
        
       
        
        if len(pcl) > 0 and self.num_dyn == 0 and np.linalg.norm(f_angle-angle_locgoal)<pi/2 and (not no_path):  #and ((not self.if_direct) and (not last_if_direct)):
            coe_dis_goal = 0.4
            print("heuristic!")
        else:
            coe_dis_goal = 1
            print("direct to goal!")
        if (f_angle[0]!=float("Inf") and f_angle[1]!=float("Inf")):
            f_angle = np.array(f_angle)
            angle_locgoal = ((angle_locgoal*coe_dis_goal + f_angle*(1-coe_dis_goal)- angle_locgoal)/(self.det_ang)).astype(int)*(self.det_ang)+angle_locgoal      
        loc_goal_pl = np.array([cos(angle_locgoal[0])*self.detect_l*cos(angle_locgoal[1]),
                sin(angle_locgoal[0])*self.detect_l*cos(angle_locgoal[1]),
                sin(angle_locgoal[1])*self.detect_l])
        i=0
        dis_obs = self.d_obstacle(self,loc_goal_pl,pcl_w,0,0,0,pts_size) 
        dis_obs_list = [dis_obs]
        ang_list = [angle_locgoal]
        while dis_obs < 100 and i < 7:
            
            if i < 1:
                div_ang,_ = self.apf1(self,loc_goal_pl,local_pos,pcl,next_wp,np.array(mass_a),np.array(pcl_w),max_det_ang,wpts0)
            angle_locgoal1 = (div_ang-angle_locgoal)/np.linalg.norm(div_ang-angle_locgoal)*self.det_ang*(i+1) + angle_locgoal
            if (f_angle[0]!=float("Inf") and np.linalg.norm(angle_locgoal1 - f_angle) < max_det_ang) or f_angle[0]==float("Inf"):
                loc_goal_pl = np.array([cos(angle_locgoal1[0])*self.detect_l*cos(angle_locgoal1[1]),
                    sin(angle_locgoal1[0])*self.detect_l*cos(angle_locgoal1[1]),
                    sin(angle_locgoal1[1])*self.detect_l])
                dis_obs = self.d_obstacle(self,loc_goal_pl,pcl_w,0,0,0,pts_size) 
                dis_obs_list.append(dis_obs)
                ang_list.append(angle_locgoal1)
            i+=1
        if dis_obs>100:
            if i >0:
                f_angle = angle_locgoal1
            else:
                f_angle = angle_locgoal
            no_path = 0
        else:
            f_angle = ang_list[np.argmin(np.array(dis_obs_list))]
            loc_goal_pl = np.array([cos(f_angle[0])*self.detect_l*cos(f_angle[1]),
                sin(f_angle[0])*self.detect_l*cos(f_angle[1]),
                sin(f_angle[1])*self.detect_l])
            no_path=1
        print("i:number of iters to find wps",i,pts_size)
        wp_unit = loc_goal_pl/np.linalg.norm(loc_goal_pl)            
        for i in range(2):
            dis2wp *= i+1
            next_wp = wp_unit*dis2wp
            wpts.append(next_wp)
        wpts = np.array(wpts)
        # if wpts0 is not None:
        #     wpts0 = wpts0-local_pos
        #     # wpts_mot = np.r_[(np.array([wpts0[0]-local_pos])+np.array([wpts0[1]-local_pos]))/2,np.array(wtps[1::])]
        #     unit_vt = (wpts0[0]*1.3+wpts[0]*0.7)/2
        #     wpts_mot = np.r_[np.array([unit_vt]),(wpts[1::]*1.0+wpts0[1::]*1.0)/2]
        #     print("new wpts_mot",wpts_mot,wpts0,wpts)
        # else:
        #     wpts_mot = wpts.copy()
        wpts_mot = wpts.copy()
        return wpts,wpts_mot,f_angle,no_path,[pcl_w,pts_size]
    @staticmethod
    def get_localgoal(self,local_pos,pcl,f_angle,loc_goal,loc_goal_old,path_rec,state,no_path_last):

        global no_path
        dmin=[]
        wps=[]
        angle_locgoal=np.array([self.Angle(loc_goal[0:2],[1,0]),self.Angle([np.linalg.norm(loc_goal[0:2]),loc_goal[2]],[1,0])])
        angle_vel=np.array([self.Angle(state[3:5],[1,0]),self.Angle([np.linalg.norm(state[3:5]),state[5]],[1,0])])
        max_det_ang = asin(min(self.max_accel/np.linalg.norm(self.velocity),0.8))#
        angle_vel=[self.Angle(state[3:5],[1,0]),self.Angle([np.linalg.norm(state[3:5]),state[5]],[1,0])]
#        angle_locgoal=[(angle_locgoal[0]+0.3*angle_vel[0])/1.3,(angle_locgoal[1]+0.3*angle_vel[1])/1.3]
        # if np.linalg.norm(loc_goal)<self.detect_l*self.predict_coe:
        #     self.detect_l=max(np.linalg.norm(loc_goal)/self.predict_coe,1)
        for coe in [1]:
            no_path=1
            circle_p=np.array([cos(angle_locgoal[0])*self.detect_l*self.predict_coe*coe,
                sin(angle_locgoal[0])*self.detect_l*self.predict_coe*coe,
                sin(angle_locgoal[1])*self.detect_l*self.predict_coe*coe])

            last_if_direct = self.if_direct
            self.if_direct = self.d_obstacle(self,circle_p,pcl,0,no_path_last,loc_goal_old)<self.uav_r
            if (not self.if_direct):
                # if (abs(angle_locgoal[0]-f_angle[0])<max_det_ang or self.pp_restart==1 or f_angle[0]==float("Inf")):
                loc_goal = circle_p
                f_angle = angle_locgoal
                    
                no_path=0
                print("directly fly to local goal")
                break
#            if not self.if_direct:
            if len(pcl) > 0 and self.num_dyn == 0 and np.linalg.norm(f_angle-angle_locgoal)<pi/3:  #and ((not self.if_direct) and (not last_if_direct)):
#                coe_dis_goal = min(max(min(self.detect_l,np.linalg.norm(pcl[0]))/self.detect_l,0.2),0.4)
                coe_dis_goal = 0.5
#            elif len(pcl) > 0 and self.num_dyn != 0:
#                coe_dis_goal = 1
            else:
                coe_dis_goal = 1
            if (f_angle[0]!=float("Inf") and f_angle[1]!=float("Inf")):
                f_angle = np.array(f_angle)
                # print(angle_locgoal,f_angle) # + angle_vel*(1-coe_dis_goal)*2/3 +f_angle*(1-coe_dis_goal)*1/3 + 
                angle_locgoal = ((angle_locgoal*coe_dis_goal + f_angle*(1-coe_dis_goal)- angle_locgoal)/(self.det_ang)).astype(int)*(self.det_ang)+angle_locgoal
            
            if np.linalg.norm(loc_goal)>0.1*self.detect_l and len(pcl)!=0 and self.if_direct: #np.linalg.norm(loc_goal)>0.3*self.detect_l and
                if coe_dis_goal ==1:
                    start_ang = 2*self.det_ang
                else:
                    start_ang = 0
                for de_angle in np.arange(start_ang,pi/2*1.1,self.det_ang):
                    circle_p1=np.array([cos(angle_locgoal[0]+de_angle)*self.detect_l*self.predict_coe*cos(angle_locgoal[1]),
                                    sin(angle_locgoal[0]+de_angle)*self.detect_l*self.predict_coe*cos(angle_locgoal[1]),
                                    sin(angle_locgoal[1])*self.detect_l*self.predict_coe])
                    circle_p2=np.array([cos(angle_locgoal[0]-de_angle)*self.detect_l*self.predict_coe*cos(angle_locgoal[1]),
                                    sin(angle_locgoal[0]-de_angle)*self.detect_l*self.predict_coe*cos(angle_locgoal[1]),
                                    sin(angle_locgoal[1])*self.detect_l*self.predict_coe])
                    circle_p3=np.array([cos(angle_locgoal[0])*self.detect_l*self.predict_coe*cos(angle_locgoal[1]+de_angle),
                                    sin(angle_locgoal[0])*self.detect_l*self.predict_coe*cos(angle_locgoal[1]+de_angle),
                                    sin(angle_locgoal[1]+de_angle)*self.detect_l*self.predict_coe])
                    circle_p4=np.array([cos(angle_locgoal[0])*self.detect_l*self.predict_coe*cos(angle_locgoal[1]-de_angle),
                                    sin(angle_locgoal[0])*self.detect_l*self.predict_coe*cos(angle_locgoal[1]-de_angle),
                                    sin(angle_locgoal[1]-de_angle)*self.detect_l*self.predict_coe])
                    

#                    print('check points:/n',circle_p1,circle_p2,circle_p3,circle_p4,angle_locgoal)
                    d_min1=self.d_obstacle(self,circle_p1,pcl,1,no_path_last,loc_goal_old)
                    if de_angle == 0 :
                        if d_min1 > self.uav_r :
                            if abs(angle_locgoal[0]-f_angle[0])<max_det_ang or self.pp_restart==1 or f_angle[0]==float("Inf"):  # and abs(angle_locgoal[0]+de_angle-angle_vel[0]) < pi/3)
                                loc_goal = circle_p1
                                f_angle[0]=angle_locgoal[0]
                            else:
                                loc_goal = loc_goal_old
                            no_path=0
                            break
                        else:
                            continue
                    elif d_min1 > self.uav_r :#and local_pos[2]<self.upb:
                        if abs(angle_locgoal[0]+de_angle-f_angle[0])<max_det_ang or self.pp_restart==1 or f_angle[0]==float("Inf"):  # and abs(angle_locgoal[0]+de_angle-angle_vel[0]) < pi/3)
                            loc_goal = circle_p1
                            f_angle[0]=angle_locgoal[0]+de_angle
                        else:
                            loc_goal = loc_goal_old
                        no_path=0
                        break


                    elif self.d_obstacle(self,circle_p2,pcl,2,no_path_last,loc_goal_old)> self.uav_r:
                        if abs(angle_locgoal[0]-de_angle-f_angle[0])<max_det_ang or self.pp_restart==1 or f_angle[0]==float("Inf"):
                            loc_goal = circle_p2
                            f_angle[0]=angle_locgoal[0]-de_angle
                        else:
                            loc_goal=loc_goal_old
                        no_path=0
                        break
                    elif self.d_obstacle(self,circle_p3,pcl,3,no_path_last,loc_goal_old) > self.uav_r:
                        if abs(angle_locgoal[1]+de_angle-f_angle[1])<max_det_ang or f_angle[1]==float("Inf"):
                            loc_goal = circle_p3
                            f_angle[1]=angle_locgoal[1]+de_angle
                        else:
                            loc_goal=loc_goal_old
                        no_path=0
                        break
                    
                    elif local_pos[2]>self.downb and circle_p4[2]>0.3 and self.d_obstacle(self,circle_p4,pcl,4,no_path_last,loc_goal_old) > self.uav_r :
                        if abs(angle_locgoal[1]-de_angle-f_angle[1])<max_det_ang or f_angle[1]==float("Inf"):
                            loc_goal = circle_p4
                            f_angle[1]=angle_locgoal[1]-de_angle
                        else:
                            loc_goal=loc_goal_old
                        no_path=0
                        break
                #loc_goal=np.clip(loc_goal[0:2]-pos[0:2],-self.detect_l*self.predict_coe,self.detect_l*self.predict_coe)
                # if d_min1<=self.uav_r and d_min2<=self.uav_r and d_min3<=self.uav_r and d_min4<=self.uav_r:
                #     no_path=1
                # else:
                #     break
                if no_path==0:
                    self.no_path=0
                    break
                elif max(self.dmin)>0.6*self.uav_r:
                    loc_goal = self.wps[np.argmax(self.dmin)]
                else:
                    no_path=2
            else:
                loc_goal=loc_goal-0
                f_angle = angle_locgoal

                break
#        if no_path==2:
#            if len(path_rec)>1:
#                loc_goal=path_rec[-3-self.no_path]-local_pos
#                self.no_path+=1
#            else:
#                loc_goal=path_rec[-1]-local_pos
#            print('!!!!go back:',loc_goal+local_pos)
        if f_angle[0] == float("Inf"):
            f_angle[0] =0
        if f_angle[1] == float("Inf"):
            f_angle[1] =0
        # elapsed = (time.time() - starttime)
        # print('find goal time:',elapsed)
        # if np.linalg.norm(loc_goal)>self.detect_l:
        #     print('goal is too far:',loc_goal)
        # print('f_angle:',f_angle)
        return loc_goal,f_angle,no_path
       

    @staticmethod
    def calculate(self,lst_control,wpts,state,obstacle,b2e,path_rec,pred_dt):
    
        d_goal=np.linalg.norm(wpts[0]) 
        px = state[0]
        py = state[1]
        pz = state[2]
        vx = state[3]
        vy = state[4]
        vz = state[5]
        ax = state[6]
        ay = state[7]
        az = state[8]
        dt=self.dt
        if self.obs_v==1:
            self.max_accel*=self.acc_CD
            self.vel_coe += 0.2
        if self.no_path:
            self.max_accel*=2
            print("no path found, accel increase",self.no_path)
        starttime = time.time()
        control_gain=self.control_gain
        speed_cost_gain=self.speed_cost_gain
        if self.max_speed == self.min_speed:
            self.max_speed = self.min_speed + 0.2
       
        ve=max(np.linalg.norm(state[3:6]),self.min_speed)*wpts[0]/d_goal
        # t_max = 2.9*self.max_speed/self.max_accel #np.linalg.norm(ve-state[3:6])/self.max_accel
        ae=(ve-state[3:6])/(np.linalg.norm(ve-state[3:6])/self.max_accel*self.vel_coe) #pred_dt 
        x0=np.array([max(min(ae[0],self.max_accel-0.2),-self.max_accel+0.2),max(min(ae[1],self.max_accel-0.2),-self.max_accel+0.2),\
                      max(min(ae[2],self.max_accel-0.2),-self.max_accel+0.2)])
        bons = ((-self.max_accel, self.max_accel), (-self.max_accel, self.max_accel), (-self.max_accel, self.max_accel))

        trj_dt = self.vel_coe*np.linalg.norm(wpts[0])/self.max_speed
        cons = (
                {'type': 'ineq', 'fun': lambda x: -np.linalg.norm([vx+x[0]*trj_dt,vy+x[1]*trj_dt,vz+x[2]*trj_dt])+self.max_speed,
                 'jac' : lambda x: np.array([2*trj_dt*(vx+x[0]*trj_dt),2*trj_dt*(vy+x[1]*trj_dt),2*trj_dt*(vz+x[2]*trj_dt)])/(-2*np.linalg.norm([vx+x[0]*trj_dt,vy+x[1]*trj_dt,vz+x[2]*trj_dt]))}
             )
        res = minimize(self.fun, x0, args=(trj_dt,px,py,pz,vx,vy,vz,ax,ay,az,obstacle,dt,wpts,control_gain,speed_cost_gain,lst_control),method='SLSQP',\
                   options={'maxiter':15},constraints=cons,bounds = bons, tol = 2e-2)
        traj_end = ((np.array([vx,vy,vz])+res.x[0:3]*trj_dt/2)*trj_dt)
        pp_1 = traj_end -wpts[0]
        pp_2 = traj_end
        pp_3 = np.linalg.norm(wpts[0])
        pp_l = np.linalg.norm(np.cross(pp_1,pp_2))/pp_3
        pp_d=min(np.linalg.norm(traj_end -wpts[0]),pp_l)
        print("static traj end difference",pp_d,res.status,res.success,res.nit)
        opttime = (time.time() - starttime)
        
        print("Optimize Time used:",opttime)
        
        # print('position and goal',state[0:3],loc_goal+state[0:3],'control:',res.x)
        print("current velocity:",state[3:6])

        res.x[2] = max(np.linalg.norm(res.x[0:2])/(self.max_accel*1.414),0.66)*0.66+res.x[2] # compensation for the +z velocity
        if vz+res.x[2]*trj_dt > 3 and trj_dt > 0:
            res.x[2] = (3 - vz)/trj_dt
        elif vz+res.x[2]*trj_dt < -2 and trj_dt > 0:
            res.x[2] = (-2 - vz)/trj_dt

        if state[2]-self.downb<0 and trj_dt >0:
            res.x[2] = 0.5*self.max_accel
        if state[2]-self.upb>-1 and trj_dt >0:
            res.x[2] = -0.7
            
        # trj_dt = self.dis2wp/self.max_speed
        print('set velocity:',[vx+res.x[0]*trj_dt*self.vel_coe,vy+res.x[1]*trj_dt*self.vel_coe,vz+res.x[2]*trj_dt*self.vel_coe])
        set_vel = [vx+res.x[0]*trj_dt*self.vel_coe,vy+res.x[1]*trj_dt*self.vel_coe,vz+res.x[2]*trj_dt*self.vel_coe]
        pred_trj = []
        for ti in range(11):
            tti = trj_dt*(ti/5)
            pred_trj.append((np.array([vx,vy,vz])+res.x[0:3]*tti/2)*tti+self.local_pos)
        return set_vel[0],set_vel[1],set_vel[2],pp_d,res.x,pred_trj
    
    @staticmethod
    def jac(x,trj_dt,px,py,pz,vx,vy,vz,ax,ay,az,obstacle,dt,loc_goal,control_gain,speed_cost_gain,para_g):
        tg=para_g*loc_goal
        jac_vt = control_gain*np.array([x[0],x[1],x[2]])/np.linalg.norm(x[0:3]) + \
            4*np.array([2*trj_dt*(vx+x[0]*trj_dt),2*trj_dt*(vy+x[1]*trj_dt),2*trj_dt*(vz+x[2]*trj_dt)])/(-2*np.linalg.norm([vx+x[0]*trj_dt,vy+x[1]*trj_dt,vz+x[2]*trj_dt]))\
                                                    + 12*np.array([(vx*trj_dt+0.5*x[0]*trj_dt**2-tg[0])*trj_dt**2,(vy*trj_dt+0.5*x[1]*trj_dt**2-tg[1])*trj_dt**2,\
                                                 (vz*trj_dt+0.5*x[2]*trj_dt**2-tg[2])*trj_dt**2])/(2*np.linalg.norm((np.array([vx,vy,vz])+x[0:3]*trj_dt/2)*trj_dt - tg))
        return jac_vt
    @staticmethod
   
    def fun(x,trj_dt,px,py,pz,vx,vy,vz,ax,ay,az,obstacle,dt,wpts,control_gain,speed_cost_gain,lt_ctrl):
        # speed =np.linalg.norm(np.array([vx,vy,vz]))
        # func = control_gain*np.linalg.norm(x[0:3])+speed_cost_gain*abs(min(trj_dt,np.sign(trj_dt)*50)) -np.linalg.norm(np.array([vx,vy,vz])+x[0:3]*trj_dt)+ 30*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*trj_dt/2)*trj_dt) - para_g*loc_goal)
        traj_end = ((np.array([vx,vy,vz])+x[0:3]*trj_dt/2)*trj_dt)
        traj_end2 = ((np.array([vx,vy,vz])+x[0:3]*trj_dt)*2*trj_dt)
        pp_1 = traj_end -wpts[0]
        # pp_2 = traj_end
        # pp_3 = np.linalg.norm(wpts[0])
        # pp_l = np.linalg.norm(np.cross(pp_1,pp_2))/pp_3
        pp_d = 0.6*np.linalg.norm(pp_1)
        pp_12 = traj_end2 -wpts[1]
        pp_22 = traj_end2-wpts[0]
        pp_32 = np.linalg.norm(wpts[1]-wpts[0])
        pp_l2 = np.linalg.norm(np.cross(pp_12,pp_22))/pp_32       
        control_gain=1.0 #+0.2*np.linalg.norm([vx,vy,vz])
        if lt_ctrl is None:
            lt_ctrl = x
            control_gain = 0
        
        func = control_gain*np.linalg.norm(x[0:3]-lt_ctrl)-2*np.linalg.norm(np.array([vx,vy,vz])+x[0:3]*trj_dt) + 40*pp_d + 40*pp_l2
        return func
    
    @staticmethod
    def fun_dyn(x,px,py,pz,vx,vy,vz,ax,ay,az,obstacle,pred_dt,loc_goal,control_gain,speed_cost_gain,to_goal_cost_gain,para_g,ve):
        # func = to_goal_cost_gain*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3]) - para_g*loc_goal)+speed_cost_gain*(x[3]-0.01)**2 #+speed_cost_gain*abs(min(x[3],np.sign(x[3])*50)) #+ 50*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3]) - para_g*loc_goal)
        func = speed_cost_gain*x[3] + to_goal_cost_gain*np.linalg.norm(np.array([vx+x[0]*x[3],vy+x[1]*x[3],vz+x[2]*x[3]])-ve) + to_goal_cost_gain*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3]) - para_g*loc_goal)
        return func
    
    @staticmethod
    def calculate1(self,control,loc_goal,state,obstacle,b2e,path_rec,min_dis):

        # self.max_accel = min(0.5 , self.max_speed*0.6)
        # loc_goal = np.matmul(b2e, loc_goal)
        d_goal=np.linalg.norm(loc_goal)
        vx=self.max_speed*loc_goal[0]/d_goal#v
        vy=self.max_speed*loc_goal[1]/d_goal
        vz=self.max_speed*loc_goal[2]/d_goal
        dv=np.linalg.norm(np.array([vx,vy,vz])-state[3:6])
        aa=np.linalg.norm(np.array([vx,vy,vz]))
        bb=np.linalg.norm(state[3:6])
        cos_dv=(aa**2+bb**2-dv**2)/(2*aa*bb)
        vm=bb*cos_dv
        dv_p=bb*(1-cos_dv**2)**0.5
        
        if self.obs_v==1:
            self.max_accel*=self.acc_CD
        # if self.no_path:
        #     self.max_accel*=2
        #     print("no path found, accel increase",self.no_path)
#        elif bb<0.3 :
#            self.max_accel=self.max_speed+0.1
        if len(obstacle):
            min_d=np.linalg.norm(obstacle[0])
        else:
            min_d =3
#        if min_d > 3:
#            min_d==3
        
        if dv>self.max_accel:
            if (min_d>2*self.uav_r) or cos_dv<=0 or cos_dv > 0.96 : 
#                self.max_accel=self.max_speed
#                self.max_accel+=0.3
                print("fast mode!!",[vx,vy,vz],self.max_accel)
                vx=state[3]+self.max_accel*(vx-state[3])/dv
                vy=state[4]+self.max_accel*(vy-state[4])/dv
                vz=state[5]+self.max_accel*(vz-state[5])/dv*1.5
            elif (cos_dv>0 and dv_p<self.max_accel) : 
                delt_v=(self.max_accel**2-dv_p**2)**0.5
                vm=vm+delt_v
                vx=vm/aa*vx  # it is more safe
                vy=vm/aa*vy
                vz=vm/aa*vz
                print("safe mode1!!")
            else:   #(cos_dv>0 and dv_p>self.max_accel):
#                vx=state[3]+self.max_accel*(-state[3])/bb
#                vy=state[4]+self.max_accel*(-state[4])/bb
#                vz=state[5]+self.max_accel*(-state[5])/bb
                vx=state[3]+self.max_accel*(vx-state[3])/dv
                vy=state[4]+self.max_accel*(vy-state[4])/dv
                vz=state[5]+self.max_accel*(vz-state[5])/dv
                # vx*=0.6
                # vy*=0.6
                # vz*=0.6
                print("safe mode2!!")

        if np.linalg.norm(np.array([vx,vy]))>max(self.max_speed,0.8)*0.8 and vz<self.max_speed*0.3:
            print("increase z vel!")
            vz+=0.4
            vz=min(vz,self.max_speed)
            vxy = (self.max_speed**2-vz**2)**0.5
            rati = vxy/np.linalg.norm(np.array([vx,vy]))
            vx*=rati*vx*self.vel_coe
            vy*=rati*vy*self.vel_coe
        else:
            vz += 0.1
        if state[2]-self.downb<0.3:
            vz=0.6
        vz= max(-0.1,vz)
        print('set velocity:',[vx,vy,vz],"speed limit:",self.max_speed)
        print('position and goal',state[0:3],loc_goal)
        print('state:',state)
        print('number of points:',len(obstacle))
        return vx,vy,vz
    @staticmethod
    def getDis(pointX,pointY,pointZ,lineX1,lineY1,lineZ1,lineX2,lineY2,lineZ2):
        a=lineY2-lineY1
        b=lineX1-lineX2
        c=lineX2*lineY1-lineX1*lineY2
        ab=np.linalg.norm([pointX-lineX1,pointY-lineY1,pointZ-lineZ1])**2
        ac=np.linalg.norm([pointX-lineX2,pointY-lineY2,pointZ-lineZ2])**2
        bc=np.linalg.norm([lineX2-lineX1,lineY2-lineY1,lineZ2-lineZ1])**2
        QP=np.array([pointX-lineX1,pointY-lineY1,pointZ-lineZ1])
        v=np.array([lineX2-lineX1,lineY2-lineY1,lineZ2-lineZ1])
        if ac>ab+bc or ab>ac+bc: #or ac>ab+bc:
        # ang1=abs(Angle([pointX-lineX1,pointY-lineY1],[lineX2-lineX1,lineY2-lineY1]))
        # ang2=abs(Angle([pointX-lineX2,pointY-lineY2],[lineX1-lineX2,lineY1-lineY2]))
        # if ang1>pi/2 or ang2>pi/2:
            dis=10
        else:
            #fabs(a*pointX+b*pointY+c))/(pow(a*a+b*b,0.5))
            dis = np.linalg.norm(np.cross(QP, v))/(bc**0.5)
        if dis==0:
            print("getdis input:",[[pointX,pointY,pointZ],[lineX1,lineY1,lineZ1],[lineX2,lineY2,lineZ2]])
        return dis
    @staticmethod
    def d_obstacle(self,x,obstacle,cir_num,no_path_last,loc_goal_old,pts_size=[]):
        self.iter+=1
        d_ob=float("inf")
        vmod=self.max_speed
        if len(obstacle)==0:
            d_ob=float("inf")
        elif no_path_last == 2 and np.linalg.norm(loc_goal_old-x) < 0.5:  # if no wp is found last step , reject wp which is too close to it.
            d_ob = 0
        elif np.linalg.norm(obstacle[0]) < self.uav_r * 0.6:
            d_ob = 0
        else:
            #for i in range(1,len(Traj)):
            for j in range(len(obstacle)):
                if len(pts_size):
                    size = pts_size[j]
                else:
                    size = 0
                #if min(px-0.1*x[1],px+self.predict_coe*x[0])<obstacle[j,0]<max(px-0.1*x[1],px+self.predict_coe*x[0]) and min(py-0.1*x[1],py+self.predict_coe*x[1])<obstacle[j,1]<max(py-0.1*x[1],py+self.predict_coe*x[1]):
                getdis=self.getDis(obstacle[j][0],obstacle[j][1],obstacle[j][2],0,0,0,x[0],x[1],x[2])
                if getdis<self.uav_r+size:#np.linalg.norm(Traj[i][0:2]-obstacle[j])<self.uav_r:
                        #d_ob=np.linalg.norm(Traj[i][0:2]-obstacle[j])    # only x,y for 2d map
                    d_ob=getdis
                    break
                # if d_ob!=float("inf"):
                #     break
            if len(self.c_dyn)!=0 and d_ob==float("inf"):
                avoi_ang=[]
                avoi_angyz=[]
                vlist=[]
                safe_c= 0.2
                dyn_sf_r = min(self.uav_r + safe_c +max(self.r_dyn) ,0.9)
                vc=0
                dis=[]
                msp_coe = 1.0
                # print('c_dyn:',self.c_dyn,'v_dyn:',self.v_dyn)
                uav_v=x*np.linalg.norm(self.velocity)/np.linalg.norm(x)
                self.v_dyn =np.array(self.v_dyn) * 1.0
                for i in range(len(self.c_dyn)):
                    rela_v=x*self.velocity/np.linalg.norm(x)-self.v_dyn[i]
                    bc=np.linalg.norm(rela_v)
                    QP=self.c_dyn[i]-self.local_pos
                    ll=np.linalg.norm(QP-rela_v)
                    if ll<=np.linalg.norm(QP)+bc:
                        dis.append(np.linalg.norm(np.cross(QP, rela_v))/(bc))
#                    avoi_ang.append([atan2(QP[0],QP[1])-sin(safe_c*dyn_sf_r/np.linalg.norm(QP)),atan2(QP[0],QP[1])+sin(safe_c*dyn_sf_r/np.linalg.norm(QP))])
#                    avoi_angyz.append([atan2(QP[1],QP[2])-sin(safe_c*dyn_sf_r/np.linalg.norm(QP)),atan2(QP[1],QP[2])+sin(safe_c*dyn_sf_r/np.linalg.norm(QP))])
                    avoi_ang.append([atan2(QP[0],QP[1])-sin((dyn_sf_r+self.r_dyn[i])/np.linalg.norm(QP)),atan2(QP[0],QP[1])+sin((dyn_sf_r+self.r_dyn[i])/np.linalg.norm(QP))])
                    avoi_angyz.append([atan2(QP[1],QP[2])-sin((dyn_sf_r+self.r_dyn[i])/np.linalg.norm(QP)),atan2(QP[1],QP[2])+sin((dyn_sf_r+self.r_dyn[i])/np.linalg.norm(QP))]) 
                    print("relative velocity distance to dynobs",dis)
#                    print("radius of moving obstacles",self.r_dyn)
#                dis=min(dis)
#                if dis<safe_c*dyn_sf_r: #considering the shape of the dynobs and the safe radious, the dis should be much bigger than uav_r 
#                    d_ob=dis
#                print('np.array(dis)-(dyn_sf_r+self.r_dyn)',np.array(dis)-(dyn_sf_r+self.r_dyn))
                if (np.array(dis)-(dyn_sf_r)<0).any(): #considering the shape of the dynobs and the safe radious, the dis should be much bigger than uav_r 
                    d_ob=dis[np.argmin(np.array(dis)-(dyn_sf_r+self.r_dyn))]
#                    d_ob=abs(min(np.array(dis)-(dyn_sf_r+self.r_dyn)))
                    dis=d_ob
                    
                #else:
                    #self.obs_v=1
                    #self.max_speed= np.linalg.norm(self.velocity)

                if d_ob!=float("inf"):
                    avoi_ang=np.array(avoi_ang).reshape(1,-1)[0]
                    avoi_angyz=np.array(avoi_angyz).reshape(1,-1)[0]
                    # vnum=np.argmin(abs(avoi_ang-atan2(rela_v[0],rela_v[1])))
                    # max_speed=np.linalg.norm(self.velocity) + self.pred_dt * self.max_accel*self.acc_CD
                    # min_speed=np.linalg.norm(self.velocity) - self.pred_dt * self.max_accel*self.acc_CD
                    if cir_num<3 and cir_num!=0:
                        for vnum in range(len(avoi_ang)):
#                            max_speed=min(np.linalg.norm(self.v_dyn[int(vnum/2)]),self.m_speed)
                            
                            ang_v=avoi_ang[vnum]
                            avoi_v=np.array([sin(ang_v),cos(ang_v)])
                            print('avoi_v',avoi_v)
                            # print(np.r_[uav_v[0:2],-avoi_v].reshape(2,2).T)
			    
                            vmod=np.dot(np.linalg.inv(np.r_[uav_v[0:2],-avoi_v].reshape(2,2).T),np.array(self.v_dyn[int(vnum/2)][0:2]))[0]
                            
                            uav_v_nxt=x*vmod/np.linalg.norm(x)
                            print('vmod',vmod,'uav_v_nxt',uav_v_nxt)
                           # ( np.linalg.norm(uav_v_nxt[0:2]- self.velocity[0:2]) < self.acc_CD*self.max_accel*self.pred_dt and 
                            if vmod > 0 and np.linalg.norm(uav_v_nxt[0:2]- self.velocity[0:2]) < self.acc_CD*self.max_accel*self.pred_dt*2:# or (vmod<0 and vmod>-0.3*self.max_speed):
                                vlist.append(vmod)
                                d_ob=float("inf")
                                self.obs_v=1
                                vc+=1
                            if (vnum+1 & 1)==0 : #if we can't find the speed under the limit that can avoid the ith obstacle, break the circle and this current search direction is skipped 
                                if vc==0:
                                    d_ob=dis
                                    break
                                else:
                                    vc=0
                    vc=0
                    if cir_num>2 or (cir_num==0 and d_ob!=float("inf")):
                        for vnum in range(len(avoi_angyz)):
#                            max_speed=min(np.linalg.norm(self.v_dyn[int(vnum/2)]),self.m_speed)
                            # max_speed=self.m_speed
                            ang_v=avoi_angyz[vnum]
                            avoi_v=np.array([sin(ang_v),cos(ang_v)])
                            print('avoi_v',avoi_v)
                            # print(np.r_[uav_v[0:2],-avoi_v].reshape(2,2).T)
                            vmod=np.dot(np.linalg.inv(np.r_[uav_v[1:3],-avoi_v].reshape(2,2).T),np.array(self.v_dyn[int(vnum/2)][1:3]))[0]
                            
                            uav_v_nxt=x*vmod/np.linalg.norm(x)
                            print('vmod',vmod,'uav_v_nxt',uav_v_nxt)
                             # ( np.linalg.norm(uav_v_nxt[1:3]- self.velocity[1:3]) < self.acc_CD*self.max_accel*self.pred_dt and
                            if vmod > 0 and np.linalg.norm(uav_v_nxt[1:3]- self.velocity[1:3]) < self.acc_CD*self.max_accel*self.pred_dt*2:# or (vmod<0 and vmod>-0.3*self.max_speed):
                                vlist.append(vmod)
                                d_ob=float("inf")
                                self.obs_v=1
                                vc+=1
                            if (vnum+1 & 1)==0 :
                                if vc==0:
                                    d_ob=dis
                                    break
                                else:
                                    vc=0
                    if len(vlist)>0 and d_ob==float("inf"):
                        print("old speed:",self.velocity)
                        vlist=np.array(vlist)
                        sp1=max(vlist)
                        sp2=min(vlist)
                        sp=np.array([sp1,sp2])
          #              if sp1>self.max_speed and sp2<self.max_speed:            #find the final speed in the list of feasible speed (vlist)
          #                  self.max_speed=sp[np.argmin(abs(sp-self.max_speed))]
          #              elif sp1>self.max_speed and sp2>self.max_speed:
          #                  self.max_speed=max(sp)
          #              elif sp1<self.max_speed and sp2<self.max_speed:
          #                  self.max_speed=min(sp)
                        self.max_speed=sp[np.argmin(abs(sp-np.linalg.norm(self.velocity)))]
                        print("speed:",vlist,self.max_speed)
                        # dv=np.linalg.norm(x*self.max_speed/np.linalg.norm(x)-self.velocity)
                        # if dv>self.acc_CD*self.max_accel:
                        #     d_ob=dis
                    
        print('d_ob:',d_ob)

        self.dmin.append(d_ob)
        self.wps.append(x)
        if d_ob!=float("inf") and d_ob>self.uav_r:
            d_ob=self.uav_r-0.1
        return d_ob
  
    @staticmethod
    def distance_filter(self,pcl,f_angle,loc_goal,p_near,p_num):
        min_dis=3
        filtered_pcl=[]

        i=0
        for point in pcl:
            point=pcl[i]
            
            # d_point=np.linalg.norm(point)
            if i<p_near or (i<=p_num and abs(self.Angle(point[0:2],loc_goal[0:2]))<pi/1.5):
                # print('p_in_front:',point)
                filtered_pcl.append(point)
                i+=1
                # print('filtered_pcl-point',point,[d_point])
            elif i>p_num:
                break
        
        filtered_pcl=np.array(filtered_pcl)
        return filtered_pcl
    @staticmethod
    def fd_3d_goal1(self,local_pos,pcl,d3_check,loc_goal):
        # angle_locgoal=np.array([self.Angle(loc_goal[0:2],[1,0]),self.Angle([np.linalg.norm(loc_goal[0:2]),loc_goal[2]],[1,0])])
        print(d3_check)
        
        if d3_check is None or (d3_check[1] == 0).all():
            return loc_goal
        elif d3_check is not None and (d3_check[1] == 1).all():
            d3_check[0,2] = 1.5
            return d3_check[0]
        elif (d3_check[1] == d3_check[2]).all():
            detect_rg = 1*np.linalg.norm(d3_check[2,0:2]-local_pos[0:2])    # for a better look
        else:
            detect_rg = np.linalg.norm(d3_check[1,0:2]-local_pos[0:2])
        detect_rg = max(3,detect_rg)
        print("3d goal search radius",detect_rg)
        g_goal = d3_check[-1]-local_pos
        f_goal = d3_check[0]-local_pos
        if_direct = self.d_obstacle(self, g_goal,pcl,0,0,0)>self.uav_r
        if if_direct:
            print("no collision, directly to final goal")
            return g_goal
        angle_locgoal=np.array([self.Angle(g_goal[0:2],[1,0]),self.Angle([np.linalg.norm(g_goal[0:2]),g_goal[2]],[1,0])])
        max_det_ang = abs(angle_locgoal[0]-self.Angle(f_goal[0:2],[1,0]))
        if max_det_ang > pi:
            max_det_ang = 2*pi - max_det_ang
        print("max angular search range for 3D goal:",max_det_ang)
        if max_det_ang < 2*self.det_ang:
            return loc_goal
        ang_pix = []
        max_grid=int(pi/2*1.2/self.det_ang)*2+3
        # if max_grid%2 ==0:
        #     max_grid-=1                                # it is odd
        h_gridlth=int((max_grid-1)/2)
        grid_graph=np.zeros([max_grid,max_grid]).tolist()
        grid_graph01=np.zeros([max_grid,max_grid])
        ang_pix_ele = np.array([0,0])
        for pt in pcl:
            angle_pt=np.array([self.Angle(pt[0:2],[1,0]),self.Angle([np.linalg.norm(pt[0:2]),pt[2]],[1,0])])
            ang_dif = angle_pt - angle_locgoal
            if (np.rint(ang_dif/self.det_ang).astype(int) != ang_pix_ele).any() and np.linalg.norm(ang_dif) < pi/2*1.2 :# and local_pos[2]+self.predict_coe*self.detect_l*sin((min_lsua-1.5)*self.det_ang+angle_locgoal[1])> 0:  # (abs(ang_dif) < pi/2).all() and 
                ang_pix_ele=np.rint(ang_dif/self.det_ang).astype(int)
                ang_lt = np.linalg.norm(ang_dif/self.det_ang-ang_pix_ele)
                #print(ang_pix_ele,h_gridlth)
                if isinstance(grid_graph[ang_pix_ele[0]+h_gridlth][ang_pix_ele[1]+h_gridlth],list):
                    grid_graph[ang_pix_ele[0]+h_gridlth][ang_pix_ele[1]+h_gridlth].append(np.r_[pt,ang_lt])
                else:
                    grid_graph[ang_pix_ele[0]+h_gridlth][ang_pix_ele[1]+h_gridlth]=[np.r_[pt,ang_lt]]
                grid_graph01[ang_pix_ele[0]+h_gridlth,ang_pix_ele[1]+h_gridlth]=1
                ang_pix.append(ang_pix_ele)
        
        if (not len(ang_pix)):
            print("find 3d goal: go directly")
            return loc_goal  # no_path=0 indicates there exists a feasible path

        ang_pix = np.array(ang_pix)
        ang_edge = []
      
        for ii in range(1,h_gridlth+1):
            adj3 = grid_graph01[h_gridlth-1:h_gridlth+2,h_gridlth+ii-1:h_gridlth+ii+2]
            if (adj3 == 0).all():
                ang_edge.append([0,ii])
                break
            adj4 = grid_graph01[h_gridlth-ii-1:h_gridlth-ii+2,h_gridlth+ii-1:h_gridlth+ii+2]
            if (adj4 == 0).all():
                ang_edge.append([-ii,ii])
                break
            adj5 = grid_graph01[h_gridlth+ii-1:h_gridlth+ii+2,h_gridlth+ii-1:h_gridlth+ii+2]
            if (adj5== 0).all() :
                ang_edge.append([ii,ii])
                break
        print("find wp: ang_edge",ang_edge)
        if len(ang_edge): 
            # ang_goal = np.array(ang_edge[np.argmin(np.linalg.norm(np.array(ang_edge),axis=1))])
            ang_goal = np.array(ang_edge[0])
        else:
            return loc_goal
        print("find wp: ang_goal",ang_goal)
        uniq_a2=np.unique(ang_pix,axis=0)
        ang_goal1 = uniq_a2[np.argmin(np.linalg.norm(uniq_a2 - ang_goal,axis=1))]
        print("test", np.array(grid_graph[ang_goal1[0]+h_gridlth][ang_goal1[1]+h_gridlth]))
        p_index = np.argmax(np.array(grid_graph[ang_goal1[0]+h_gridlth][ang_goal1[1]+h_gridlth])[:,3])
        p_len= np.linalg.norm(np.array(grid_graph[ang_goal1[0]+h_gridlth][ang_goal1[1]+h_gridlth])[p_index,0:3])
        # ang_safe=1+int(sin(min(1,self.uav_r/p_len)))
        ang_safe=asin(min(1,self.uav_r/p_len))/self.det_ang
        
        if ang_safe:
            print("p_len", p_len, ang_safe)
            unit_safe_vec = (ang_goal-ang_goal1)/np.linalg.norm(ang_goal1-ang_goal)
            if unit_safe_vec[1]<0:
                unit_safe_vec[0] =  np.sign(unit_safe_vec[0])
                unit_safe_vec[1]  = 0
            ang_goal = (ang_goal + ang_safe*(ang_goal-ang_goal1)/np.linalg.norm(ang_goal1-ang_goal))*self.det_ang
        else:
            ang_goal = (ang_goal)*self.det_ang
        
        # if np.linalg.norm(ang_goal) > max_det_ang:
        #     return loc_goal,angle_locgoal,no_path,all_edge_goal
        ang_goal = ang_goal + angle_locgoal
        loc_goal = np.array([self.detect_l*cos(ang_goal[0])*self.predict_coe*cos(ang_goal[1]),
                            self.detect_l*sin(ang_goal[0])*self.predict_coe*cos(ang_goal[1]),
                            self.detect_l*sin(ang_goal[1])*self.predict_coe])

        if np.linalg.norm(ang_goal) > max_det_ang:
            return loc_goal
        ang_goal = ang_goal + angle_locgoal
        d3_goal = np.array([detect_rg*cos(ang_goal[0])*self.predict_coe*cos(ang_goal[1]),
                            detect_rg*sin(ang_goal[0])*self.predict_coe*cos(ang_goal[1]),
                            detect_rg*sin(ang_goal[1])*self.predict_coe])
        self.if_d3_goal = 1
        return d3_goal
    @classmethod
    def start(self,pcl,pcl_cam,state,goal,r,p,y,loc_goal_old,f_angle,path_rec,pretime,dyn_time,
              if_direct,no_path,pp_restart,pcl_time,d3_check,d3_pos_real,glb_goal_3d,if_d3_goal,lst_control,wpts0):
        self.pp_restart = pp_restart
        self.if_d3_goal = if_d3_goal
        all_edge_goal=[]
        if pretime==0:
            pred_dt=0
        else:
            pred_dt=time.time()-pretime
        self.pred_dt = pred_dt
        pretime=time.time()
        starttime1 = time.time()
        control_method.init(self)
        e2b = earth_to_body_frame(r, p, y)
        b2e = body_to_earth_frame(r,p,y)
        # loc_goal = np.matmul(e2b, goal)
        loc_goal=goal
        print('r,p,y:',[r,p,y],'loc_goal in body:',loc_goal)
        self.detect_l = max(0.5,min(np.linalg.norm(loc_goal),self.detect_l))
        # print(pcl) pcl1=[]
        self.velocity=state[3:6]
        local_pos=state[0:3]+self.pred_coe*pred_dt*self.velocity
        self.local_pos=state[0:3]+self.pred_coe*pred_dt*self.velocity

        print('num-octomap',len(pcl))
        print('num-depthcam',len(pcl_cam))
        self.if_direct = if_direct
        pcl_map = []
        if len(pcl)>0:
            pcl=pcl-local_pos
            # pcl=pcl[::2]
            
            if len(pcl)>self.pnum_map:
                pcl=control_method.distance_filter(self,pcl,f_angle,loc_goal,0,self.pnum_map)
            pcl_map = pcl.copy()
        num_dyn=0
        if len(pcl_cam)>0:
            if pcl_cam[-1][0]!=0:
                print('number of dyn!!!!!!:',pcl_cam[-1][0])
                num_dyn=int(pcl_cam[-1][0])
                now = rospy.get_rostime()
                pcl_dt = now.secs + now.nsecs*1e-9 - pcl_time
                print("pcl_dt:",pcl_dt)
                self.v_dyn=pcl_cam[-1-2*num_dyn:-num_dyn-1]
                # print(np.array(pcl_cam[-1-3*num_dyn:-2*num_dyn-1]) ,np.array(self.v_dyn))
                print(pcl_cam[-1-3*num_dyn::])
                self.c_dyn=np.array(pcl_cam[-1-3*num_dyn:-2*num_dyn-1]) + np.array(self.v_dyn) * self.pred_coe*(pred_dt+pcl_dt)
                self.r_dyn=0.5*np.array(pcl_cam[-num_dyn-1:-1])[:,0]
                
#                print('r of mov:',0.5*np.array(pcl_cam[-num_dyn-1:-1]))
                pcl_cam=pcl_cam[0:-1-3*num_dyn]
            
            if len(pcl_cam)>0:
                # print(pcl_cam)
                pcl_cam=pcl_cam-local_pos
                pcl_cam=control_method.distance_filter(self,pcl_cam,f_angle,loc_goal,0,self.p_num)
                
                if len(pcl_cam)>self.p_num*0.8:
                    pcl_cam_1=pcl_cam[int(self.p_num*2/3)::2]
                    pcl_cam=np.r_[pcl_cam[0:int(self.p_num*2/3)],pcl_cam_1]
        min_dis=0
        if num_dyn != 0:
            dyn_time  =pretime
        self.num_dyn = num_dyn
        if len(pcl) >0 and len(pcl_cam)>0:
            min_dis=min(np.linalg.norm(pcl[0]),np.linalg.norm(pcl_cam[0]))
#             min_dis=np.linalg.norm(pcl_cam[0])
        elif len(pcl_cam):
            min_dis=np.linalg.norm(pcl_cam[0])
        elif len(pcl):
#            min_dis=np.linalg.norm(pcl[0])*(1-0.2)+self.dis_p1*0.2
            min_dis=np.linalg.norm(pcl[0])
# 
        if min_dis !=0 and num_dyn ==0:
            self.max_speed=max(self.max_speed*min(1,min_dis/self.pcl_fl)**0.8,self.min_speed+0.3)
        self.uav_r = (np.linalg.norm(self.velocity)*(self.max_speed+(self.max_speed**2+2*self.max_accel*self.dis2wp)**0.5))/(4*self.max_accel)
        self.uav_r=max(min(self.uav_r,0.8),0.4)
        print("self.uav_r",self.uav_r)
        # control=np.array([0, 0, 0, 1])
        pclall=[]
        if len(pcl_cam) and len(pcl_map):
           
            if np.linalg.norm(pcl_cam[0]) > np.linalg.norm(pcl_map[0]):
                pclall=np.r_[pcl_map[0:min(self.pnum_map,len(pcl_map))],pcl_cam]
            else:
                pclall=np.r_[pcl_cam,pcl_map[0:min(self.pnum_map-20,len(pcl_map))]]
                
        elif len(pcl_map):
            pclall=pcl_map
        elif len(pcl_cam):
            pclall=pcl_cam
        if len(glb_goal_3d) ==0 or np.linalg.norm(glb_goal_3d-self.local_pos)< 0.5 or (np.linalg.norm(self.local_pos-d3_pos_real)>self.d3_fly_leth and self.d_obstacle(self, glb_goal_3d-self.local_pos ,pcl_map,0,0,0)<0.4): #:
            loc_goal_3d = []
            loc_goal_pl = loc_goal
            print("update 3d goal")
        else:
            print("don't update 3d goal")
            loc_goal_3d = glb_goal_3d-self.local_pos
            loc_goal_pl = loc_goal_3d
            
        if len(pcl_map) and (len(loc_goal_3d) == 0):  #np.linalg.norm(loc_goal_3d)< 0.4):#
            d3_time = time.time()
            # loc_goal_3d = control_method.check_goal(self,local_pos,pcl_map,d3_check,loc_goal)
            self.if_d3_goal = 0
            loc_goal_pl = control_method.fd_3d_goal1(self,local_pos,pcl_map,d3_check,loc_goal)
            # self.if_d3_goal = 0
            if self.if_d3_goal:
                # d3_time_real = time.time()
                d3_pos_real = self.local_pos.copy()
                print("better 3d goal is found!")
                loc_goal_3d = loc_goal_pl
            
            print("find 3d path time cost:",time.time()-d3_time)

        starttime11 = time.time()
        # if len(pclall)>0:
        print("point number for wp search:",len(pclall))
        wpts,wpts_mot,f_angle,no_path,pcl_w=control_method.get_wpts(self,local_pos,pclall,loc_goal_pl,loc_goal_old,state.copy(),no_path,wpts0,f_angle)  # get the waypoints
        
        starttime2 = time.time()
        traj_dif = 1
        control = None
        pred_trj = []
        if no_path == 2:
            vx,vy,vz = 0,0,0
            print("no path found, go back!!!")
        elif np.linalg.norm(self.velocity[0:2]) > 0.3:
        #else:
            vx,vy,vz,traj_dif,control,pred_trj = control_method.calculate(self,lst_control,wpts_mot,state.copy(),pcl,b2e,path_rec,pred_dt)
        if traj_dif > 0.08 and num_dyn ==0:
            self.max_speed = min(np.linalg.norm(self.velocity) + 0.3,self.max_speed)
            vx,vy,vz = control_method.calculate1(self,lst_control,wpts_mot[0],state.copy(),pcl,b2e,path_rec,min_dis)
             #test:velocity control
        steptime = (time.time() - starttime1)
        wptime = (starttime2 - starttime11)
        optitime = (time.time() - starttime2)
        print("Step Time used:",steptime,"find local goal time:",wptime)

        if len(pclall):
            global_pcl = pclall+local_pos
        else:
            global_pcl = pclall
        return [vx,vy,vz],loc_goal,f_angle,steptime,wptime,optitime,self.iter,len(pcl),dyn_time,pretime,global_pcl ,self.if_direct,no_path,traj_dif,loc_goal_3d,d3_pos_real,self.if_d3_goal,control, wpts_mot+local_pos,pred_trj,pcl_w

