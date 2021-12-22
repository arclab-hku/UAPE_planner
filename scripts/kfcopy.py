#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 27 18:29:51 2021

@author: chen
"""

            lastkf_c = copy.copy(c_dyn_l)
            lastkf_v = copy.copy(v_dyn_l)
            c_dyn_l,v_dyn_l,if_track=[],[],[]
            x_mat=[]
            for kk in range(len(c_dynkf)):
                if kk == 0:
                    if last_kftime:
                        kf_dt = time.time()-last_kftime
                    else:
                        kf_dt = protime
                    last_kftime = time.time()
                # set initial value
                if (len(c_dyn1)!=0): #dyn_obs_reset==1 or 
                    z_mat = np.mat([c_dynkf[kk],v_dynkf[kk]])
                    p_mat = np.mat([[1, 0], [0, 1]])   # initial state Covariance matrix
                    print("found dynamic!")
                elif len(c_dyn1)==0:
                    c_dynkf[kk] = c_dynkf[kk]+v_dynkf[kk]*kf_dt
                    z_mat = np.mat([c_dynkf[kk],v_dynkf[kk]])
                    p_mat = np.mat([[3, 0], [0, 3]])   # initial state Covariance matrix
                    print("no dynamic imformation, KF predict!")
                if len(lastkf_c):
                    dis_gaps = np.linalg.norm(np.array(z_mat)[0]-(np.array(lastkf_c)+kf_dt*np.array(lastkf_v)),axis=1)
                    mingap_kf = min(dis_gaps)
                    if mingap_kf < 0.3:
                        index_kf = np.where(dis_gaps==mingap_kf)[0][0]
                        if np.linalg.norm(lastkf_v[index_kf]-v_dynkf[kk]) < 0.6:
                            x_mat = np.mat([lastkf_c[index_kf],lastkf_v[index_kf]])
                            print("kf tracking!",np.linalg.norm(lastkf_v[index_kf]-v_dynkf[kk]),mingap_kf)
                            track_flag = 1
                        # else:
                        #     index_kf = -1
                if len(x_mat) ==0:
                    print("new KF initial")
                    x_mat = np.mat([[1, -kf_dt], [0, 1]]) * np.mat([c_dynkf[kk],v_dynkf[kk]])
                    track_flag = 0
                # z_mat = np.mat(c_dynkf[kk])
#                                    if len(c_dyn1)==0:
#                                        q_mat = np.mat([[0.001, 0], [0, 0.001]])
#
##                                        print("use KF predict as input")
#                                    else:
#                                        q_mat = np.mat([[0.1, 0], [0, 0.1]])  # State transition Covariance matrix
#                                        print("use ground truth as input")
                q_mat = np.mat([[0.1, 0], [0, 0.1]])
                
                f_mat = np.mat([[1, kf_dt], [0, 1]])     # State transition matrix
              
#                                    q_mat = np.mat([[0.1, 0], [0, 0.1]])
               
                h_mat = np.mat([[1, 0],[0,1]])  #State observation matrix
                r_mat = np.mat([4])     #State observation noise Covariance matrix
 
                
                x_predict = f_mat * x_mat
                p_predict = f_mat * p_mat * f_mat.T + q_mat
                kalman = p_predict * h_mat.T / (h_mat * p_predict * h_mat.T + r_mat)
                x_mat = x_predict + kalman *(z_mat - h_mat * x_predict)
                p_mat = (np.eye(2) - kalman * h_mat) * p_predict
                
                
                if len(c_dyn_l) == 0 or min(np.linalg.norm(np.array(x_mat)[0]-np.array(c_dyn_l),axis=1)) > 0.3:
                    c_dyn_l.append(list(np.array(x_mat)[0]))
                    v_dyn_l.append(list(np.array(x_mat)[1]))
                    if_track.append(track_flag)
                print("KF is used!!")
#                                    c_dynkf=c_dyn_l
#                                    v_dynkf=v_dyn_l
                x_mat = []
                dyn_obs_reset=0