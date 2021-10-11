#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
#include <Tools.h>
#include <ros/ros.h>

#include <ros_missn.h>
//#include <controllers/backstepping.h>
#include <planner_fast.h>
#include <Tools.h>
//#include <logs/log_flight.h>

using namespace std;
using namespace Eigen;

#define CtrlFreq 50
#define MaxVel 2.0

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv,"traj_planner");   
    ros::NodeHandle nh;                // Create a handle to this process node.
    RosClass flying(&nh, CtrlFreq);
    Vector3d ct_pos,ct_vel,ct_acc;
    ct_pos.setZero();
    Vector3d p_d, v_d, a_d, p_d_yaw;
    MatrixXd sp_pos, sp_vel, sp_acc;
    TrajectoryGenerator_fast reference;
    // ct_pos<<0,0,4;
    // Intialize Planner
    // cout << "ctpos:\n"<<ct_pos<<endl;
    States state;
    cout << "Traj node initialized!" << endl;
    do{state = flying.get_state();
       ct_pos = state.P_E;
            ros::Duration(0.1).sleep();}
    while (ct_pos.norm() < 1e-3);
    cout << "UAV message received!" << ct_pos << endl;
    while (!flying.corridor_update or flying.waypoints.rows() < 2)
    { ros::Duration(0.05).sleep();
      state = flying.get_state();
    }
    cout << "Corridor received!" << flying.waypoints << endl<< flying.cd_r<< endl;
    state = flying.get_state();
    ct_pos = state.P_E;
    ct_vel = state.V_E;
    ct_acc = state.A_E;
    reference.replan_traj(MaxVel,ct_pos,ct_vel,ct_acc,flying.waypoints,flying.cd_c,flying.cd_r);
    double timee, traj_last_t;
    traj_last_t =ros::Time::now().toSec();
    
    while (nh.ok())
    {
    double t1 = ros::Time::now().toSec();
    if (!flying.corridor_update)
    {
     ros::Duration(1/CtrlFreq).sleep();
     timee = ros::Time::now().toSec() - traj_last_t + 1/CtrlFreq;
     state = flying.get_state();
     if (reference.total_t < timee+0.1)
     {cout << "reach goal, wait for new path!" << endl;
      
      ros::Duration(0.5).sleep();
      continue;
     }
       
       // cout << "(corridor not update)" << endl;
    }
    else{
    flying.set_cod_update(false);
    state = flying.get_state();
    ct_pos = state.P_E;
    ct_vel = state.V_E;
    ct_acc = state.A_E;
    cout << "the updated waypoints:" << flying.waypoints << endl;
    bool if_safe = reference.check_traj_safe(flying.cd_c, flying.cd_r, ros::Time::now().toSec() - traj_last_t);
    
    cout << "corridor update, check safety result:" << if_safe <<endl;
    if (if_safe && (timee < 0.8))
    {
    //TrajectoryGenerator_fast reference(MaxVel,ct_pos,ct_vel,ct_acc,flying.waypoints,flying.cd_c,flying.cd_r);
   // double FlyTime = reference.get_duration();
//     Vector2d v2 = (reference.waypoints.col(1).head(2) - reference.waypoints.col(0).head(2));
//     Vector2d v1;
//     v1<<1.0,0.0;
//     double desire_psi=acos(v1.dot(v2) /(v1.norm()*v2.norm())); 
//     
//     if (v2(1)<0)
//     {desire_psi = -desire_psi;}
 
     timee = ros::Time::now().toSec() - traj_last_t + 1/CtrlFreq;
        
        // cout<<"44"<<endl;
        // calculate control inputs
        
        }
    
    else{
    reference.replan_traj(MaxVel,ct_pos,ct_vel,ct_acc,flying.waypoints,flying.cd_c,flying.cd_r);
    traj_last_t =ros::Time::now().toSec();
    timee = 1/CtrlFreq;
    cout << "old traj is not safe" << endl;
    
    }
     cout << "traj time cost: " << ros::Time::now().toSec()-t1 <<endl<<ros::Time::now().toSec()<<endl<<t1<<endl<<traj_last_t<<endl;  
       }
      // cout << "timee:"<<timee<<endl;
        reference.get_desire(timee, p_d, v_d, a_d,p_d_yaw);
    //    bsc.controller(state, p_d, v_d, a_d,p_d_yaw,next_goal);
        // cout<<"55"<<endl;
        // step forward
        Vector2d v2 = (p_d_yaw - state.P_E).head(2);
        Vector2d v1;
        v1<<1.0,0.0;
        double desire_psi=acos(v1.dot(v2) /(v1.norm()*v2.norm())); 
        
        if (v2(1)<0)
        {desire_psi = -desire_psi;}
        state = flying.step(desire_psi, p_d, v_d, a_d, "pos_vel_acc_yaw"); 
      //  cout << "state setpoint:" << desire_psi <<"\n"<< p_d<<"\n"<<v_d<<"\n"<< a_d << endl;
        // state = flying.step(0.0, bsc.Vc, "yaw_n_velocity"); 
        // cout<<"flying.step: \n"<<endl;
        // log
        // logger.desires(p_d, v_d, a_d);
        // logger.states(state);
        // logger.actors(bsc.forceCtrl, bsc.eulerCtrl);

        // break if crashed
//         if (flying.done)
//         {
//             break;
//         }
       // cout << "pub traj (out):" << reference.total_t <<endl;
        reference.get_traj_samples(sp_pos, sp_vel,sp_acc, ros::Time::now().toSec() - traj_last_t);
        flying.pub_traj (sp_pos, sp_vel,sp_acc);
    
    
    ros::Duration(1/CtrlFreq).sleep();
    
    }

  
    return 0;}


