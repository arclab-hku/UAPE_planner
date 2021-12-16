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
#include <path_searching/include/path_searching/kinodynamic_astar.h>
//#include <logs/log_flight.h>

using namespace std;
using namespace Eigen;

#define CtrlFreq 80
#define MaxVel 2.0

unique_ptr<KinodynamicAstar> kino_path_finder_;
// struct Config
// {


//     std::string odomFrame;

//     // Params
//     double scaleSI;
//     double mapHeight;
//     Eigen::Vector3d polyhedronBox;
//     double rho;
//     double totalT;
//     int qdIntervals;
//     double horizHalfLen;
//     double vertHalfLen;
//     double safeMargin;
//     double velMax;
//     double thrustAccMin;
//     double thrustAccMax;
//     double bodyRateMax;
//     double gravAcc;
//     Eigen::Vector4d penaltyPVTB;
//     bool useC2Diffeo;
//     double optRelTol;
//     double trajVizWidth;
//     Eigen::Vector3d trajVizRGB;
//     std::string routeStoragePath;
//     std::string ellipsoidPath;
//     Eigen::Vector4d ellipsoidVizRGBA;} config;
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv,"traj_planner");   
            // Create a handle to this process node.
    ros::NodeHandle nh("~");
    RosClass flying(&nh, CtrlFreq);
    Vector3d ct_pos,ct_vel,ct_acc;
    ct_pos.setZero();
    Vector3d p_d, v_d, a_d, p_d_yaw;
    a_d.setZero();
    MatrixXd sp_pos, sp_vel, sp_acc, waypoints_m;
    double last_path_t;
    TrajectoryGenerator_fast reference;
    Eigen::Vector3d end_state = Eigen::Vector3d::Zero(3);
    vector<double> goalp;
    double dis_goal;
    double sfck_t;
    nh.getParam("goal", goalp);
    nh.getParam("search/horizon", dis_goal);
    nh.getParam("sfck_t", sfck_t);
    dis_goal = dis_goal-0.5;
    Eigen::Vector3d g_goal = {goalp[0],goalp[1],goalp[2]};
    Eigen::Vector3d goal;
    bool if_initial = true;
    // cout << "mk1" << endl;
        
    reference.read_param(&nh);
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
    while (!flying.pcl_update)
    { ros::Duration(0.05).sleep();
      state = flying.get_state();
    }
    cout << "Point cloud received!\n" << endl;
    state = flying.get_state();
    ct_pos = state.P_E;
    ct_vel = state.V_E;
    ct_acc = state.A_E;
    cout << "reset env!\n" << endl;
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    cout << "set params !\n" << flying.obs_pointer << endl;
    kino_path_finder_->setEnvironment(flying.obs_pointer,flying.dynobs_pointer);
    cout << "set obs !\n" << endl;
    kino_path_finder_->init();
    
    if ((g_goal-ct_pos).norm()>dis_goal)
    {goal = ct_pos + (g_goal-ct_pos)/(g_goal-ct_pos).norm()*dis_goal;}
    else{goal = g_goal;}
    kino_path_finder_->reset();
    cout << "begin search!\n" << goal << endl;
    int status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, true);
    last_path_t = ros::Time::now().toSec();
    if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
   
    status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, false);
    last_path_t = ros::Time::now().toSec();
    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path. Please restart" << endl;
      return 0;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }}

    vector<Eigen::Vector3d> waypoints, start_end_derivatives;
    kino_path_finder_->getSamples(0.3, waypoints,start_end_derivatives);
    waypoints_m = Map<MatrixXd>(waypoints[0].data(),3,waypoints.size());
    reference.replan_traj(MaxVel,ct_pos,ct_vel,ct_acc,waypoints_m,flying.cd_c,flying.cd_r,flying.obs_pointer, flying.dynobs_pointer,ros::Time::now().toSec());
    double timee, traj_last_t;
    traj_last_t =ros::Time::now().toSec();
    
    while (nh.ok())
    {
      if ((g_goal-ct_pos).norm()>dis_goal)
    {goal = ct_pos + (g_goal-ct_pos)/(g_goal-ct_pos).norm()*dis_goal;}
    else{goal = g_goal;}
    double t1 = ros::Time::now().nsec;
    if (!flying.pcl_update) // || !flying.waypoint_update)  || !flying.trigger
    {
     ros::Duration(1/CtrlFreq).sleep();
     timee = ros::Time::now().toSec() - traj_last_t + 1/CtrlFreq;
     state = flying.get_state();
     if (reference.total_t < timee+0.2)
     {cout << "reach goal, wait for new path!" << endl;
      
      ros::Duration(0.5).sleep();
      continue;
     }
       
       // cout << "(corridor not update)" << endl;
    }
    else{
    flying.set_cod_update(false);
    flying.set_pcl_update(false);
    state = flying.get_state();
    // ct_pos = state.P_E;
    // ct_vel = state.V_E;
    // ct_acc = state.A_E;
    if (if_initial)
    { ct_pos = state.P_E;
    ct_vel = state.V_E;
    ct_acc = state.A_E;}
    else{
    ct_pos = p_d;
    ct_vel = v_d;
    ct_acc = a_d;}
    // ct_acc = Vector3d::Zero(3);
    //ros::Time::now().toSec() - last_path_t > 0.1 && 
    if ((goal-ct_pos).norm()>1.0 && flying.obs_pointer->size()>0)
    {
    chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();
    cout << "The obs pointer:\n" << flying.obs_pointer << "---pcl size: "<< flying.obs_pointer->size()<< endl;
    
    kino_path_finder_->setEnvironment(flying.obs_pointer,flying.dynobs_pointer);
    if (!kino_path_finder_->checkOldPath(waypoints) || ros::Time::now().toSec() - last_path_t > sfck_t)
    {
    kino_path_finder_->reset();
    status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, true);
    
    if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc,goal, end_state, false);
    last_path_t = ros::Time::now().toSec();
    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path. Please restart" << endl;
      // return 0;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }}
    else{last_path_t = ros::Time::now().toSec();}
    double compTime = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    cout << "kino path planning finished! time cost (ms)： " <<compTime<<endl;
    // vector<Eigen::Vector3d> kinotraj = kino_path_finder_->getKinoTraj(0.01);
    // vector<Eigen::Vector3d> waypoints, start_end_derivatives;
    if (status != KinodynamicAstar::NO_PATH){
    kino_path_finder_->getSamples(0.3, waypoints,start_end_derivatives);}
    }
    else {cout << "[kino replan]: Old kino path is safe" << endl;}}
    cout << "Goal:\n" << goal << endl <<g_goal<<endl;
    cout << "the updated waypoints:" << waypoints[0] << endl << waypoints.back()<<endl;
    MatrixXd waypoints_m = Map<MatrixXd>(waypoints[0].data(),3,waypoints.size());
    bool if_safe = reference.check_polyH_safe(traj_last_t, waypoints_m,ct_pos, flying.obs_pointer, flying.dynobs_pointer,ros::Time::now().toSec());
    
    // cout << "corridor update, check safety result:" << if_safe <<endl;
    cout << "point cloud update, check safety result:" << if_safe <<endl;
    if (if_safe && (timee < sfck_t))
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
    reference.replan_traj(MaxVel,ct_pos,ct_vel,ct_acc,waypoints_m,flying.cd_c,flying.cd_r,flying.obs_pointer, flying.dynobs_pointer,ros::Time::now().toSec());
    traj_last_t =ros::Time::now().toSec();
    timee = 1/CtrlFreq;
    cout << "old traj is not safe, get new traj!" << endl;
    
    }
     cout << "traj time cost (ms): " << (ros::Time::now().nsec-t1)*1e-6 <<endl;
    //  <<ros::Time::now().toSec()<<endl<<t1<<endl<<traj_last_t<<endl<<flying.dynobs_pointer->time_stamp<<endl;  
       }
      // cout << "timee:"<<timee<<endl;
        reference.get_desire(timee, p_d, v_d, a_d,p_d_yaw);
        if_initial = false;
    //    bsc.controller(state, p_d, v_d, a_d,p_d_yaw,next_goal);
        // cout<<"55"<<endl;
        // step forward
        Vector2d v2 = (p_d_yaw - state.P_E).head(2);
        Vector2d v1;
        v1<<1.0,0.0;
        double desire_psi=acos(v1.dot(v2) /(v1.norm()*v2.norm())); 
        
        if (v2(1)<0)
        {desire_psi = -desire_psi;}
        // desire_psi = state.Euler(2) + clip(desire_psi - state.Euler(2),-0.9,0.9);
        state = flying.step(desire_psi, p_d, v_d, a_d, "pos_vel_acc_yaw_c"); 
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
        flying.pub_path (waypoints);
        flying.pub_polyh (reference.decompPolys);
    
    ros::Duration(1/CtrlFreq).sleep();
    
    }

  
    return 0;}


