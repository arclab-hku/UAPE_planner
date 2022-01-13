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

#define CtrlFreq 100
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
    Matrix<double, 3, 5> camera_vertex,camera_vertex_b;
    double d2r = 3.14159265/180;
    double cam_depth = 10.0;
    double h_fov = 87; // in degree
    double v_fov = 58;
    camera_vertex_b.col(0) << 0,0,0;
    camera_vertex_b.col(1) << cam_depth,tan(h_fov/2*d2r)*cam_depth,tan(v_fov/2*d2r)*cam_depth;
    camera_vertex_b.col(2) << cam_depth,-tan(h_fov/2*d2r)*cam_depth,tan(v_fov/2*d2r)*cam_depth;
    camera_vertex_b.col(3) << cam_depth,-tan(h_fov/2*d2r)*cam_depth,-tan(v_fov/2*d2r)*cam_depth;
    camera_vertex_b.col(4) << cam_depth,tan(h_fov/2*d2r)*cam_depth,-tan(v_fov/2*d2r)*cam_depth;
    double dis_goal,dis_goal_ini;
    double sfck_t;
    bool ifMove;
    nh.getParam("goal", goalp);
    nh.getParam("search/horizon", dis_goal_ini);
    nh.getParam("sfck_t", sfck_t);
    nh.getParam("ifMove", ifMove);
    nh.getParam("cam_depth", cam_depth);
    nh.getParam("h_fov", h_fov);
    nh.getParam("v_fov", v_fov);
    dis_goal = dis_goal_ini-0.5;
    Eigen::Vector3d g_goal = {goalp[0],goalp[1],goalp[2]};
    Eigen::Vector3d goal;
    bool if_initial = true;
    double ball_pass_time = 1.5;
    double min_dist2dynobs = 1e6;
    double tmp_dist,t_gap_ball;
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
    // flying.dynobs_pointer->ball_number = 0;
    while (!flying.pcl_update)
    { ros::Duration(0.05).sleep();
      state = flying.get_state();
    }
    cout << "Point cloud received!\n" << endl;
    state = flying.get_state();
    ct_pos = state.P_E;
    ct_vel = state.V_E;
    ct_acc = state.A_E;
    camera_vertex = (state.Rota*camera_vertex_b).array().colwise() + ct_pos.array();
    // cout << "reset env!\n" << endl;
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    // cout << "set params !\n" << flying.obs_pointer << endl;
    kino_path_finder_->setEnvironment(flying.obs_pointer,flying.dynobs_pointer,camera_vertex);
    // cout << "set obs !\n" << endl;
    kino_path_finder_->init();
    
    if ((g_goal-ct_pos).norm()>dis_goal)
    {goal = ct_pos + (g_goal-ct_pos)/(g_goal-ct_pos).norm()*dis_goal;}
    else{goal = g_goal;}
    kino_path_finder_->reset();
    cout << "begin search!\n" << goal << endl;
    int status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, true);
    last_path_t = ros::Time::now().toSec();
    while (status == KinodynamicAstar::GOAL_OCC)
    {
    dis_goal -= 0.5;
    goal = ct_pos + (g_goal-ct_pos)/(g_goal-ct_pos).norm()*dis_goal;
    kino_path_finder_->reset();
    status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, false);}
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
     dis_goal = dis_goal_ini-0.5;
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

    state = flying.get_state();
    flying.set_cod_update(false);
    flying.set_pcl_update(false);
    if (flying.dynobs_pointer->ball_number>0 && ros::Time::now().toSec()-flying.dynobs_pointer->ball_time_stamp > ball_pass_time) // for bag sim// flying.dynobs_pointer->ball_number>0 && (flying.dynobs_pointer->ballvel[0](0) > -0.2)|| 
    // { cout<<"dyn ball from backside: "<<flying.dynobs_pointer->ballvel[0]<<endl;
    {cout<<"dyn ball time out: "<<flying.dynobs_pointer->ballvel[0]<<endl;
      flying.dynobs_pointer->ball_number = 0;
      // double p_gap = flying.dynobs_pointer->ballpos[0](0) - ct_pos(0);
      // double ball_pass_time1 = (-flying.dynobs_pointer->ballvel[0](0) + sqrt(pow(flying.dynobs_pointer->ballvel[0](0),2)-2*flying.dynobs_pointer->ballacc[0](0)*p_gap))/(2*p_gap);
      // double ball_pass_time2 = (-flying.dynobs_pointer->ballvel[0](0) - sqrt(pow(flying.dynobs_pointer->ballvel[0](0),2)-2*flying.dynobs_pointer->ballacc[0](0)*p_gap))/(2*p_gap);
      // if (ball_pass_time1>0)
      // {ball_pass_time = ball_pass_time1;}
      // else if (ball_pass_time2>0)
      // {ball_pass_time = ball_pass_time2;}
      // else{ball_pass_time = 2;}
      // cout<<"ball_pass_time: "<<ball_pass_time<<endl;
    }
    for (int bi = 0; bi < flying.dynobs_pointer->dyn_number; bi++)
     
    { t_gap_ball = ros::Time::now().toSec() - flying.dynobs_pointer->time_stamp;
      tmp_dist = (state.P_E-flying.dynobs_pointer->centers[bi]-t_gap_ball*flying.dynobs_pointer->vels[bi]).norm();
      if (tmp_dist < min_dist2dynobs)
      {min_dist2dynobs = tmp_dist;
      cout<<"min distance from objects to drone:"<<min_dist2dynobs<<endl<<state.P_E<<endl<<flying.dynobs_pointer->centers[bi]+t_gap_ball*flying.dynobs_pointer->vels[bi];}
      
    }
    for (int di = 0; di < flying.dynobs_pointer->ball_number && flying.dynobs_pointer->ballvel[0](0) < -0.4; di++)
    { t_gap_ball = ros::Time::now().toSec() - flying.dynobs_pointer->ball_time_stamp;
      tmp_dist = (state.P_E-(flying.dynobs_pointer->ballpos[di] + t_gap_ball*flying.dynobs_pointer->ballvel[di] + 0.5*t_gap_ball*t_gap_ball*flying.dynobs_pointer->ballacc[di])).norm();
      if (tmp_dist < min_dist2dynobs)
      {min_dist2dynobs = tmp_dist;
      cout<<"min distance from ball to drone:"<<min_dist2dynobs<<endl;}
    }
    if (if_initial || !ifMove)
    {
    ct_pos = state.P_E;
    ct_vel = state.V_E;
    ct_acc = state.A_E;}
    else{
    ct_pos = p_d;
    ct_vel = v_d;
    ct_acc = a_d;}
    if_initial = false;
    camera_vertex = (state.Rota*camera_vertex_b).array().colwise() + state.P_E.array();
    // ct_acc = Vector3d::Zero(3);
    //ros::Time::now().toSec() - last_path_t > 0.1 && 
    if ((goal-ct_pos).norm()>1.0 && flying.obs_pointer->size()>0)
    {
    chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();
    // cout << "The obs pointer:\n" << flying.obs_pointer << "---pcl size: "<< flying.obs_pointer->size()<< endl;
    
    kino_path_finder_->setEnvironment(flying.obs_pointer,flying.dynobs_pointer,camera_vertex);
    if (!kino_path_finder_->checkOldPath(waypoints) || ros::Time::now().toSec() - last_path_t > sfck_t)
    {
    kino_path_finder_->reset();
    status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, true);
    while (status == KinodynamicAstar::GOAL_OCC)
    {
    dis_goal -= 0.5;
    goal = ct_pos + (g_goal-ct_pos)/(g_goal-ct_pos).norm()*dis_goal;
    kino_path_finder_->reset();
    status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, false);}
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
    else {cout << "[kino replan]: Old kino path is safe" << endl;}
    }
    else
    {waypoints.clear();
     waypoints.emplace_back(ct_pos);
     waypoints.emplace_back(goal);}
    // cout << "Goal:\n" << goal << endl <<g_goal<<endl;
    // cout << "the updated waypoints:" << waypoints[0] << endl << waypoints.back()<<endl;
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
      double desire_psi;
        if ((goal-ct_pos).norm()>0.2)
        {
        reference.get_desire(timee, p_d, v_d, a_d,p_d_yaw);
        Vector2d v2 = (p_d_yaw - state.P_E).head(2);
        Vector2d v1;
        v1<<1.0,0.0;
        desire_psi=acos(v1.dot(v2) /(v1.norm()*v2.norm())); 
        
        if (v2(1)<0)
        {desire_psi = -desire_psi;}}
        else{
          p_d=goal;
          v_d.setZero();
          a_d.setZero();
          if_initial = true;
        }
        
    //    bsc.controller(state, p_d, v_d, a_d,p_d_yaw,next_goal);
        // cout<<"55"<<endl;
        // step forward

        // desire_psi = state.Euler(2) + clip(desire_psi - state.Euler(2),-0.9,0.9);
        if (ifMove)
        {
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
        reference.get_traj_samples(sp_pos, sp_vel,sp_acc, ros::Time::now().toSec() - traj_last_t);}
        else{reference.get_traj_samples(sp_pos, sp_vel,sp_acc, 0.0);}
        flying.pub_traj (sp_pos, sp_vel,sp_acc);
        flying.pub_path (waypoints);
        flying.pub_polyh (reference.decompPolys);
        flying.pub_fovshape (camera_vertex);
        if (flying.dynobs_pointer->ball_number >0)
        {
         flying.pub_ballstates();
        }
    ros::Duration(1/CtrlFreq).sleep();
    if (if_initial)
    {break;}

    }
    
  
    return 0;}


