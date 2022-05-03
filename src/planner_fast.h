#pragma once

// #include "am_traj.hpp"
//#include <planners/wayp_reader.h>
#include<traj_opt/se3_planner.h>
#include<traj_opt/se3gcopter_cpu.hpp>
#include <call_states/ros_communicate.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Tools.h>
using namespace std;
using namespace Eigen;
//using namespace min_jerk;
struct Config
{


    string odomFrame;

    // Params
    double scaleSI;
    // double mapHeight;
    Vector3d polyhedronBox,global_min,global_size;
    double rho;
    double totalT;
    int qdIntervals;
    double horizHalfLen;
    double vertHalfLen;
    double safeMargin;
    double velMax;
    double thrustAccMin;
    double thrustAccMax;
    double bodyRateMax;
    double gravAcc;
    Vector4d penaltyPVTB;
    bool useC2Diffeo;
    bool if_debug;
    double optRelTol;
    double trajVizWidth;
    Vector3d trajVizRGB;
    string routeStoragePath;
    string ellipsoidPath;
    Vector4d ellipsoidVizRGBA;
    double max_yaw_range = 1.57;
    double yaw_w = 0.4;
    double yaw_gap_max = 0.55;
    double horizon = 8.0;
    inline void loadParameters(const ros::NodeHandle &nh_priv)
    // {       cout << "mk3" << endl;
        {vector<double> vecPolyhedronBox, vecPenaltyPVTB, vecTrajVizRGB, vecEllipsoidVizRGBA,vecGlobal_min,vecGlobal_size;;
       // string packagePath = ros::package::getPath("ahpf_planner");
       // packagePath += packagePath.back() == '/' ? "" : "/";
       // string packageUrl = "package://plan_manage/";
     //   nh_priv.getParam("OdomFrame", odomFrame);
     nh_priv.getParam("MaxYawRange", max_yaw_range);
     nh_priv.getParam("YawGapWeight", yaw_w);
     nh_priv.getParam("YawGapMax", yaw_gap_max);

        nh_priv.getParam("ScaleSI", scaleSI);
        // cout << nh_priv.getParam("ScaleSI", scaleSI) << endl;
        // nh_priv.getParam("MapHeight", mapHeight);
        nh_priv.getParam("PolyhedronBox", vecPolyhedronBox);
        // cout << "mk4" << endl<< scaleSI << endl << vecPolyhedronBox[2] << endl << vecPolyhedronBox[0] <<endl;
        polyhedronBox << vecPolyhedronBox[0], vecPolyhedronBox[1], vecPolyhedronBox[2];
        nh_priv.getParam("GlobalBox_min", vecGlobal_min);
        nh_priv.getParam("GlobalBox_size", vecGlobal_size);
        global_min << vecGlobal_min[0], vecGlobal_min[1], vecGlobal_min[2];
        global_size <<vecGlobal_size[0], vecGlobal_size[1], vecGlobal_size[2];
        nh_priv.getParam("Rho", rho);
        nh_priv.getParam("TotalT", totalT);

        nh_priv.getParam("QdIntervals", qdIntervals);
        nh_priv.getParam("HorizHalfLen", horizHalfLen);
        
        nh_priv.getParam("VertHalfLen", vertHalfLen);
        nh_priv.getParam("SafeMargin", safeMargin);
        nh_priv.getParam("VelMax", velMax);
        
        nh_priv.getParam("ThrustAccMin", thrustAccMin);
        nh_priv.getParam("ThrustAccMax", thrustAccMax);
        nh_priv.getParam("BodyRateMax", bodyRateMax);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("PenaltyPVTB", vecPenaltyPVTB);
        penaltyPVTB << vecPenaltyPVTB[0], vecPenaltyPVTB[1], vecPenaltyPVTB[2], vecPenaltyPVTB[3];
        nh_priv.getParam("UseC2Diffeo", useC2Diffeo);
        nh_priv.getParam("OptRelTol", optRelTol);
        nh_priv.getParam("TrajVizWidth", trajVizWidth);
        nh_priv.getParam("TrajVizRGB", vecTrajVizRGB);
        nh_priv.getParam("if_debug",if_debug);
        nh_priv.getParam("search/horizon",horizon);
        trajVizRGB << vecTrajVizRGB[0], vecTrajVizRGB[1], vecTrajVizRGB[2];
       // nh_priv.getParam("RouteStoragePath", routeStoragePath);
       // routeStoragePath = packagePath + routeStoragePath;
       // nh_priv.getParam("EllipsoidPath", ellipsoidPath);
       // ellipsoidPath = packageUrl + ellipsoidPath;
        nh_priv.getParam("EllipsoidVizRGBA", vecEllipsoidVizRGBA);
        ellipsoidVizRGBA << vecEllipsoidVizRGBA[0], vecEllipsoidVizRGBA[1], vecEllipsoidVizRGBA[2], vecEllipsoidVizRGBA[3];
        // cout << "mk5" << endl;
        // nh_priv.getParam("QuadrotorPath", quadrotorPath);
        // quadrotorPath = packageUrl + quadrotorPath;
    }};

/*
// Generate Trajectory by waypoints.txt
*/
class TrajectoryGenerator_fast
{
    private:
        ros::NodeHandle nh_;
        double MaxVel, MaxVelCal;
        double Acc = 5.0;
        bool if_config = false; // if the waypoints and polyhedrons are updated
        Matrix3d iS, fS;        // xyz * pva        // intial & finial state
        bool G = 9.8016;
        double delta_t_yaw = 0.5;
        // min_jerk::JerkOpt jerkOpt;              // optimizer
        // min_jerk::Trajectory minJerkTraj;       // trajectory generated by optimizer
        //JerkOpt jerkOpt;              // optimizer
      //  Trajectory minJerkTraj;       // trajectory generated by optimizer
        
       // WayPointsReader waypGen;        // read waypoints from txt

        VectorXd allocateTime(const MatrixXd &wayPs, double vel, double acc);
        void get_wPs(const MatrixXd &waypoints, const MatrixXd &cd_c, const VectorXd &cd_r,const Vector3d &start);
        bool get_new_wps(Trajectory traj, const MatrixXd &cd_c, const VectorXd &cd_r);
     //   bool check_traj_safe(const MatrixXd &cd_c, VectorXd &cd_r);
        void update_wps(const vector<Vector3d> &out_centers, const MatrixXd &cd_c, const VectorXd &cd_r);
        int get_insert_ind(const Vector3d &check_c);
        int get_insert_ind_1(const Vector3d &check_c, const MatrixXd &cd_c, const VectorXd &cd_r);
        bool safe_check(const Vector3d pos, const MatrixXd &cd_c,const  VectorXd &cd_r);
        bool dyn_safe_check(Vector3d pt, double check_t);
        void sort_vec(const VectorXd& vec, VectorXd& sorted_vec,  VectorXi& ind);
        void Traj_opt(const MatrixXd &iniState, const MatrixXd &finState, double plan_t);
        void gen_polyhedrons(vec_Vec3f *obs_pointer);
        void check_wps_in_polyH (void);
        void Yaw_plan(Matrix<double, 3, 5> camera_vertex_b,double plan_t);
        bool inFOV(Matrix<double, 3, 5> camera_vertex, Vector3d  ct_center);
    public:
//         MatrixXd waypoints;    // pva * time
        Config config;
        vec_Vec3f *obs_pointer;
        dynobs_tmp *dynobs_pointer;
        vector<MatrixXd> hPolys;
        vec_E<Polyhedron3D> decompPolys;
        Trajectory traj;
        VectorXd ts;        // time for pieces
        VectorXf bound;
        vector<Vector3d> wPs;
        double total_t;
        uint check_sfc_ind;
        vector<double> yaw_plan;
        vector<double> yaw_plan_t;
        Vector3d last_check_pos;
        ros::Time last_check_time;
        bool check_traj_safe(const MatrixXd &cd_c, const VectorXd &cd_r, const double start_t);
        void replan_traj(Vector3d &start,Vector3d &vi,Vector3d &ai,MatrixXd &waypoints,vector<Vector3d> &start_end_divs ,vec_Vec3f *obs_pointer, dynobs_tmp *dynobs, double plan_t, Matrix<double, 3, 5> camera_vertex, bool full_trip = true,bool if_reach = true);
        bool check_polyH_safe (const double start_t, const MatrixXd &waypoints,  Vector3d &start, vec_Vec3f *obs_pointer, dynobs_tmp *dynobs, double plan_t);
        void read_param (const ros::NodeHandle *nh_priv);
        double generate();
        int N;
        void get_desire(double timee, Vector3d &p_d, Vector3d &v_d, Vector3d &a_d, Vector3d &p_d_yaw);
        void get_traj_samples(MatrixXd &sp_pos, MatrixXd &sp_vel, MatrixXd &sp_acc, double start_t);
        Vector2d  getYaw (double t);      
        bool last_jointPolyH_check(Vector3d ct_pos);
};