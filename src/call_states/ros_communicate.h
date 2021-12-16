#pragma once

#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>

#include <Tools.h>

#include <ros/ros.h>
#include <mavros_msgs/State.h> //subs 
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h> //services
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include<traj_opt/se3_planner.h>
using namespace std;
using namespace Eigen;

       typedef struct dynobs_tmp
        {
         vector<Vector3d> centers;
         vector<Vector3d> obs_sizes;
         vector<Vector3d> vels;
         double time_stamp;
         int dyn_number;
         vector<Vector3d> ballpos;
         vector<Vector3d> ballvel;
         vector<Vector3d> ballacc;
         vector<Vector3d> ball_sizes;
         double ball_time_stamp;
         int ball_number;
        }; 

class Listener
{
    private:
        Vector3d A_B;

    public:
        // mavros states
        mavros_msgs::State flight_state;
        mavros_msgs::ExtendedState flight_estate;
        
        // local position
        //// linear states
        Vector3d P_E, V_E;
        //// angular states
        Quaterniond Quat;
        Matrix3d Rota;
        // imu
        Vector3d A_E, Rate_B;
        MatrixXd cd_c;
        VectorXd cd_r;
        MatrixXd waypoints;
        vec_Vec3f obs;
        dynobs_tmp dynobs;
        
        bool pcl_update = false;
        bool waypoint_update = false;
        bool trigger = false;
        sensor_msgs::PointCloud cloud;
        // // control states
        // //// linear states
        // Vector3d P_E, V_E, A_E, A_B;
        // //// angular states
        // Quaterniond Quat;
        // Matrix3d Rota, Rota_EB;
        // Vector3d Euler, Rate_E, Rate_B;

        void stateCb(const mavros_msgs::State::ConstPtr &);
        void estateCb(const mavros_msgs::ExtendedState::ConstPtr &);
        void posCb(const geometry_msgs::PoseStamped::ConstPtr &);
        void velCb(const geometry_msgs::TwistStamped::ConstPtr &);
        void imuCb(const sensor_msgs::Imu::ConstPtr &);
        void crdCb(const visualization_msgs::MarkerArray::ConstPtr &);
        void wptsCb(const nav_msgs::Path::ConstPtr &);
        void obsCb(const sensor_msgs::PointCloud2::ConstPtr &);
        void odomCb(const nav_msgs::Odometry::ConstPtr & msg);
        void triggerCb(const geometry_msgs::PoseStamped::ConstPtr & msg);
        void ballCb(const ??::??::ConstPtr & msg);

};
