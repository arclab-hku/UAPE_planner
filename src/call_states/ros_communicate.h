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
using namespace std;
using namespace Eigen;

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
        bool corridor_update = false;
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
};
