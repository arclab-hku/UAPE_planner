#pragma once

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <mavros_msgs/State.h>//subs
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>//pubs
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h> //services
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <call_states/ros_communicate.h>
#include <Tools.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
using namespace std;
using namespace Eigen;


/*
// Basic functions to mavros
*/
class RosClass
{
    private:
        // ros
        ros::NodeHandle nh_;
        ros::Subscriber state_sub_, exstate_sub_, pos_sub_, vel_sub_, imu_sub_,corridor_sub_,wpts_sub_;
        ros::Publisher pos_pub_, raw_pub_, actuCtrl_pub_,traj_pub_,detail_traj_pub_;
        ros::ServiceClient arming_client_, land_client_, set_mode_client_;
        ros::Rate rate;

        Listener listener_;  // listen drone states from mavros, handle the raw data.

        // drone parameters
        double mass_;
        Matrix3d Inertia_;

        // drone states
        States state;

        // control parameters
        int Freq_;
        double h_, k_thr_, k_tor_xy_, k_tor_z_, gravity = 9.8066;

        States get_state_();
        void crashed_();

        // fcu modes
        bool setArm_(void);
        bool setLand_(void);
        bool setMode_Offboard_(void);

    public:
        // waypoints
        Vector3d Start, End;
        
        bool done;
        Matrix3d RCtrl;
        MatrixXd waypoints;
        MatrixXd cd_c;
        VectorXd cd_r;
        bool corridor_update = false;
        double Yaw;
        // init ros node
        RosClass(ros::NodeHandle *nodehandle, int FREQ);
        States get_state();
        // init variables
        void init(
            double HGT = 2.0,
            double yaw = 0,
            double THR_C = 0.4025, //the Normalized thrust when the drone is hovering. <motorConstant>1.5e-05</motorConstant>  for 0.4025 ;/<motorConstant>2.5e-05</motorConstant> for 0.2896685.
            double TOR_C = 0.06, //<momentConstant>0.06</momentConstant>
            double MASS = 1.5,
            double Ix = 0.029125,
            double Iy = 0.029125,
            double Iz = 0.055225);
        Vector3d get_position();
        void set_cod_update(bool cod_update);
        //basic missions
        States launch(void);
        States step(double double_n, Vector3d pos,Vector3d vel,Vector3d acc, string mode);
        void pub_traj(MatrixXd pos, MatrixXd vel, MatrixXd acc);
        void land(Vector3d endp);
};


