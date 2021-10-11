/*
This code includes all communication classes to MAVROS.
*/
#pragma once
#include <call_states/ros_communicate.h>

void Listener::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
    flight_state = *msg;
}
void Listener::estateCb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    flight_estate = *msg;
}

void Listener::posCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose = *msg;
    // position
    P_E(0) = pose.pose.position.x;
    P_E(1) = pose.pose.position.y;
    P_E(2) = pose.pose.position.z;
//     cout<<P_E<<endl;
    // orientation
    Quat.x() = pose.pose.orientation.x;
    Quat.y() = pose.pose.orientation.y;
    Quat.z() = pose.pose.orientation.z;
    Quat.w() = pose.pose.orientation.w;
    Quat = Quat.normalized();
    Rota = Quaternion2Rota(Quat);    //from body to earth
}

void Listener::wptsCb(const nav_msgs::Path::ConstPtr& msg)
{

 nav_msgs::Path wpts = *msg;
 int len = wpts.poses.size();
 //MatrixXd waypoints_ins(len,3);
 waypoints.resize(len,3);
 for (int i =0;i<len;i++)
 {
 waypoints.row(i) << wpts.poses[i].pose.position.x, wpts.poses[i].pose.position.y, wpts.poses[i].pose.position.z;
 }
 cout<< "wpts callback:" << waypoints << endl << len <<endl;
// waypoints = waypoints_ins;
}
// void Listener::posCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     geometry_msgs::PoseStamped pose = *msg;
//     // position
//     P_E(0) = pose.pose.position.x;
//     P_E(1) = pose.pose.position.y;
//     P_E(2) = pose.pose.position.z;
//     // orientation
//     Quat.x() = pose.pose.orientation.x;
//     Quat.y() = pose.pose.orientation.y;
//     Quat.z() = pose.pose.orientation.z;
//     Quat.w() = pose.pose.orientation.w;
//     Rota = Quat.normalized().toRotationMatrix();    //from body to earth
//     Rota_EB = Rota.transpose(); //from earth to body
//     //Euler = Rota.eulerAngles(0, 1, 2);  // roll pitch yaw
//     Euler = Quaternion2Euler(Quat);
// }

void Listener::velCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    geometry_msgs::TwistStamped velocity = *msg;
    // linear velocity
    V_E(0) = velocity.twist.linear.x;
    V_E(1) = velocity.twist.linear.y;
    V_E(2) = velocity.twist.linear.z;
}

// void Listener::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     sensor_msgs::Imu imu = *msg;
//     // accerleration
//     A_B(0) = imu.linear_acceleration.x;
//     A_B(1) = imu.linear_acceleration.y;
//     A_B(2) = imu.linear_acceleration.z;//-9.8066;
//     A_E = Rota * A_B;
//     A_E(2) = A_E(2) - 9.8066;
//     // angular velocity
//     Rate_B(0) = imu.angular_velocity.x;
//     Rate_B(1) = imu.angular_velocity.y;
//     Rate_B(2) = imu.angular_velocity.z;
//     Rate_E = Rota * Rate_B;
// }

void Listener::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu imu = *msg;
    // accerleration
    A_B(0) = imu.linear_acceleration.x;
    A_B(1) = imu.linear_acceleration.y;
    A_B(2) = imu.linear_acceleration.z;     //  -9.8066;
    A_E = Rota * A_B;
    A_E(2) = A_E(2) - 9.8066;
    // angular velocity
    Rate_B(0) = imu.angular_velocity.x;
    Rate_B(1) = imu.angular_velocity.y;
    Rate_B(2) = imu.angular_velocity.z;
}

void Listener::crdCb(const visualization_msgs::MarkerArray::ConstPtr& msg)
{ visualization_msgs::MarkerArray corridor = *msg;
  
  corridor_update = true;
  cout<< "corridor updated!" <<endl;
  int len = corridor.markers.size();
//   len = (len-2)/5+2;
//   len = ball.pose.orientation.w=1.0
  cout<< "corridor updated!" << len <<endl;
  int i = 0;
   while (i < len)
  { visualization_msgs::Marker ball =  corridor.markers[i];
    
    if (ball.color.b < 0.9)
    {cout << "keep cords num:"<< i << endl;
    break;}
    i++;}
//   for (int i =0; i<len;i++)
   cd_c.resize(i,3);
   cd_r.resize(i);
 int j = 0;
   while (j < i)
  { visualization_msgs::Marker ball =  corridor.markers[j];
    
    cd_c.row(j) << ball.pose.position.x, ball.pose.position.y, ball.pose.position.z;
    cd_r(j) = ball.scale.x/2;
    j++;
    
  }
 // cd_c = cd_c_ins;
 // cd_r = cd_r_ins;
}

/*
FCU Modes Requests
*/
/*
class fcuModes
{
    private:
        // ros
        ros::NodeHandle nh_;
        ros::ServiceClient arming_client_, land_client_, set_mode_client_;
    
    public:
        // init clients
        fcuModes(ros::NodeHandle *nodehandle);

        // fcu modes
        bool setArm_(void);
        bool setLand_(void);
        bool setMode_Offboard_(void);
};

fcuModes::fcuModes(ros::NodeHandle* nodehandle):nh_(*nodehandle) 
{   
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

bool fcuModes::setArm_()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}

bool fcuModes::setLand_()
{
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;

    if (land_client_.call(land_cmd) && land_cmd.response.success)
    {
        ROS_INFO("Vehicle landed");
        return true;
    }
    return false;
}

bool fcuModes::setMode_Offboard_()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
                ROS_INFO("Offboard enabled");
        return true;
    }
    return false;
}
*/