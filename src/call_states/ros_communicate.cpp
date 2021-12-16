/*
This code includes all communication classes to MAVROS.
*/
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
void Listener::triggerCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    trigger = true;
}
void Listener::wptsCb(const nav_msgs::Path::ConstPtr& msg)
{
 waypoint_update = true;
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

void Listener::odomCb(const nav_msgs::Odometry::ConstPtr & msg)
    {
        V_E(0) = msg->twist.twist.linear.x;
        V_E(1) = msg->twist.twist.linear.y;
        V_E(2) = msg->twist.twist.linear.z;

            // position
        P_E(0) = msg->pose.pose.position.x;
        P_E(1) = msg->pose.pose.position.y;
        P_E(2) = msg->pose.pose.position.z;
        // cout<<P_E<<endl;
        // orientation
        Quat.x() = msg->pose.pose.orientation.x;
        Quat.y() = msg->pose.pose.orientation.y;
        Quat.z() = msg->pose.pose.orientation.z;
        Quat.w() = msg->pose.pose.orientation.w;
        Quat = Quat.normalized();
        Rota = Quaternion2Rota(Quat);    //from body to earth
    }
void Listener::obsCb(const sensor_msgs::PointCloud2::ConstPtr & msg)
{   sensor_msgs::PointCloud cloud;
//    {cout<<"convert!"<<endl;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg,cloud);
    pcl_update = true;
    obs.clear();
    // obs.resize(cloud.points.size());
    dynobs.time_stamp = cloud.header.stamp.sec + cloud.header.stamp.nsec * 1e-9;
    dynobs.dyn_number = cloud.points.back().x;
    dynobs.centers.clear();
    dynobs.vels.clear();
    dynobs.obs_sizes.clear();
    dynobs.centers.resize(dynobs.dyn_number);
    dynobs.vels.resize(dynobs.dyn_number);
    dynobs.obs_sizes.resize(dynobs.dyn_number);
    obs.resize(cloud.points.size()-dynobs.dyn_number*3-1);
    // cout << "point cloud received, dynamic number: " <<  cloud.points.back().x << "pcl size:" <<cloud.points.size()<<endl;
    for (size_t i = 0; i < cloud.points.size()-1; i++)
    {
    if (i < obs.size())
    {
    obs[i](0) = cloud.points[i].x;
    obs[i](1) = cloud.points[i].y;
    obs[i](2) = cloud.points[i].z;
    }
    else if (i < obs.size() + dynobs.dyn_number)
{   dynobs.centers[i-obs.size()](0) = cloud.points[i].x;
    dynobs.centers[i-obs.size()](1) = cloud.points[i].y;
    dynobs.centers[i-obs.size()](2) = cloud.points[i].z;}
    else if (i < obs.size() + 2*dynobs.dyn_number)
{   dynobs.vels[i-obs.size()-dynobs.dyn_number](0) = cloud.points[i].x;
    dynobs.vels[i-obs.size()-dynobs.dyn_number](1) = cloud.points[i].y;
    dynobs.vels[i-obs.size()-dynobs.dyn_number](2) = cloud.points[i].z;}
    else
{   dynobs.obs_sizes[i-obs.size()-2*dynobs.dyn_number](0) = cloud.points[i].x;
    dynobs.obs_sizes[i-obs.size()-2*dynobs.dyn_number](1) = cloud.points[i].y;
    dynobs.obs_sizes[i-obs.size()-2*dynobs.dyn_number](2) = cloud.points[i].z;}

    }
  
}
void ballCb(const ??::??::ConstPtr & msg)
{
    dynobs.ball_number = msg.states.size();
    dynobs.ballpos.clear();
    dynobs.ballpos.resize(dynobs.ball_number);
    dynobs.ballvel.clear();
    dynobs.ballvel.resize(dynobs.ball_number);
    dynobs.ballacc.clear();
    dynobs.ballacc.resize(dynobs.ball_number);
    dynobs.ball_sizes.clear();
    dynobs.ball_sizes.resize(dynobs.ball_number);
    dynobs.ball_time_stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9;
    for (size_t i = 0; i < dynobs.ball_number; i++)
    {
    dynobs.ballpos[i](0) = msg.states[i].position.x;
    dynobs.ballpos[i](1) = msg.states[i].position.y;
    dynobs.ballpos[i](2) = msg.states[i].position.z;
    dynobs.ballvel[i](0) = msg.states[i].velocity.x;
    dynobs.ballvel[i](1) = msg.states[i].velocity.y;
    dynobs.ballvel[i](2) = msg.states[i].velocity.z;
    dynobs.ballacc[i](0) = msg.states[i].acceleration.x;
    dynobs.ballacc[i](1) = msg.states[i].acceleration.y;
    dynobs.ballacc[i](2) = msg.states[i].acceleration.z;
    dynobs.ball_sizes[i](0) = 0.4;
    dynobs.ball_sizes[i](1) = 0.4;
    dynobs.ball_sizes[i](2) = 0.4;
    }
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