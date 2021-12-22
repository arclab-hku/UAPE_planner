import rospy

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid,Path
from sensor_msgs.msg import Imu,PointCloud2,PointField
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped,AccelStamped,PointStamped
from geometry_msgs.msg import Twist, Vector3Stamped, Pose, Point, Quaternion, Vector3,Accel
from sensor_msgs import point_cloud2
from visualization_msgs.msg import MarkerArray, Marker
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Header
from est_local_pos import Estimator_local_position
from est_local_acc import Estimator_local_acc
from get_control import control_method
import numpy as np
import tf
from pymavlink import mavutil
from mavros_test_common import MavrosTestCommon
from pid import PID
from tf.transformations import quaternion_from_euler
import math
import time
import getpass
from message_filters import Subscriber,ApproximateTimeSynchronizer
class StateMachineError(Exception):
    pass


class UAVController(object):
    def register(self):
        self.startpos=[0,0,0]
        self.f1 = open("/home/" + getpass.getuser() +"/catkin_ws/src/AHPF_planner/data/"+time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()))+"OT.txt", 'a')
        self.f2 = open("/home/" + getpass.getuser() +"/catkin_ws/src/AHPF_planner/data/"+time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()))+"PV.txt", 'a')
        # self.f.write('number of points   '+'find wp iters   '+'find wp time   '+'optimization time   '+'step  time  '+'position*3  '+'velocity*3  '+'attitude*3'+'\n')
#        self.path=[]
        self.steptime=[]
        self.velo_goal=[]
        self.pid=PID(4,0.1,5,-0.7,0.7)
        self.g=9.8 #gravity
        # self.goal=np.array([-11,-11,1])
        self.goal=None
        self.ifend=0
        self.tick = 0
        self.loc_goal_old= np.array([0,0,1.0])
        self.old_goal=[]
        self.pretime=0
        self.f_angle=[float("Inf"),float("Inf")]
        self.path_rec=[]
        self.c_dyn1,self.v_dyn1=[],[]
        self.dyn_time=0
        self.ros_data = {}
        self.vel=None
        self.angular_velocity_estimator=Estimator_local_position()
        self.accelerator_estimator=Estimator_local_acc()
        self.ros_data["plc"]=None
        self.ros_data["plcall"]=None
        self.if_direct = True
        self.no_path = 1
        self.pp_restart = 0
        self.global_goal = None
        self.d3_check_r = 0
        self.d3_check = None
        self.d3_time = 0
        self.d3_pos = 0
        self.glb_goal_3d = []
        self.all_edge_goal=[]
        self.if_d3_goal = 0
        self.control = None
        self.wpts = None
        self.pred_trj = []
        self.if_align = 0
        self.pcl_w = []
        rospy.init_node('UAV_controller')
        rospy.loginfo("UAV_controller started!")
        self.goal_sub = rospy.Subscriber('/goal_global',
                                              Point,
                                              self.goal_callback)
#        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
#                                              PoseStamped,
#                                              self.local_position_callback)
        self.local_imu_sub = rospy.Subscriber('mavros/imu/data',
                                              Imu,
                                              self.Imu_callback)
#        self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity',
#                                              TwistStamped,
#                                              self.velocity_callback)
#        self.local_vel_sub1 = rospy.Subscriber('mavros/local_position/velocity_local',
#                                       TwistStamped,
#                                       self.velocity_callback)
        self.state_sub = rospy.Subscriber('mavros/state',
                                          State,
                                          self.state_callback)
        self.plc_sub = rospy.Subscriber('/octomap_point_cloud_centers_local',#/filtered_RadiusOutlierRemoval',
                                          PointCloud2,
                                          self.plc_callback,queue_size=1)
        self.plcall_sub = rospy.Subscriber('/points_global_all',#/filtered_RadiusOutlierRemoval',
                                          PointCloud2,
                                          self.plcall_callback,queue_size=1)
        self.ahpf_path_pub = rospy.Publisher('/ahpf_path', Path, queue_size=1)
        self.predtrj_pub = rospy.Publisher('/pred_path', Path, queue_size=1)
        self.set_arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_landing = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pos_setvel_pub = rospy.Publisher(
            'mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pos_setacc_pub = rospy.Publisher(
            'mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10) #set acceleration command
        self.target_odom_pub = rospy.Publisher(
            "target_odom", Odometry, queue_size=50)
        self.setpoint_odom_pub = rospy.Publisher(
            "setpoint_odom", Odometry, queue_size=50)
        self.pclw_publisher = rospy.Publisher(
            "obs_sphere", MarkerArray, queue_size=10)
        self.UAVBB_publisher = rospy.Publisher(
            "UAVBB_marker", Marker, queue_size=10)
        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10) 
        self.pcl_use_pub = rospy.Publisher('/points_global_use', PointCloud2, queue_size=1)
        
        self.path_3d_pub = rospy.Publisher('path_3d', Path, queue_size=1)
        self.all_edge_goal_pub = rospy.Publisher('all_edge_goal', MarkerArray, queue_size=1)
        self.tss = ApproximateTimeSynchronizer([
                                                # Subscriber('/gt_iris_base_link_imu', Odometry),
                                                Subscriber('mavros/local_position/pose', PoseStamped),
						# Subscriber('mavros/vision_pose/pose', PoseStamped),
                                                Subscriber('mavros/local_position/velocity_local',TwistStamped),
                                                Subscriber('/points_global_all',PointCloud2)],
        7,0.06, allow_headerless=True)
        self.tss.registerCallback(self.pos_vel)
        self.globalgoal_sub = rospy.Subscriber('/move_base_simple/goal',
                                    PoseStamped,
                                    self.global_goal_callback,queue_size=1)
    def pub_obs(self,pclw):
        if len(pclw[0]):
            pcl_w = np.array(pclw[0])+self.local_pos
        obs_size = pclw[1]
        obs_ss = MarkerArray()
        for i in range(len(obs_size)):
            pub_ball = Marker()
            pub_ball.header.frame_id = "map"
            pub_ball.header.stamp = rospy.Time.now()
            pub_ball.type=pub_ball.SPHERE
            pub_ball.color.a = 0.6
            pub_ball.color.b = 1.0
            pub_ball.scale.x = obs_size[i]
            pub_ball.scale.y = obs_size[i]
            pub_ball.scale.z = obs_size[i]
            pub_ball.pose.position.x = pcl_w[i][0]
            pub_ball.pose.position.y = pcl_w[i][1]
            pub_ball.pose.position.z = pcl_w[i][2]
            pub_ball.id = i
            obs_ss.markers.append(pub_ball)
        self.pclw_publisher.publish(obs_ss)
    def pub_3d_path(self, data):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]
            path.poses.append(pose)
        self.path_3d_pub.publish(path)
        
    def publish_predtrj(self, data):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)
        self.predtrj_pub.publish(path)
        
    def publish_apfpath(self, data):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        data = np.r_[np.array([self.local_pos]),data]
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)
        self.ahpf_path_pub.publish(path)       
    def pub_all_edge_goal(self):
        pub_edge_goalAl=MarkerArray()
        pub_edge=Marker()
        pub_goal=Marker()
        pub_goalo=Marker()
        pub_edge.header.frame_id = "map"
        pub_edge.header.stamp = rospy.Time.now()
        pub_edge.type=pub_edge.LINE_LIST
        pub_edge.color.a = 1.0
        pub_edge.color.b = 1.0
        pub_edge.scale.x = 0.03
        for egi in self.all_edge_goal[0:-1]:
            p1=Point()
            p2=Point()
            p1.x,p1.y,p1.z = self.local_pos
            p2.x,p2.y,p2.z = self.local_pos+egi
            pub_edge.points.append(p1)
            pub_edge.points.append(p2)
        
        pub_goal.header.frame_id = "map"
        pub_goal.header.stamp = rospy.Time.now()
        pub_goal.type=pub_goal.LINE_LIST
        pub_goal.color.a = 0.8
        pub_goal.color.r = 1.0
        pub_goal.color.g = 1.0
        pub_goal.scale.x = 0.1      
        p3=Point()
        p3.x,p3.y,p3.z=self.all_edge_goal[-2]+self.local_pos
        pub_goal.points.append(p1)
        pub_goal.points.append(p3)
        
        pub_goalo.header.frame_id = "map"
        pub_goalo.header.stamp = rospy.Time.now()
        pub_goalo.type=pub_goalo.LINE_LIST
        pub_goalo.color.a = 0.8
        pub_goalo.color.r = 1.0
        pub_goalo.color.g = 0.5
        pub_goalo.scale.x = 0.1      
        p4=Point()
        p4.x,p4.y,p4.z=self.all_edge_goal[-1]+self.local_pos
        pub_goalo.points.append(p1)
        pub_goalo.points.append(p4)
        
        pub_edge.id=1
        pub_goal.id=2
        pub_goalo.id=3
        pub_edge.pose.orientation.w = 1.0
        pub_goal.pose.orientation.w = 1.0
        pub_goalo.pose.orientation.w = 1.0
        pub_edge_goalAl.markers.append(pub_edge)
        pub_edge_goalAl.markers.append(pub_goal)
        pub_edge_goalAl.markers.append(pub_goalo)
        self.all_edge_goal_pub.publish(pub_edge_goalAl)
        
            
    def global_goal_callback(self,goal):
        if goal.pose.position.z == 0: 
            self.global_goal = np.array([goal.pose.position.x, goal.pose.position.y, 1.0])
        else:
            self.global_goal = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])
        print('goal received!!')
    def pos_vel(self,pos,vel,pcl):
        # self.ros_data["local_position"] = pos.pose
        self.ros_data["local_position"] = pos
        self.angular_velocity_estimator.append(pos)
        self.ros_data["velocity"] = vel
        self.accelerator_estimator.append(vel)
        assert isinstance(pcl, PointCloud2)

        data = list(point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True))
        # print('z mean:',np.mean(np.array(data)[:,2]))
        self.ros_data["plcall"] = data
        self.pcl_time = pcl.header.stamp.secs + pcl.header.stamp.nsecs * 1e-9
        self.vel_time = vel.header.stamp.secs + vel.header.stamp.nsecs * 1e-9
        self.pos_time = pos.header.stamp.secs + pos.header.stamp.nsecs * 1e-9
        # print("alighed")
        self.if_align = 1
    def reset(self):
        self.taken_off=False
        self.ros_data = {}
        self.assure_reseted = False
        self.angular_velocity_estimator.reset()
        self.accelerator_estimator.reset()
        
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        MavrosTestCommon.setup()
        MavrosTestCommon.wait_for_topics(60)
        MavrosTestCommon.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        MavrosTestCommon.log_topic_vars()
        MavrosTestCommon.set_mode("OFFBOARD", 5)
        MavrosTestCommon.set_arm(True, 5)
        print(1)
        
    def landing(self):
        rospy.loginfo_throttle(1,"UAV start landing...")
        ret = self.set_landing(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)

        return ret

    def is_user_reset(self):
        state = self.ros_data.get("state", None)
        if state is not None:
            if state.armed==False and state.mode != "OFFBOARD":
                return True
        return False
    
    def prepare_offboard(self):
        # self.test_posctl()
        if self.ros_data.get("local_position", None) is not None:
            rx,ry,rz,_,_,_=self.parse_local_position("e")
            for i in range(20):
                self.set_local_position(rx,ry,1.0)
                time.sleep(2.0 / self.ros_rate)
            # self.uavmode.set_mode("OFFBOARD", 5)           
    def take_off(self):
    # self.test_posctl()
        rx,ry,rz,_,_,y=self.parse_local_position("e")
        yaw=math.atan2(self.goal[1]-ry,self.goal[0]-rx)
        if abs(yaw-y)>math.pi:
            yaw=-yaw
        # if abs(yaw-y)>math.pi:
        #    yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
        # while rz<0.6:
        #     print('z error:',rz-1.0)
        #     self.set_local_position(rx,ry,0.7,y)
            
        #     _,_,rz,_,_,y=self.parse_local_position("e")
        #     time.sleep(0.05)
        while rz<1.0:
            print('z error:',rz-1.0)
            self.set_local_position(rx,ry,1.2,y)
            
            _,_,rz,_,_,y=self.parse_local_position("e")
            time.sleep(0.05)
#        uavofb=MavrosTestCommon()
#        uavofb.setUp()
#        uavofb.set_mode("OFFBOARD", 10)
#        while abs(yaw-y)>0.2:
#            yaw=math.atan2(self.goal[1]-ry,self.goal[0]-rx)
#            if abs(yaw-y)>math.pi:
#               yaw=-yaw
#            self.set_local_position(rx,ry,1.0,yaw)
            # time.sleep(1.0 / self.ros_rate)
        
#            _,_,_,_,_,y=self.parse_local_position("e")
        # self.uavmode.set_mode("OFFBOARD", 5)

    def turn_to_goal(self):
        rx,ry,rz,r,p,y=self.parse_local_position("e")
        vel = TwistStamped()
        vel.header = Header()
        vel.header.frame_id = "map"
        vel.header.stamp = rospy.Time.now()
        vel.twist.linear.x = 0
        vel.twist.linear.y = 0
        vel.twist.linear.z = 0
        vel.twist.angular.x = 0#pid.step(0-r)
        vel.twist.angular.y = 0#pid.step(0-p)
        if self.global_goal is None:
            self.global_goal = self.goal
        yaw=math.atan2(self.global_goal[1]-ry,self.global_goal[0]-rx)

        if abs(yaw-y)>math.pi:
            yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
        while abs(yaw-y)>0.1:
            yaw=math.atan2(self.global_goal[1]-ry,self.global_goal[0]-rx)
            if abs(yaw-y)>math.pi:
                yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
            vel.twist.angular.z = self.pid.step(yaw-y)
            self.pos_setvel_pub.publish(vel)
            time.sleep(2.0 / self.ros_rate)
            rx,ry,rz,r,p,y=self.parse_local_position("e")
    def is_start_ready(self):
        ret = False

        condition = all(
            [
                self.ros_data.get("local_position", None) is not None,
                self.ros_data.get("imu", None) is not None,
                self.ros_data.get("velocity", None) is not None,
                self.ros_data.get("state", None) is not None,
                self.goal is not None
            ]
        )

        # print("local_position",self.ros_data.get("local_position", None))
        # print("imu",self.ros_data.get("imu", None))
        # print("velocity",self.ros_data.get("velocity", None))
        # print("state",self.ros_data.get("state", None))

        if condition:
            if self.ros_data["state"].connected:
                if self.ros_data["state"].mode != "OFFBOARD":
                    self.assure_reseted = True

                if self.ros_data["state"].mode == "OFFBOARD" and self.ros_data["state"].armed :#and self.assure_reseted:
                    ret = True
                else:
                    rospy.loginfo_throttle(1,"Please meet the takeoff condition first!\n mode:%s, armed:%s"% (self.ros_data["state"].mode,self.ros_data["state"].armed))
            else:
                rospy.logwarn_throttle(1,"mavros connection failure!")
        else:
            rospy.logwarn_throttle(1,"subscribed data has not been fully prepared, waiting for data...")

        return ret

    def safety_monitor(self):
        x=self.ros_data["local_position"].pose.position.x
        y=self.ros_data["local_position"].pose.position.y
        z=self.ros_data["local_position"].pose.position.z
        if z>=0.35:
            self.taken_off=True
        #note that z axis have a hystersis, for the shaking avoidance..


        state=self.ros_data.get("state", None)
        if state is not None:
            
            if state.armed==False or state.mode!="OFFBOARD":
                rospy.loginfo_throttle(1,"Current status unable to takeoff!\n mode:%s, armed:%s"% (self.ros_data["state"].mode,self.ros_data["state"].armed))
                rospy.logwarn("user interupted! trying to end this episode...")
                return True
        else:
            rospy.logwarn("abnormal state!") # this should not happen....
            return True

        # if self.taken_off and (x<-2.5 or x>2.5 or y<-1.5 or y>1.5 or z<0.25 or z>3):
        if self.taken_off and (z<0.25 or z>5):
            rospy.logwarn("land due to safety constraint! pos:%s" % str((x,y,z)) )
            return True

        return False

    def globalmap_callback(self, data):
        self.ros_data["globalmap"]=data
        
    def local_position_callback(self, data):
        self.ros_data["local_position"] = data
        self.angular_velocity_estimator.append(data)
        
    def goal_callback(self,data):
        # print("goal::::",data)
        if self.goal is None:
            self.goal=np.array([data.x,data.y,data.z])
        if data.z==0.1:
            self.ifend=1
            self.goal[0:2]=np.array([data.x,data.y])
        else:
            self.ifend=0
            self.goal=np.array([data.x,data.y,data.z])
        
    def Imu_callback(self, data):
        self.ros_data["imu"] = data

    def velocity_callback(self, data):
        self.ros_data["velocity"] = data
        self.accelerator_estimator.append(data)

    def state_callback(self, data):
        self.ros_data["state"] = data
        
    def plc_callback(self, data):
        assert isinstance(data, PointCloud2)
        global gen

        data = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        
        # print('z mean:',np.mean(np.array(data)[:,2]))
        self.ros_data["plc"] = data
    
    def plcall_callback(self, data):
        assert isinstance(data, PointCloud2)
        global gen
        self.pcl_time = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        data = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        
        # print('z mean:',np.mean(np.array(data)[:,2]))
        self.ros_data["plcall"] = data
    def user_control_init(self):
        pass# raise NotImplementedError

    def user_control_logic(self):
        raise NotImplementedError
    
    def user_control_end(self,mode="normal"):
#        pass# raise NotImplementedError
        uavoff=MavrosTestCommon()
        uavoff.setUp()
        uavoff.set_arm(False, 5)  
        uavoff.set_mode("STABILIZED", 10)
        rospy.loginfo("UAV disarmed!")
    def set_local_position(self,x,y,z,yaw=0):
        pos = PoseStamped()
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
        quaternion = quaternion_from_euler(0, 0, yaw)
        pos.pose.orientation = Quaternion(*quaternion)
        pos.header = Header()
        pos.header.frame_id = "map"
        pos.header.stamp = rospy.Time.now()
        self.pos_setpoint_pub.publish(pos)
        
    def xyz_array_to_pointcloud2(self,points, frame_id=None, stamp=None,):
        '''
        Create a sensor_msgs.PointCloud2 from an array
        of points.
        '''
        msg = PointCloud2()
        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12*points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()
        
        return msg
    def set_velocity(self,ifend_old):
        plc=[]
        plcall=[]
        if self.ros_data.get("plc", None) is not None:
            plc=self.ros_data["plc"]  #points of octomap
            # print("plc:",plc)
            self.d3_check = np.array(plc[-3::])
            plc=plc[0:-3]
            print("3d points received!",self.d3_check)
        if self.ros_data.get("plcall", None) is not None:
            plcall=self.ros_data["plcall"]  #points of depth camera
        rx,ry,rz,r,p,y=self.parse_local_position("e")
        vx,vy,vz,vro,vpi,vya=self.parse_velocity()
#        ros_t = rospy.get_rostime().secs+rospy.get_rostime().nsecs*1e-9
#        local_pos=np.array([rx,ry,rz]) + np.array([vx,vy,vz]) * (ros_t - self.pos_time)
#        local_ori=np.array([r,p,y]) + np.array([vro,vpi,vya]) * (ros_t - self.pos_time)
        local_pos=np.array([rx,ry,rz])
        local_ori=np.array([r,p,y])
        self.local_pos = local_pos
        self.path_rec.append(local_pos.copy())
        loc_goal=self.goal-local_pos
        ax,ay,az=self.parse_linear_acc()
        
        state=np.array([local_pos[0],local_pos[1],local_pos[2],vx,vy,vz,ax,ay,az])
        if self.global_goal is None:
            self.global_goal = self.goal
        print("self.if_d3_goal",self.if_d3_goal)
        # if len(self.glb_goal_3d) and (self.if_d3_goal)<0.5: #or np.linalg.norm(self.local_pos-self.d3_pos)
        #     yaw_goal = self.glb_goal_3d
        # else:
        #     yaw_goal = self.goal
        if self.d3_check is not None and len(self.d3_check):
            yaw_goal = self.d3_check[-1]
        else:
            yaw_goal = self.global_goal
        yaw_glb = math.atan2(yaw_goal[1]-ry,yaw_goal[0]-rx)
        yaw_loc = math.atan2(self.goal[1]-ry,self.goal[0]-rx)
        if abs(yaw_glb-yaw_loc)>math.pi:
            yaw_loc=(2*math.pi-abs(yaw_loc))*np.sign(-yaw_loc)
        
        if abs(yaw_glb - yaw_loc) > math.pi/3:
            yaw = (yaw_glb*1.5 +yaw_loc*0.5)/2
        else:
            yaw = yaw_glb
        if abs(yaw-y)>math.pi:
            yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
        # loc_goal_3d = self.glb_goal_3d - self.local_pos
        # yaw_3dglb=0
        if not ifend_old and self.if_align:
            velo,self.loc_goal_old,self.f_angle,steptime,wptime,optitime,iternum,pointnum,self.dyn_time,self.pretime,plc_use,self.if_direct,self.no_path,traj_dif,loc_goal_3d,self.d3_pos,self.if_d3_goal,self.control,self.wpts,self.pred_trj,self.pcl_w = control_method.start(plc,plcall,state,loc_goal,local_ori[0],local_ori[1],local_ori[2],self.loc_goal_old, 
                                           self.f_angle,self.path_rec,self.pretime,self.dyn_time,self.if_direct,self.no_path,
                                               self.pp_restart,self.pcl_time,self.d3_check,self.d3_pos,self.glb_goal_3d,self.if_d3_goal,self.control,self.wpts)
            self.if_align = 0
            self.f1.write(str(pointnum)+" ,"+str(iternum)+" ,"+str(wptime)+" ,"+str(optitime)+" ,"+str(steptime)+" ,"+str(traj_dif)+"\n")
            print("write data")
            if len(loc_goal_3d) !=0:
                self.glb_goal_3d = loc_goal_3d+self.local_pos
                yaw_3dglb = math.atan2(self.glb_goal_3d[1]-ry,self.glb_goal_3d[0]-rx)
                if abs(yaw_3dglb-yaw_loc)>math.pi:
                    yaw_loc=(2*math.pi-abs(yaw_loc))*np.sign(-yaw_loc)
                if abs(yaw_3dglb - yaw_loc) > math.pi/3:
                    yaw = (yaw_3dglb*1.5 +yaw_loc*0.5)/2
                else:
                    yaw = yaw_3dglb
            else:
                self.glb_goal_3d = []
            self.velo_goal=self.loc_goal_old*0.5+local_pos
            self.steptime.append(steptime)
            if self.d3_check is not None and np.linalg.norm(self.d3_check[-1,0:2]-self.local_pos[0:2])<1:
                velo = np.linalg.norm(self.d3_check[-1,0:2]-self.local_pos[0:2])*np.array(velo)
            # time.sleep(0.5)
            # print("sleep 0.5s to wait global goal update")
            vel = TwistStamped()
            vel.header = Header()
            vel.header.frame_id = "map"
            vel.header.stamp = rospy.Time.now()
            vel.twist.linear.x = velo[0]
            vel.twist.linear.y = velo[1]
            vel.twist.linear.z = velo[2]
            vel.twist.angular.x = 0#pid.step(0-r)
            vel.twist.angular.y = 0#pid.step(0-p)
            vel.twist.angular.z = self.pid.step(yaw-y)
            self.vel=vel
            if len(plc_use):
                plc_use=self.xyz_array_to_pointcloud2(np.array(plc_use),'map',rospy.Time.now())
                self.pcl_use_pub.publish(plc_use)

        if abs(yaw-y) > math.pi/4: #len(self.old_goal)!= 0 and (self.old_goal != self.goal).any():
            while abs(yaw-y)>math.pi/4:
            #for i in range(10):
                rx,ry,rz,r,p,y=self.parse_local_position("e")
                yaw_glb = math.atan2(yaw_goal[1]-ry,yaw_goal[0]-rx)
                yaw_loc = math.atan2(self.goal[1]-ry,self.goal[0]-rx)
                if abs(yaw_glb-yaw_loc)>math.pi:
                    yaw_loc=(2*math.pi-abs(yaw_loc))*np.sign(-yaw_loc)
                if abs(yaw_glb - yaw_loc) > math.pi/4:
                    yaw = (yaw_glb*0.5 +yaw_loc*1.5)/2
                else:
                    yaw = yaw_glb
                if abs(yaw-y)>math.pi:
                    yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
                self.set_local_position(rx,ry,max(rz,1),y+np.sign(yaw-y)*0.3)
                # self.set_local_position(rx,ry,1.5,yaw)
                print(yaw,y)
                time.sleep(0.1)
            print("position cmd published!")
            self.pp_restart = 1

#            
        elif self.no_path == 2 and len(self.path_rec)>3:
            for i in range(10):
                self.set_local_position(self.path_rec[-4][0],self.path_rec[-4][1],self.path_rec[-4][2],yaw)
                time.sleep(0.05)
                # del self.path_rec[-2::]
        else:
            self.pos_setvel_pub.publish(self.vel)
            print("velocity cmd published!-uav controller")
            self.pp_restart = 0
            self.ros_data["plc"] = None
        self.old_goal = self.global_goal.copy()

        if self.d3_check is not None:
            if len(self.glb_goal_3d) and (self.if_d3_goal):  #==1 or np.linalg.norm(self.local_pos-self.d3_pos))<0.5 
                self.pub_3d_path([local_pos,self.glb_goal_3d,self.d3_check[-1]])
            else: 
                self.pub_3d_path([local_pos])
             
            print("3d path published")
        if len(self.all_edge_goal):
            self.pub_all_edge_goal()
        if len(self.wpts):
            self.publish_apfpath(self.wpts)
        if len(self.pred_trj):
            self.publish_predtrj(self.pred_trj)
        if len(self.pcl_w):
            self.pub_obs(self.pcl_w)
       
    def set_accel(self):
        plc=self.ros_data["plc"]
        rx,ry,rz,r,p,y=self.parse_local_position("e")
        
        local_pos=np.array([rx,ry,rz])
        self.path_rec.append(local_pos.copy())
        loc_goal=self.goal-local_pos
        ax,ay,az=self.parse_linear_acc()
        vx,vy,vz,_,_,_=self.parse_velocity()
        state=np.array([rx,ry,rz,vx,vy,vz,ax,ay,az])
        accel,self.loc_goal_old,self.f_angle = control_method.start(plc,state,loc_goal,r,p,y,self.loc_goal_old,self.f_angle,self.path_rec)
        acc = Vector3Stamped()
        acc.header = Header()
        acc.header.frame_id = "map"
        acc.header.stamp = rospy.Time.now()
        acc.vector.x = accel[0]
        acc.vector.y = accel[1]
        acc.vector.z = -accel[2]-self.g
        # acc.accel.angular.x = 0
        # acc.accel.angular.y = 0
        # acc.accel.angular.z = 0
        # rospy.Publisher(
        #     'mavros/setpoint_accel/send_force', True, queue_size=10)
        self.pos_setacc_pub.publish(acc)

    def parse_local_position(self, mode="q"):
        local_position=self.ros_data["local_position"]
        rx=local_position.pose.position.x
        ry=local_position.pose.position.y
        rz=local_position.pose.position.z
        
        qx=local_position.pose.orientation.x
        qy=local_position.pose.orientation.y
        qz=local_position.pose.orientation.z
        qw=local_position.pose.orientation.w
#        self.path.append([rx,ry,rz])
        if mode == "q":
            return (rx,ry,rz,qx,qy,qz,qw)

        elif mode == "e":
            rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
            return (rx,ry,rz)+ rpy_eular
        else:
            raise UnboundLocalError
            
    def parse_accelerate_by_local_vel(self):
        return self.accelerator_estimator.estimate()

    def parse_angular_velocity_by_local_position(self):
        return self.angular_velocity_estimator.estimate()
    
    def parse_velocity(self):
        wx=self.ros_data["velocity"].twist.angular.x
        wy=self.ros_data["velocity"].twist.angular.y
        wz=self.ros_data["velocity"].twist.angular.z
        vx=self.ros_data["velocity"].twist.linear.x
        vy=self.ros_data["velocity"].twist.linear.y
        vz=self.ros_data["velocity"].twist.linear.z
        return (vx,vy,vz,wx,wy,wz)
    
    def parse_linear_acc(self):
        ax=self.ros_data["imu"].linear_acceleration.x
        ay=self.ros_data["imu"].linear_acceleration.y
        az=self.ros_data["imu"].linear_acceleration.z
        return (ax,ay,az-self.g)

    def run(self, mode = "Takeoff"):
        state = "init"
        while not rospy.is_shutdown():
           
            if mode == "Takeoff" and state != "init" and state!="reset" and state!= "wait_signal" and self.is_user_reset():
                state = "reset"
#                self.user_control_end(mode="force")
                rospy.loginfo("user reset from the remote controller!")
            if state == "init":
                self.register()
                self.user_control_init()
                state = "reset"

            elif state == "reset":
                self.reset()
                
                if mode == "Takeoff":
                    state = "wait_signal"
                else:
                    state = "user_defined1"
            
            elif state == "wait_signal":

                # time.sleep(0.1)
                # rospy.loginfo(state)
                print("111")
                self.prepare_offboard()
                # rospy.loginfo("reset")
                print("22")
                ret = self.is_start_ready()
                print("33")
                if ret:
                    # self.prepare_offboard()
                    
                    self.take_off()
                    self.turn_to_goal()
                    state = "program"
                    rospy.loginfo("now starting user program!")

            elif state == "program":
                if self.safety_monitor() or self.user_control_logic():
                    print('average step time:',np.mean(self.steptime))
                    state = "landing"
                    rospy.loginfo("landing triggered!")
                    
            elif state == "landing":
                if not self.landing():
                    rospy.logwarn("error occured in the landing process!")
                    state = "error"
                else:        
                    state="finish"
                    rospy.loginfo("Task finished! waiting for user control!")
                    
                    self.f1.close()
                    self.f2.close()
#                if state!="landing":
#                    self.user_control_end()

            elif state=="finish":
                time.sleep(0.01)
                rx,ry,rz,r,p,y=self.parse_local_position("e")
                if rz < 0.3:
                    self.user_control_end()
                    
            elif state == "error":
                time.sleep(0.01)
                rospy.logwarn_throttle(
                    0.5, "error occured! need manual operation")
            elif state == "user_defined1":
                self.user_control_init()
                self.user_control_reset()
                state = "user_defined2"
            elif state == "user_defined2":
                if self.user_control_logic():
                    self.user_control_end()
                    state = "finish"
            else:
                raise StateMachineError
