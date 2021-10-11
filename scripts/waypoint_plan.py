#!/usr/bin/env python2
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
from rtree import index
from FET_utils import FET
from utils import earth_to_body_frame,body_to_earth_frame
import os

class waypoint_planner (object):
    def register(self):
       
        # self.f1 = open("/home/" + getpass.getuser() +"/catkin_ws/src/AHPF_planner/data/"+time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()))+"OT.txt", 'a')
        # self.f2 = open("/home/" + getpass.getuser() +"/catkin_ws/src/AHPF_planner/data/"+time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()))+"PV.txt", 'a')
        self.dr = 0.6
        self.det_range = 6
        self.sample_dis = 0.6
        self.SA = 0.4
        self.path_num = 3
        self.path_num_max = 6
        self.start_height = 1 #m
        self.max_height = 3.5
        self.back_distance = 0.5
        self.ros_data = {}
        self.waypoints=[]
        # self.ros_data["pcl_cam"] = None
        # self.ros_data["local_position"]  = None
        self.cam_h_ang = 43.5*math.pi/180
        self.cam_v_ang = 29*math.pi/180 #half FOV degree
        self.xb,self.yb,self.zb = 0.12,0,0
        self.keynum_dif_thre = 0.1 #keypoint number relative difference
        self.rec_key_dis = 1.0
        self.rec_key_time = 0.5
        self.last_key_num = 0
        self.wp_time = 0
        self.last_pos = np.array([0,0,0])
        self.last_time = 0
        self.key_num = None
        self.global_goal= None
        self.global_pcl = []
        self.cam_ori = []
        self.if_align = 0
        self.key_id = 0
        self.goal_alter = 0
        rospy.init_node('waypoint_planner')
        rospy.loginfo("waypoint_planner started!")
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
        self.goal_sub = rospy.Subscriber('/goal_global',
                                              Point,
                                              self.goal_callback)
        self.local_imu_sub = rospy.Subscriber('mavros/imu/data',
                                              Imu,
                                              self.Imu_callback)
        self.state_sub = rospy.Subscriber('mavros/state',
                                          State,
                                          self.state_callback)

        self.pos_setpoint_pub = rospy.Publisher(
                    'mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.alpub = rospy.Publisher('/points_global_all', PointCloud2, queue_size=10)
        self.wpts_pub = rospy.Publisher('/wpts_path', Path, queue_size=10)
        self.set_landing = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.path_3d_pub = rospy.Publisher('path_3d', Path, queue_size=10)
        self.sample_pub = rospy.Publisher('/tree', MarkerArray, queue_size=10)
        self.thrust_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget,queue_size=10)
        self.cam_fov_pub = rospy.Publisher('/camera_fov', Marker, queue_size=10)
        self.tss = ApproximateTimeSynchronizer([
                                                # Subscriber('/gt_iris_base_link_imu', Odometry),
                                                Subscriber('/mavros/local_position/pose', PoseStamped),
						# Subscriber('mavros/vision_pose/pose', PoseStamped),
                                                Subscriber('mavros/local_position/velocity_local',TwistStamped),
                                                Subscriber('/filtered_RadiusOutlierRemoval',PointCloud2)],
        7,0.06, allow_headerless=True)
        self.tss.registerCallback(self.pos_vel)
        self.globalgoal_sub = rospy.Subscriber('/move_base_simple/goal',
                                    PoseStamped,
                                    self.goal_callback,queue_size=1)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        
    def goal_callback(self,goal):
        self.goal_alter = 1
        if goal.pose.position.z == 0: 
            self.global_goal = np.array([goal.pose.position.x, goal.pose.position.y, 1.0])
        else:
            self.global_goal = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])
        print('goal received!!')
        
    def Imu_callback(self, data):
        self.ros_data["imu"] = data
    def state_callback(self, data):
        self.ros_data["state"] = data
    def pos_vel(self,pos,vel,pcl):
        # self.ros_data["local_position"] = pos.pose
        self.ros_data["local_position"] = pos
        # self.angular_velocity_estimator.append(pos)
    
        assert isinstance(pcl, PointCloud2)
        data = list(point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True))
        # print('z mean:',np.mean(np.array(data)[:,2]))
        self.ros_data["pcl_cam"] = data
        self.pcl_timestamp = pcl.header.stamp
        self.pcl_time = pcl.header.stamp.secs + pcl.header.stamp.nsecs * 1e-9
        self.pos_time = pos.header.stamp.secs + pos.header.stamp.nsecs * 1e-9
        self.ang_vel = np.array([vel.twist.angular.x,vel.twist.angular.y,vel.twist.angular.z])
        self.line_vel = np.array([vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z])
        self.ros_data["velocity"] = self.line_vel
        # print("alighed")
        self.if_align = 1
        
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
    def insert_keyframe (self,pos,ori,mode=1): #pos,ori:list
        now = rospy.get_time()
        time_pos_dif = (now-self.last_time > self.rec_key_time) and (np.linalg.norm(pos-self.last_pos) > self.rec_key_dis)
        if_insert = (mode ==1 and self.key_num is not None and time_pos_dif and abs(self.key_num-self.last_key_num)/self.key_num > self.keynum_dif_thre) or\
        (mode ==2 and time_pos_dif)
        if if_insert:
            self.idx.insert(self.key_id,tuple(pos))
            self.last_key_num = self.key_num
            self.last_time = now
            self.last_pos = pos
            self.key_id+=1
            self.cam_ori.append(np.r_[pos,ori])
            print("new key frame inserted!",now,pos)
    def pub_cam_fov (self):
        fov = Marker()
        pose = np.array(self.parse_local_position(mode="e"))
        uav_pos=pose[0:3]
        fov.type = fov.LINE_LIST
        fov.header.frame_id = "map"
        fov.header.stamp = rospy.Time.now()
       
        fov.color.a = 1.0
        fov.color.b = 1.0
        fov.scale.x = 0.03
        p3=Point()
        p3.x,p3.y,p3.z = uav_pos
        dots=np.matmul(self.b2e,self.dots.T).T+uav_pos
        for i in range(4):
            p1=Point()
            p2=Point()
            p1.x,p1.y,p1.z = dots[i]
            if i < 3:
                p2.x,p2.y,p2.z = dots[i+1]
            else:
                p2.x,p2.y,p2.z = dots[0]
            fov.points.append(p1)
            fov.points.append(p2)
            fov.points.append(p3)
            fov.points.append(p1)
        fov.id=6
        fov.pose.orientation.w = 1.0
        self.cam_fov_pub.publish(fov)
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
        
    def prepare_offboard(self):
        # self.test_posctl()
        if not self.ros_data["state"].armed:
            try:
                res1 = self.set_arming_srv(True)
                if not res1.success:
                    rospy.logerr("failed to send arm command")
            except rospy.ServiceException as e:
                    rospy.logerr(e)
        if self.ros_data.get("local_position", None) is not None:
            rx,ry,rz,_,_,_=self.parse_local_position("e")
            for i in range(20):
                # self.set_local_position(rx,ry,1.0)
                self.set_thrust()
                if self.ros_data["state"].mode != "OFFBOARD" and i%10==0:
                    try:
                        res2 = self.set_mode_srv(0, "OFFBOARD")
                        if not res2.mode_sent:
                            rospy.logerr("failed to send mode command")
                    except rospy.ServiceException as e:
                        rospy.logerr(e)
                time.sleep(0.05)
            # self.uavmode.set_mode("OFFBOARD", 5)       
    def landing(self):
        rospy.loginfo_throttle(1,"UAV start landing...")
        ret = self.set_landing(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)

        return ret
    def take_off(self):
    # self.test_posctl()
        rx,ry,rz,_,_,y=self.parse_local_position("e")
        yaw=math.atan2(self.global_goal[1]-ry,self.global_goal[0]-rx)
        if abs(yaw-y)>math.pi:
            yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
        # for i in range(40):
        #     self.set_thrust(0.99)
        #     time.sleep(0.05)
        dy = np.sign(yaw-y)*min(0.2,abs(yaw-y)*0.1)
        while rz<self.start_height:
            print('z error:',rz-self.start_height)
            self.set_local_position(rx,ry,self.start_height+0.2,y)
            
            _,_,rz,_,_,y=self.parse_local_position("e")
            time.sleep(0.1)
        
    def set_thrust(self,thrust=0.7059122):
        attitude_raw = AttitudeTarget()
        attitude_raw.orientation.w = 1.0
        # attitude_raw.orientation.x = att_quat.q2;
        # attitude_raw.orientation.y = att_quat.q3;
        # attitude_raw.orientation.z = att_quat.q4;
        attitude_raw.thrust = thrust
        attitude_raw.type_mask = 0b00000111
        self.thrust_pub.publish(attitude_raw)
    def turn_to_goal(self):
        rx,ry,rz,r,p,y=self.parse_local_position("e")
        yaw=math.atan2(self.global_goal[1]-ry,self.global_goal[0]-rx)

        if abs(yaw-y)>math.pi:
            yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
        while abs(yaw-y)>0.1:
            yaw=math.atan2(self.global_goal[1]-ry,self.global_goal[0]-rx)
            if abs(yaw-y)>math.pi:
                yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)

            self.set_local_position(rx,ry,rz,y+(yaw-y)*0.3)
            time.sleep(0.1)
            _,_,_,r,p,y=self.parse_local_position("e")
        
    def is_start_ready(self):
        ret = False
        received = [
                self.ros_data.get("local_position", None) is not None,
                self.ros_data.get("imu", None) is not None,
                self.ros_data.get("velocity", None) is not None,
                self.ros_data.get("state", None) is not None,
                self.global_goal is not None
            ]
        condition = all(received)

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
            # print(received)

        return ret
    def pub_waypoints(self,waypoints):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for d in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)
        self.wpts_pub.publish(path)
    def pub_samples(self,all_sample):
        pose = np.array(self.parse_local_position(mode="e"))
        uav_pos = pose[0:3]
        sps = MarkerArray()
        sp1 = Marker()
        sp1.type = sp1.POINTS
        sp1.header.frame_id = "map"
        sp1.header.stamp = rospy.Time.now()
        sp1.color.a=1
        sp1.color.g = 1
        sp1.id = 0
        sp1.pose.orientation.w=1.0
        sp1.scale.x,sp1.scale.y = 0.1,0.1
        # if len(all_sample):
        #     sp1.action = Marker.DELETEALL
        #     self.sample_pub.publish(sp)
        # else:
        sp1.action = Marker.ADD
        sp2 = Marker()
        sp2.type = sp2.POINTS
        sp2.header.frame_id = "map"
        sp2.header.stamp = rospy.Time.now()

        sp2.id = 1
        sp2.pose.orientation.w=1.0
        sp2.scale.x,sp2.scale.y = 0.1,0.1
        sp2.color.a=1
        sp2.color.r = 1
        # if len(all_sample):
        #     sp2.action = Marker.DELETEALL
        #     self.sample_pub.publish(sp)
        # else:
        sp2.action = Marker.ADD
        # print("all_sample",all_sample)
        if len(all_sample):
            print("samples:",all_sample[1])
            for jt in all_sample[1]:
                
             
                p = Point()
                p.x,p.y,p.z = jt+uav_pos 
                sp1.points.append(p)       
            for jt in all_sample[0]:
                p = Point()
                p.x,p.y,p.z = jt+uav_pos 
                sp2.points.append(p)  
            sps.markers.append(sp1)
            sps.markers.append(sp2)
            self.sample_pub.publish(sps)
            
    def xyz_array_to_pointcloud2(self,points, frame_id=None, stamp=None):
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
            
    def get_waypoints(self,seeds):
        
        self.if_align=0
        pose = np.array(self.parse_local_position(mode="e"))
        uav_pos = pose[0:3]
        uav_ori = pose[3::]
        uav_pos = (self.pcl_time - self.pos_time)*self.line_vel + uav_pos
        uav_ori = (self.pcl_time - self.pos_time)*self.ang_vel + uav_ori
        dis2goal = np.linalg.norm(self.global_goal-uav_pos)
    

        
        if 1 < dis2goal < self.det_range:
            goal = self.global_goal-uav_pos
            if_pos_control = 0
        elif dis2goal <1:
            if_pos_control = 1
            goal = self.global_goal
        else:
            goal = (self.global_goal-uav_pos)/dis2goal*self.det_range
            if_pos_control = 0
        if if_pos_control:
            for i in range(20):
                self.set_local_position(goal[0],goal[1],goal[2])
                rospy.sleep(0.1)
            return "arrived",[],[]
        self.insert_keyframe (uav_pos,uav_ori,2)
        pcl = np.array(self.ros_data["pcl_cam"])
        pcl_c = pcl.copy()
        pcl_c[:,0]=pcl[:,2]+self.xb  #distance from camera to uav center, xb yb zb are offset
        pcl_c[:,1]=-pcl[:,0]+self.yb
        pcl_c[:,2]=-pcl[:,1]+self.zb
#                pcl_c=np.array(pcl_c)
        
        self.b2e = body_to_earth_frame(uav_ori[0],uav_ori[1],uav_ori[2])
        pcl=np.matmul(self.b2e,pcl_c.T).T +np.tile(uav_pos,(len(pcl_c),1))
        # pcl_global = pcl[pcl[:,2]>0.3]
        pcl_global = pcl[::4]
        pcl = (pcl_global - uav_pos)
        if len(self.waypoints):
            if self.goal_alter==0 and FET.check_path(pcl,wp.waypoints-uav_pos,self.SA) and np.linalg.norm(self.waypoints[0]-uav_pos) < self.det_range-1:
                print("use old waypoints!")
                self.goal_alter = 0
                return "waypoint",self.waypoints,[]
        if_path=2
        path_num = self.path_num
        
        while if_path==2 and path_num<=self.path_num_max:
            print("path_num",path_num)
            t_start = time.time()
            if_path,waypoints,all_sample = FET.get_path (pcl,goal,self.dr,self.det_range,self.SA,seeds,path_num,
                                                         self.idx,[self.det_range,self.cam_h_ang,self.cam_v_ang],
                                                         self.cam_ori,pose,self.max_height-uav_pos[2])
            self.wp_time = time.time()-t_start
            path_num += 1
            
        self.global_pcl = pcl_global
        
        print("waypoint time cost:",self.wp_time)
        if if_path==1:
            waypoints = np.array([uav_pos,uav_pos+goal])
            print("directly fly to goal!",len(pcl))
            return "waypoint",waypoints+uav_pos,[]
        elif if_path==0:
            print("success waypoints:",waypoints,"all_sample len:",len(all_sample),"check_num:",FET.check_num,"check_fov_num",FET.check_fov_num,"pcl len:",len(pcl))
            return "waypoint",waypoints+uav_pos,all_sample
        else:
            print("no path, go back!")
            return "goback",[],[]
    def go_back(self):
        pose = np.array(self.parse_local_position(mode="e"))
        uav_pos = pose[0:3]
        if len(self.cam_ori):
            for i in range(-1,-len(self.cam_ori)-1,-1):
                backpos = self.cam_ori[i][0:3]
                if np.linalg.norm(backpos-uav_pos) > self.back_distance:
                    break
        else:
            backpos = uav_pos
        for i in range(10):
            self.set_local_position(backpos[0],backpos[1],backpos[2],pose[-1]) 
            rospy.sleep(0.05)
        return "waypoint"
                
if __name__ == "__main__":
    wp = waypoint_planner()
    wp.register()
    # print("wp.cam_h_ang",wp.cam_h_ang,wp.cam_v_ang,wp.path_num)
    FET = FET()
    p = index.Property()
    p.dimension = 3
    p.dat_extension = 'data'
    p.idx_extension = 'index'
    wp.idx = index.Index(properties=p)
    wp_time = 0
    ii=0
    mode="take off"
    seeds = FET.gen_angle_seed(wp.sample_dis,wp.dr,int(np.rint(wp.det_range/wp.dr)))
    ss= (wp.det_range*np.cos(wp.cam_h_ang))
    dot1 = np.array([wp.det_range,-wp.det_range*np.tan(wp.cam_h_ang),ss*np.tan(wp.cam_v_ang)])
    dot2 = dot1*np.array([1,-1,1])
    dot3 = dot1*np.array([1,-1,-1])
    dot4 = dot1*np.array([1,1,-1])
    wp.dots = np.r_[[dot1],[dot2],[dot3],[dot4]]
    # print("body dots:",wp.dots,np.cos(wp.cam_h_ang),np.sin(wp.cam_h_ang),wp.cam_h_ang)
    print("seeds list length:",len(seeds),len(seeds[0]))
    # pid = os.fork()
    # if pid == 0:
        
    #     uavmode=MavrosTestCommon()
    #     uavmode.setUp()
    #     uavmode.set_arm(True, 5)
    #     try:
    #         uavmode.set_mode("OFFBOARD", 20)
    #     except:
    #         print("set mode fail")
        
    # else:
    while not rospy.is_shutdown():
        # if not wp.if_align:
        #     # rospy.sleep(0.5)
        #     print("wait for data align.")
        #     continue
        # if wp.global_goal is None:
        #     rospy.sleep(0.5)
        #     print("wait for a goal...")
        #     continue
            

        
        if mode=="take off":
            # wp.prepare_offboard()
            
            while (not wp.is_start_ready()):
                wp.prepare_offboard()
            wp.take_off()
            wp.turn_to_goal()
            mode = "waypoint"
        elif mode =="waypoint":
            if wp.if_align:
                mode,wp.waypoints,all_sample = wp.get_waypoints(seeds)
                wp.pub_samples(all_sample)
                wp.pub_cam_fov()
            if len(wp.waypoints):
                wp.pub_waypoints(wp.waypoints)
            if len(wp.global_pcl):
                glb_pcl2 = wp.xyz_array_to_pointcloud2(wp.global_pcl,'map',wp.pcl_timestamp)
                wp.alpub.publish(glb_pcl2)
        elif mode =="arrived":
            wp.landing()
        elif mode =="goback":
            mode = wp.go_back()
     
        