#!/usr/bin/env python2
import rospy
from utils import q2rpy_rad

from UAVController import UAVController
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import tf
import numpy as np
import time
# from build_geometry_map import *
# import minimum_snap


class Program1(UAVController):
    def __init__(self):
        self.dir = 1
        self.time_pub=0
        self.ifend_old=0
        return super(Program1, self).__init__()


    def publish_path(self, data):
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
        self.path_pub.publish(path)


    def publish_locwp(self,cv):

        locwp = Marker()
        locwp.header.frame_id = "map"
        locwp.header.stamp = rospy.Time.now()
        locwp.type = Marker.ARROW
#        locwp.pose.position.x = self.velo_goal[0]
#        locwp.pose.position.y = self.velo_goal[1]
#        locwp.pose.position.z = self.velo_goal[2]
        p1=Point()
        p2=Point()
        p1.x,p1.y,p1.z=self.local_pos[0],self.local_pos[1],self.local_pos[2]
        p2.x,p2.y,p2.z=self.velo_goal[0],self.velo_goal[1],self.velo_goal[2]
        locwp.points.append(p1)
        locwp.points.append(p2)
        locwp.scale.x = 0.1
        locwp.scale.y = 0.3
        locwp.scale.z = 0.5
        locwp.color.a = 1
        locwp.color.r = 1.5*cv/2
        locwp.color.g = 1.5*max(0,1.0-cv/2)
        locwp.color.b = 0.0

        self.locwp_publisher.publish(locwp)         

    def user_control_init(self):
        self.gate_info = {}
        self.locwp_publisher = rospy.Publisher("local_wp", Marker, queue_size=1)
        # self.dynobs_publisher = rospy.Publisher("dyn_obs", Marker, queue_size=1)
        # self.dynv_publisher = rospy.Publisher("dyn_v", Marker, queue_size=1)
        # self.local_pos_sub = rospy.Subscriber("vicon/gate/gate",
        #                                       TransformStamped,
        #                                       self.gate_pos_cb)

        self.path_pub = rospy.Publisher('/uav_path', Path, queue_size=10)
        self.ros_rate = 50
        self.cross_times = 5


    def plan(self, start_point):
        path = self.calculate_path(start_point)
        return path


    def user_control_logic(self):
        # if self.vel is not None:
        #     self.pos_setvel_pub.publish(self.vel)
        time_interval = 0.05
        ux, uy, uz, uR, uP, uY = self.parse_local_position("e")
        pos_uav = np.array([ux, uy, uz])
        ifend = False
        if self.d3_check is not None and len(self.d3_check):
            # px,py = self.d3_check[-1,0:2]
            # pz = 1.5
            ifend = np.linalg.norm(self.d3_check[-1,0:2]- pos_uav[0:2])< 0.6
        if len(self.goal) > 0:
            # if self.global_goal is not None and self.ifend == 0:
            #     px, py, pz = self.global_goal
            # else:
            px, py, pz = self.goal
            ifend = ifend and np.linalg.norm(self.goal[0:2]- pos_uav[0:2])< 0.6
            # p_gate = self.gate_info["x"], self.gate_info["y"], self.gate_info["z"]
            # q_gate = self.gate_info["qx"], self.gate_info["qy"], self.gate_info["qz"], self.gate_info["qw"]
        
        vx,vy,vz,_,_,_=self.parse_velocity()
        if self.velo_goal !=[]:
            self.publish_locwp(np.linalg.norm([vx,vy,vz]))

        self.f2.write(' ,'.join([str(x) for x in [ux,uy,uz]])+" ,"+' ,'.join([str(x) for x in [vx,vy,vz]])+" ,"+' ,'.join([str(x) for x in [uR,uP,uY]])+" ,"+str(time.time())+"\n")
        print('ifend',self.ifend,ifend)
        vel1 = TwistStamped()
        vel1.header = Header()
        vel1.header.frame_id = "map"
        vel1.header.stamp = rospy.Time.now()
        vel1.twist.linear.x = 0
        vel1.twist.linear.y = 0
        vel1.twist.linear.z = 0
        vel1.twist.angular.x = 0#pid.step(0-r)
        vel1.twist.angular.y = 0#pid.step(0-p)
        vel1.twist.angular.z = 0       
        if (ifend or self.ifend) and self.tick==0:
            print("velocity to 0")
            for jj in range(30):
                self.pos_setvel_pub.publish(vel1)
                time.sleep(0.03)
        
        if ifend: #and np.linalg.norm(np.array([px, py, pz])- pos_uav)> 0.3:
            if self.tick==0:
                print('goal reached, fly to: %s , cross time remained: %s' %([px, py, pz],self.cross_times-1))
                self.cross_times -= 1
                self.register()
            # if abs(uz-pz)<0.2:
            #     self.set_local_position(px, py, 0.3,0)
            # else:
            #     self.set_local_position(px, py, pz,0)#uY)
            # self.set_local_position(0,0,1.5,0)
            print("goal reached, publish position command",self.tick)
            self.f_angle=[float("Inf"),float("Inf")]
            for ii in range(30):
                st_goal = np.array([ux, uy, uz])+(np.array([px, py, pz])-np.array([ux, uy, uz]))*ii/29
                self.set_local_position(st_goal[0],st_goal[1],st_goal[2],uY)
                time.sleep(0.1)
            self.tick += 1
            
            
            if self.tick > 20.0 * self.ros_rate:
                if self.cross_times > 0:
                    self.user_control_reset()
                else:
                    return True
                rospy.loginfo("control reset!")

            # rospy.loginfo_throttle(0.2,self.parse_local_position())

            time.sleep(1.0 / self.ros_rate)
            


        elif self.ifend:  #np.linalg.norm(np.array([px, py])- pos_uav[0:2])< 0.5
            if self.tick==0:
                print("end point is occupied, howering")
                self.cross_times -= 1
                self.register()
            # if abs(uz-pz)<0.2:
            #     self.set_local_position(px, py, 0.3,0)
            # else:
            #     self.set_local_position(px, py, pz,0)#uY)
            # self.set_local_position(0,0,1.5,0)
            print("EPO,publish position command",self.tick)
            for ii in range(50):
                self.set_local_position(ux, uy, uz,uY)
                time.sleep(0.05)
            self.tick += 1
            
            
            if self.tick > 20.0 * self.ros_rate:
                if self.cross_times > 0:
                    self.user_control_reset()
                else:
                    return True
                rospy.loginfo("control reset!")

            time.sleep(1.0 / self.ros_rate)
        else:
            if time.time()-self.time_pub>1:
                self.publish_path(self.path_rec[::int(len(self.path_rec)/100)+1])
                self.time_pub=time.time()
            # dir_v = self.get_dir(self.traj, 1)  # the drone's speed
            self.my_state_mach = 1
            self.tick=0
            self.set_velocity(self.ifend_old)
        self.ifend_old = ifend or self.ifend
 
    def user_control_reset(self):
        rospy.loginfo("user program reset!")
        self.my_state_mach = 0
        # self.map_builder = Map_builder()


if __name__ == "__main__":
    prog = Program1()
    prog.run()


