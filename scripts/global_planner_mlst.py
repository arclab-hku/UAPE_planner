#!/usr/bin/env python2
import rospy
from nav_msgs.msg import OccupancyGrid,Path
# import tf
import numpy as np
import time
from geometry_msgs.msg import PoseStamped,Point,PointStamped
import jps1
import math
import os
from PIL import Image
import getpass
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2,PointField

class global_planner():
    def map_callback(self,data):
        print("map received")
        self.map_r1=data.info.height  #y
        self.map_c1=data.info.width   #x
        self.map= np.array(data.data).reshape(self.map_r1,self.map_c1).T
        self.map[np.where(self.map==100)]=1 # obstacle
        self.map[np.where(self.map==-1)]=0  #free
#        self.map = self.remove_zero_rowscols(self.map)
        # self.map=self.map.reshape(self.map_r,self.map_c).T
        self.map_o=[data.info.origin.position.x,data.info.origin.position.y]
        self.map_oo=[data.info.origin.position.x,data.info.origin.position.y]
        self.map_reso=data.info.resolution
        self.map_t=[self.map_o[0]+self.map_c1*self.map_reso,self.map_o[1]+self.map_r1*self.map_reso]
        self.origin=data.info.origin
        self.if_map_pub = 1
        self.if_map_empty = 0
        
    def goal_callback(self,goal):
        if goal.pose.position.z == 0: 
            self.global_goal = np.array([goal.pose.position.x, goal.pose.position.y, 1.0])
        else:
            self.global_goal = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])
        print('goal received!!')
    def remove_zero_rowscols(self,X,px,py):
        # X is a scipy sparse matrix. We want to remove all zero rows from it
        while self.if_map_pub != 1:
            time.sleep(0.02)
        if self.if_map_pub == 1:
            
            nonzero_row_indice, nonzero_col_indice = X.nonzero()
            u_row_indice = np.unique(nonzero_row_indice)
            u_col_indice = np.unique(nonzero_col_indice)
    #        print("non zero index:",u_row_indice,u_col_indice)
            if len(u_row_indice) !=0 :
                map_start0=np.rint((np.array([px,py])-self.map_o)/self.map_reso).astype(int)
                map_mat_o = [min(u_row_indice),min(u_col_indice)]
                self.map_c= max(u_row_indice) - min(map_mat_o[0],map_start0[0])  #y
                self.map_r= max(u_col_indice) - min(map_mat_o[1],map_start0[1])   #x
                print(max(u_col_indice), min(map_mat_o[1],map_start0[1]))
                self.map_o=np.array([min(map_mat_o[0],map_start0[0]),min(map_mat_o[1],map_start0[1])])*self.map_reso + self.map_o
    #            self.map_o=[max(self.map_o[0],px),max(self.map_o[0],py)]
                self.map_t=[self.map_o[0]+self.map_c*self.map_reso,self.map_o[1]+self.map_r*self.map_reso]
        #        self.origin=data.info.origin
                x_row=X[min(map_mat_o[0],map_start0[0]):max(u_row_indice)+1]
                self.if_map_pub = 0
                print("map row and col:",self.map_r,self.map_c,self.map_o)
                self.if_map_empty = 1
                return x_row[:,min(map_mat_o[1],map_start0[1]):max(u_col_indice)+1] #,u_col_indice
            else:
#                self.if_map_empty = 1
                return 0
    def thread_job():
        rospy.spin()
        
    
    def local_position_callback(self,data):
        self.pos=data
        # parse_local_position(data)
        
#    def octo_callback(self,data):
#        assert isinstance(data, Marker)
#        # global point2,pos,pub
##        self.octo_plc = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
#        self.octo_plc = []
#
#        for o_p in data.points:
#            self.octo_plc.append([o_p.x,o_p.y,o_p.z])
    def listener(self):
    
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('global_planner', anonymous=True)
        
#        self.if_map_empty = 0
#        self.octo_pub = rospy.Publisher('/octomap_point_cloud_centers_local', PointCloud2, queue_size=1)
        self.path_pub = rospy.Publisher('/jps_path', Path, queue_size=1)
        self.dir_path_pub = rospy.Publisher('/direct_jps_path', Path, queue_size=1)
        self.map_pub=rospy.Publisher('/global_2dmap', OccupancyGrid, queue_size=1)
        self.goalpub = rospy.Publisher('/goal_global', Point, queue_size=1)  #static points
        self.visualgoalpub = rospy.Publisher('/visual_goal_global', Marker, queue_size=1)
        # self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity_local',
        #                               TwistStamped,
        #                               self.velocity_callback,queue_size=1,buff_size=52428800)
        # self.octo_pub = rospy.Publisher('/octomap_point_cloud_centers_local', PointCloud2, queue_size=1)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                                  PoseStamped,
                                                  self.local_position_callback,queue_size=1,buff_size=52428800)
        self.globalmap_sub = rospy.Subscriber('/occupancy_map',
                                      OccupancyGrid,
                                      self.map_callback,queue_size=1)
        
        self.globalgoal_sub = rospy.Subscriber('/move_base_simple/goal',
                                    PoseStamped,
                                    self.goal_callback,queue_size=1)
#        self.octo_plc_sub = rospy.Subscriber('/localmap',
#                                          Marker,
#                                          self.octo_callback,queue_size=1,buff_size=52428800)
        # add_thread = threading.Thread(target = thread_job)
        # add_thread.start()
        # rospy.spin()
    
    def parse_local_position(self,local_position, mode="q"):
        # global rx,ry,rz,rpy_eular
        rx=local_position.pose.position.x
        ry=local_position.pose.position.y
        rz=local_position.pose.position.z
        
        qx=local_position.pose.orientation.x
        qy=local_position.pose.orientation.y
        qz=local_position.pose.orientation.z
        qw=local_position.pose.orientation.w
        # print(rx,ry,rz)
        if mode == "q":
            return (rx,ry,rz)
    
        # elif mode == "e":
        #     rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
        #     return (rx,ry,rz)+ rpy_eular
        else:
            raise UnboundLocalError

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
        
    def publish_dir_path(self, data,time_b):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        path.header.stamp.secs -= time_b
        path.header.stamp.secs = max(0,path.header.stamp.secs)
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)
        self.dir_path_pub.publish(path)
    def publish_map(self,data,map_ori):
        data[np.where(data==1)]=100
        data[np.where(data==-1)]=0
        mapg=OccupancyGrid()
        mapg.header.frame_id = "map"
        mapg.header.stamp = rospy.Time.now()
        mapg.info.resolution=self.map_reso
        mapg.info.width=len(data)
        mapg.info.height=len(data[0])
        mapg.info.origin=self.origin
        mapg.info.origin.position.x=map_ori[0]
        mapg.info.origin.position.y=map_ori[1]
        mapg.info.origin.position.z=0
        mapg.data=list(data.T.reshape(mapg.info.width*mapg.info.height,1)[:,0]) #T?
        self.map_pub.publish(mapg)
    def publish_goal(self,goal_p):
        goal = Marker()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.type = Marker.SPHERE
        # goal.pose.position.x = self.velo_goal[0]
        # goal.pose.position.y = self.velo_goal[1]
        # goal.pose.position.z = self.velo_goal[2]
        goal.pose.position.x = goal_p[0]
        goal.pose.position.y = goal_p[1]
        goal.pose.position.z = goal_p[2]
        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = 0
        goal.pose.orientation.w = 0
        goal.scale.x = 0.3
        goal.scale.y = 0.3
        goal.scale.z = 0.3
        goal.color.a = 1
        goal.color.r = 1
        goal.color.g = 1
        goal.color.b = 0.0
        self.visualgoalpub.publish(goal)
    def poly(self,x1,y1,x2,y2,x3,y3): #
        # a1 = -((y2-y3)*x1-(x2-x3)*y1+x2*x3-x3*y2)/((x2-x3)*(x1-x2)*(x1-x3))
        # b1 = ((y2-y3)*x1**2+x2**2*y3-x3**2*y2-(x2**2-x3**2)*y1) / ((x2-x3)*(x1-x2)*(x1-x2))
        # # c1 = ((x2*y3-x3*y2)*x1**2-(x2**2-x3**2)*x1+(x2**2*x3-x2*x3**2)*y1)/((x2-x1)*(x1-x2)*(x1-x3))
        # c1 = y1 - a1*x1**2 - b1*x1
        print("matrix",np.matrix([[x1**2,x1,1],[x2**2,x2,1],[x3**2,x3,1]]).T,"y:",[y1,y2,y3])
        abc = np.dot(np.matrix([y1,y2,y3]) , np.matrix([[x1**2,x1,1],[x2**2,x2,1],[x3**2,x3,1]]).T.I)
        
        return np.array(abc)[0]
    def map_line_col(self,p2,p1,mapu):
        blc_lt = np.where(mapu==1)
        if len(blc_lt[0])>0:
            p0 = np.array([min(p1[0],p2[0]),min(p1[1],p2[1])]).astype(float)
            
            p1=p1-p0
            p2=p2-p0
            if p2[0]<p1[0]:
                p3=p2.copy()
                p2=p1.copy()
                p1=p3
            # print(p1,p2,p0)
            line_cross = np.c_[np.arange(p1[0]+1,p2[0],1).astype(int),np.rint((p2[1]-p1[1])/(p2[0]-p1[0])*np.arange(1,p2[0]-p1[0],1)).astype(int)+int(p1[1])]
            
            # if 
            # for i range(len(blc_lt[0])):
            line_cross = line_cross.tolist()
            # print("desent",(p2[1]-p1[1])/(p2[0]-p1[0]),(p2[1]-p1[1])/(p2[0]-p1[0])*np.arange(p1[0]+1,p2[0],1))
            # print(line_cross,np.array(blc_lt).T.tolist())
            
            for lb in line_cross:
                # print(lb,np.array(blc_lt).T.tolist())
                if lb in np.array(blc_lt).T.tolist():
                    print("collision in map_line")
                    return False   # collide
        print("no collision in map_line")      
        return True   # if no collision, delete the middle point
            
    def cross_point(self,line1,line2): #calculate the cross point
        x1=line1[0,0]# select four points
        y1=line1[0,1]
        x2=line1[1,0]
        y2=line1[1,1]
        
        x3=line2[0,0]
        y3=line2[0,1]
        x4=line2[1,0]
        y4=line2[1,1]
        
        k1=(y2-y1)*1.0/(x2-x1)
        b1=y1*1.0-x1*k1*1.0
        if (x4-x3)==0:
            k2=None
            b2=0
        else:
            k2=(y4-y3)*1.0/(x4-x3)
            b2=y3*1.0-x3*k2*1.0
        if k2==None:
            x=x3
        else:
            x=(b2-b1)*1.0/(k1-k2)
        y=k1*x*1.0+b1*1.0
        #print('cross:',[x,y],'line1',line1,'line2',line2)
        if x4<x3:
            xs=x3
            x3=x4
            x4=xs
        if y4<y3:
            xs=y3
            y3=y4
            y4=xs
        if x2<x1:
            xs=x1
            x1=x2
            x2=xs
        if y2<y1:
            xs=y1
            y1=y2
            y2=xs
        tole = 0.05
        lb = 1-tole
        ub = 1+tole
        if lb*x3<=x<=ub*x4 and lb*y3<=y<=ub*y4 and lb*x1<=x<=ub*x2 and lb*y1<=y<=ub*y2:
            return np.array([x,y])
        else:
            return None     
if __name__ == '__main__':
    # global pub,rate,point2,pos
    time.sleep(3)
    planner=global_planner()
    planner.listener()
    planner.global_goal = None
    global_goal = None
    planner.if_map_pub = 1
    # convert.vel=None
    planner.pos=None
    planner.map=None
    planner.map_reso = None
    planner.map_o=None
    planner.octo_plc = None
    point_map = None
    path3=None
    path4=None
    wp=None
    map_o=None
    rosrate=80
    ifa=2
    rate = rospy.Rate(rosrate) # 
    ang_wp_tre=math.pi/4
    dis_wp_tre=2
    end_occu = 0
    last_jps_pos = np.array([0,0,0])
    last_jps_time = 0
    # global_goal_list=[]
    # global_goal_list=np.array([[15,0,1],[-12,-12,1],[0,12,1],[0,-12,1],[-16,-1,1]])
    ct=1 #iter time
    
    map_pre=None
    fname=None
    ii=0
    while not rospy.is_shutdown():
        mapu= 0
        
        pointw=Point()
        starttime1 = time.clock()

        # if ii<len(global_goal_list):
        #     global_goal=global_goal_list[ii]
        # print(pos)
#        print(planner.map)
        if planner.pos is not None and (planner.map_reso is not None):
            px,py,pz=planner.parse_local_position(planner.pos)
            if planner.global_goal is not None:
                
                if_goal_otr1 = (((planner.global_goal[0:2]-np.array(planner.map_oo))/planner.map_reso)-np.array([planner.map_c1,planner.map_r1]))>0
                if_goal_otr2 = planner.global_goal[0:2]-np.array(planner.map_oo) <0
                # print("123456",if_goal_otr1,if_goal_otr2,"map_o",np.array(planner.map_oo),((planner.global_goal[0:2]-np.array(planner.map_oo))/planner.map_reso),np.array([planner.map_c1,planner.map_r1]))
                if if_goal_otr1.any() or if_goal_otr2.any():
                    # if np.linalg.norm(np.array([px,py])-np.array(planner.map_oo)-np.array([planner.map_c1,planner.map_r1])*0.5*planner.map_reso)<0.2:
                    # print("111222")
                    rel_global_goal = planner.global_goal[0:2]-np.array([px,py])
                    line2goal=np.array([np.array([px,py])-np.array(planner.map_oo),planner.global_goal[0:2]-np.array(planner.map_oo)])
                    sqr_x = planner.map_c1*planner.map_reso
                    sqr_y = planner.map_r1*planner.map_reso
                    # if rel_global_goal[0]<rel_global_goal[1] and -rel_global_goal[0]<rel_global_goal[1]: #cross at top
                    #     global_goal = planner.cross_point(line2goal,np.array([[0,sqr_y],[sqr_x,sqr_y]]))
                    # elif rel_global_goal[0]>rel_global_goal[1] and -rel_global_goal[0]<rel_global_goal[1]:
                    #     global_goal = planner.cross_point(line2goal,np.array([[sqr_x,0],[sqr_x,sqr_y]]))
                    # elif rel_global_goal[0]>rel_global_goal[1] and -rel_global_goal[0]>rel_global_goal[1]:
                    #     global_goal = planner.cross_point(line2goal,np.array([[0,0],[sqr_x,0]]))
                    # else:
                    #     global_goal = planner.cross_point(line2goal,np.array([[0,0],[0,sqr_y]]))
                    global_goal = planner.cross_point(line2goal,np.array([[0,sqr_y],[sqr_x,sqr_y]]))
                    if global_goal is None:
                        global_goal = planner.cross_point(line2goal,np.array([[sqr_x,0],[sqr_x,sqr_y]]))
                        if global_goal is None:
                            global_goal = planner.cross_point(line2goal,np.array([[0,0],[sqr_x,0]]))
                            if global_goal is None:
                                global_goal = planner.cross_point(line2goal,np.array([[0,0],[0,sqr_y]]))
                    print("global_goal",global_goal)
                    if global_goal is not None:
                        global_goal = global_goal + np.array(planner.map_oo)
                        global_goal = np.array([global_goal[0],global_goal[1],planner.global_goal[2]])
                    else:
                        global_goal = planner.global_goal.copy()
                else:
                    global_goal = planner.global_goal.copy()
            # else:
            #     global_goal = np.array([px,py,1])
            map_reso=planner.map_reso
            mapu = planner.remove_zero_rowscols(planner.map ,px,py)
#            print("removed empty,mapu",mapu)
        
        if (mapu is not 0) and (planner.map_c*planner.map_r >0) and (global_goal is not None) and (planner.pos is not None) and (planner.map_reso is not None) and (planner.map_c1>2*ifa) and (planner.map_o is not None):#and (convert.vel is not None)

            map_c=planner.map_c
            map_r=planner.map_r
            map_o=planner.map_o
            
            ori_pre=[-15,-15]  #the pre-settled origin coordinates
            if ct==1:
                xo,yo,zo=planner.parse_local_position(planner.pos)
                try:
                    img=Image.open("/home/chen/catkin_ws/src/path_plan/maps/map1.png")
                    white,black=0,1
                    
                    img = img.convert('L')
                    img = img.point(lambda x: white if x > 200 else black)
                    img = np.array(img)
                    map_pre=img[::-1].T # pre-known map before fly
                    
                    
                    l1_pre=len(map_pre)
                    l2_pre=len(map_pre[0])
                    t_pre=[ori_pre[0]+planner.map_reso*l1_pre,ori_pre[1]+planner.map_reso*l2_pre]
                except:
                    map_pre=None
            ct+=1
            

            if map_pre is not None: #merge the prepared map and the detected map
                # print(map_o,ori_pre)
                map_o1=[min(map_o[0],ori_pre[0]),min(map_o[1],ori_pre[1])]  #oringin of the new map
                map_o=((np.array(map_o)-map_o1)/planner.map_reso).astype(int) #index of the origin of the detected map
                ori_pre=((np.array(ori_pre)-map_o1)/planner.map_reso).astype(int) #index of the origin of the prepared map
                map_c1=int((max(t_pre[0],planner.map_t[0])-map_o1[0])/planner.map_reso)
                map_r1=int((max(t_pre[1],planner.map_t[1])-map_o1[1])/planner.map_reso)
                mapu0=np.zeros([map_c1,map_r1])
                # print('t_pre,planner.map_t,map_o1,map_c1,map_r1,ori_pre,l1_pre,l2_pre',t_pre,planner.map_t,map_o1,map_c1,map_r1,ori_pre,l1_pre,l2_pre)
                mapu0[ori_pre[0]:ori_pre[0]+l1_pre,ori_pre[1]:ori_pre[1]+l2_pre]=map_pre
                mapu0[map_o[0]:map_o[0]+map_c,map_o[1]:map_o[1]+map_r]=mapu
                mapu=mapu0
                map_o=map_o1
                map_c=map_c1
                map_r=map_r1
                # print('mapu:',mapu)
            map_goal=np.rint((global_goal[0:2]-map_o)/map_reso).astype(int)
            map_start=np.rint((np.array([px,py])-map_o)/map_reso).astype(int)
            map_goal0=map_goal.copy()
            # map_d=np.array([0,0])
            m_ifa=max(1,ifa)
            map_o2=np.array([-2*m_ifa,-2*m_ifa])
            if map_goal[0]<0 or map_start[0]<0:
                map_o2[0]=min(map_goal[0],map_start[0])+map_o2[0]
            if map_goal[1]<0 or map_start[1]<0:
                map_o2[1]=min(map_goal[1],map_start[1])+map_o2[1]# map_o2 : the pixel replacement of the new origin against the old origin
            map_d=abs(map_o2)
            map_o=list((map_o2)*map_reso+np.array(map_o))

            map_c=max(map_c,map_goal[0],map_start[0])+map_d[0]
            map_r=max(map_r,map_goal[1],map_start[1])+map_d[1]
            mapu0=np.zeros([map_c+4*m_ifa,map_r+4*m_ifa])
            if len(mapu):
                mapu0[map_d[0]:len(mapu)+map_d[0],map_d[1]:len(mapu[0])+map_d[1]]=mapu
            mapu=mapu0
            if ifa:
                mapu_occu_list = np.where(mapu>0)
                for i in range(-ifa,ifa+1,1):
                    for j in range(-ifa,ifa+1,1):
                        mapu[(mapu_occu_list[0]+i,mapu_occu_list[1]+j)] = 1
            # mapu=mapc
            
            
            map_start = map_start+map_d
            map_goal = map_goal+map_d
            if mapu[map_goal[0],map_goal[1]]==1:  #when an obstacle locates at the goal
                try:
                    map_goal[1]=np.where(mapu[map_goal[0],:]==0)[0][np.argmin(abs(np.where(mapu[map_goal[0],:]==0)-map_goal[1]))]
                except:
                    map_goal[0]=np.where(mapu[:,map_goal[1]]==0)[0][np.argmin(abs(np.where(mapu[:,map_goal[1]]==0)-map_goal[0]))]
                

            if mapu[map_goal[0],map_goal[1]]==1: #(mapu[map_goal[0]-1:map_goal[0]+1,map_goal[1]-1:map_goal[1]+1]==1).any():
                end_occu = 1
            else:
                end_occu = 0
            

            if (map_start)[0]>map_c or (map_start)[1]>map_r:
                wp=global_goal
                # path4 = 
                planner.publish_dir_path(np.array([[px,py,pz],wp]),100)
                # path4 = None
            elif (last_jps_pos[0] == 0 or np.linalg.norm(last_jps_pos-np.array([px,py,pz]))>0.2 or time.time()-last_jps_time>0.3) :
                path1 = jps1.method(mapu, tuple(map_start), tuple(map_goal), 2)
                px,py,pz=planner.parse_local_position(planner.pos)
                last_jps_pos = np.array([px,py,pz])
                last_jps_time = time.time()
                if path1[0] is 0:
                    # print(mapu,map_start,map_goal,map_o,path1) #
                    print('no path')
                    wp=global_goal
                    planner.publish_dir_path(np.array([[px,py,pz],wp]),100)
                else:
                    path2 = np.array(path1[0])+np.array([0,0])
                    path2_c = path2.copy()-np.array([0,0])
                    path3=path2*map_reso+map_o
                    # if end_occu == 1:
                    #     path3 = path3[::-1]
                    # path3 = np.r_[path3,[global_goal[0:2]]]
                    # print('path3',path3)
                    path3=np.c_[path3,np.zeros([len(path3),1])]
                    path4 = path3.copy()
                    ang_wp=0
        
                    del_path = []
                    if len(path4) >2:
                        for ii in range(1,len(path3)):
                            if np.linalg.norm(path3[ii]-np.array([px,py,pz])) < 1.5:
                                del_path.append(ii)
                        path4 = np.delete(path4, del_path ,axis =0)
                        path2_c  = np.delete(path2_c, del_path ,axis =0)

                    ii=1
                    while ii<len(path2_c)-1:
                        if planner.map_line_col(path2_c[ii+1],path2_c[ii-1],mapu[min(path2_c[ii-1][0],path2_c[ii+1][0]):max(path2_c[ii-1][0],path2_c[ii+1][0])+1,min(path2_c[ii-1][1],path2_c[ii+1][1]):max(path2_c[ii-1][1],path2_c[ii+1][1])+1]):
                            path4 = np.delete(path4, ii ,axis =0)
                            path2_c  = np.delete(path2_c, ii ,axis =0)
                        else:
                            ii+=1
                            
                    if len(path4) >2:
                        # rati = min(1,np.linalg.norm(path4[1,0:2]-last_jps_pos[0:2])/np.linalg.norm(path4[2,0:2]-last_jps_pos[0:2]))**0.8
                        
                        # wp = (path4[1]*(2-rati) + path4[2]*rati)/2
                        path_seg1=np.linalg.norm(path4[1,0:2]-last_jps_pos[0:2])
                        if path_seg1>3:
                            rati=1
                        else:
                            rati = min(1,path_seg1/(path_seg1+np.linalg.norm(path4[2,0:2]-path4[1,0:2])))**0.7
                        wp = path4[1]*rati + path4[2]*(1-rati)
                    else:
                        wp = global_goal

                    uav2next_wp=np.linalg.norm(wp[0:2]-np.array([px,py]))
                    if_goal=uav2next_wp+ang_wp
                    if end_occu == 1:
                        print("end point is occupied, ")
                        wp = np.array([px,py,pz])
                        global_goal = wp

            px,py,pz=planner.parse_local_position(planner.pos)
            if np.linalg.norm(global_goal[0:2]-np.array([px,py]))<0.5 or end_occu:
                pointw.z=0.1
            else:
                pointw.z=1+min(np.linalg.norm(wp[0:2]-np.array([xo,yo]))/np.linalg.norm(global_goal[0:2]-np.array([xo,yo])),1)*(global_goal[2]-1)
            print('position:',[px,py,pz])

        else:
            if planner.pos is not None:
                print("map is not prepared,global goal:",planner.global_goal)
                px,py,pz=planner.parse_local_position(planner.pos)
                if planner.global_goal is not None:
                    global_goal = planner.global_goal.copy()

                if global_goal is not None:
                    wp=global_goal
                    if np.linalg.norm(global_goal[0:2]-np.array([px,py]))<0.5:
                        pointw.z=0.1
                    else:
                        pointw.z = 1
        protime=time.clock()-starttime1
        # time.sleep(0.05)
        if wp is not None :
            # print(point2)
            
            pointw.x=wp[0]
            pointw.y=wp[1]
            
            planner.goalpub.publish(pointw)
            planner.publish_goal([pointw.x,pointw.y,pointw.z])
            if path3 is not None:
                planner.publish_path(path3)
            if path4 is not None:
                planner.publish_dir_path(path4,0)
            if (map_o is not None) and( mapu is not 0):
                planner.publish_map(mapu,map_o)
            
            time.sleep(0.05)

        if ct%rosrate==0 and (mapu is not None):
            if fname is not None:
                os.remove(fname)
            mapsave=mapu.copy()
            mapsave[np.where(mapu!=0)]=0
            mapsave[np.where(mapu==0)]=255
            mapsave=mapsave.T[::-1]
            im = Image.fromarray(np.uint8(mapsave)).convert('RGB')
            fname="/home/"+ getpass.getuser() +"/catkin_ws/src/fuxi-planner/maps/"+'%.2f'%map_o[0]+'%.2f'%map_o[1]+"_out.png"
            im.save(fname)
        print('next goal:',pointw,wp,global_goal)
        print('global_planner protime:',protime)
