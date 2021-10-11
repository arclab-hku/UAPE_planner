#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs import point_cloud2
import tf
from geometry_msgs.msg import PoseStamped,TwistStamped,Point
import numpy as np
from utils import earth_to_body_frame,body_to_earth_frame
import threading
import time
import sklearn.cluster as skc
from visualization_msgs.msg import Marker,MarkerArray
import math,copy
from message_filters import TimeSynchronizer, Subscriber,ApproximateTimeSynchronizer
class convert_plc():
    def callback(self,data):
        #y.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        assert isinstance(data, PointCloud2)
        # global point2,pos,pub
        self.plc = [list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)),list(point_cloud2.read_points(data, field_names=("rgb")))]
        self.plc_time =data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
#        self.plc_rgb= 
#        self.plc = list(point_cloud2.read_points(data, field_names=("x", "y", "z","rgb"), skip_nans=True))
#        print("plc",self.plc)
#        print("plc_rgb",self.plc_rgb)
            # print(point2)
        # print(type(gen))
        # for p in gen:
        #   print(" x : %.3f  y: %.3f  z: %.3f" %(p[2],p[0],-p[1]))
        #   print(gen[1])
        
    def pos_pcl(self,pcl,pos,vel):
        # self.pos=pos.pose
        self.pos=pos
        self.pos_time = pos.header.stamp.secs + pos.header.stamp.nsecs * 1e-9
#        assert isinstance(pcl, PointCloud2)
        # global point2,pos,pub
        self.plc = [list(point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True)),list(point_cloud2.read_points(pcl, field_names=("rgb")))]
        self.plc_time =pcl.header.stamp.secs + pcl.header.stamp.nsecs * 1e-9
        self.plc_timestamp = pcl.header.stamp
        self.ang_vel = np.array([vel.twist.angular.x,vel.twist.angular.y,vel.twist.angular.z])
        self.line_vel = np.array([vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z])
        self.vel_time = vel.header.stamp.secs + vel.header.stamp.nsecs * 1e-9
        print("alighed",self.vel_time,self.plc_time)
        self.if_align = 1
    
    # def talker(point2):
    
    #     # while not rospy.is_shutdown():
    #     pub.publish(hello_str)
    #     rate.sleep()      
    def thread_job():
        rospy.spin()
        
    def local_position_callback(self,data):
        self.pos=data
        # parse_local_position(data)
        
    def velocity_callback(self, data):
        self.ang_vel = np.array([data.twist.angular.x,data.twist.angular.y,data.twist.angular.z])
        self.line_vel = np.array([data.twist.linear.x,data.twist.linear.y,data.twist.linear.z])
    def octo_callback(self,data):
        assert isinstance(data, PointCloud2)
        # global point2,pos,pub
        self.octo_plc = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    def listener(self):
    
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        self.if_align = 0
        rospy.init_node('point_transfrer', anonymous=True)
        self.pub = rospy.Publisher('/points_global', PointCloud2, queue_size=1)  #static points
#        self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity_local',
#                                       TwistStamped,
#                                       self.velocity_callback,queue_size=1,buff_size=52428800)
#        self.local_vel_sub1 = rospy.Subscriber('mavros/local_position/velocity',
#                                       TwistStamped,
#                                       self.velocity_callback,queue_size=1,buff_size=52428800)
        self.alpub = rospy.Publisher('/points_global_all', PointCloud2, queue_size=1)    #all points
        self.octo_pub = rospy.Publisher('/octomap_point_cloud_centers_local', PointCloud2, queue_size=1)
#        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
#                                                  PoseStamped,
#                                                  self.local_position_callback,queue_size=1)
#        self.plc_sub = rospy.Subscriber('/filtered_RadiusOutlierRemoval',
#                                      PointCloud2,
#                                      self.callback,queue_size=1,buff_size=52428800) #/filtered_RadiusOutlierRemoval',
        self.octo_plc_sub = rospy.Subscriber('/octomap_point_cloud_centers',
                                          PointCloud2,
                                          self.octo_callback,queue_size=1,buff_size=52428800) #/octomap_point_cloud_centers
        # self.dynobs_publisher = rospy.Publisher("dyn_obs", Marker, queue_size=1)
        # self.dynv_publisher = rospy.Publisher("dyn_v", Marker, queue_size=1)
        self.dyn_publisher = rospy.Publisher("dyn", MarkerArray, queue_size=1)
        self.potential_dyn_publisher = rospy.Publisher("potential_dyn", MarkerArray, queue_size=1)
        self.tss = ApproximateTimeSynchronizer([
                                                Subscriber('/filtered_RadiusOutlierRemoval',PointCloud2),
                                                # Subscriber('/gt_iris_base_link_imu', Odometry),
                                                Subscriber('mavros/local_position/pose', PoseStamped),
                                                Subscriber('mavros/local_position/velocity_local',TwistStamped)],
        6,0.02, allow_headerless=True)
        self.tss.registerCallback(self.pos_pcl)
        # add_thread = threading.Thread(target = thread_job)
        # add_thread.start()
        # rospy.spin()
    
    def parse_local_position(self,local_position, mode="e"):
        # global rx,ry,rz,rpy_eular
        rx=local_position.pose.position.x
        ry=local_position.pose.position.y
        rz=local_position.pose.position.z
        
        qx=local_position.pose.orientation.x
        qy=local_position.pose.orientation.y
        qz=local_position.pose.orientation.z
        qw=local_position.pose.orientation.w
        
#        print(time)
        # print(rx,ry,rz)
        if mode == "q":
            return (rx,ry,rz,qx,qy,qz,qw)
    
        elif mode == "e":
            rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
            return (rx,ry,rz)+ rpy_eular
        else:
            raise UnboundLocalError
            
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
    def distance_filter(self,plc,dis):
        # filtered_plc=[]
        # for point in plc:
        #     point=list(point)
        #     d_point=np.linalg.norm(point)
        #     if d_point<dis:
        #         filtered_plc.append(point+[d_point])
        # filtered_plc=np.array(filtered_plc)
        
        d_point=np.linalg.norm(plc,axis=1)
        filtered_plc=np.c_[plc,d_point]
        filtered_plc=filtered_plc[d_point<dis]
        if len(filtered_plc)>0:
            filtered_plc=filtered_plc[np.lexsort(filtered_plc.T)]
            # filtered_plc=filtered_plc[:,0:3]
        return filtered_plc[:,0:3]

    # def parse_velocity(self,vel):
    #     wx=vel.twist.angular.x
    #     wy=vel.twist.angular.y
    #     wz=vel.twist.angular.z
    #     vx=vel.twist.linear.x
    #     vy=vel.twist.linear.y
    #     vz=vel.twist.linear.z
    #     return [vx,vy,vz,wx,wy,wz]
    def publish_dyn_obs(self,c_dyn1,v_dyn1,obsd,pos):
        dyn=MarkerArray()
        
        for m in range(len(c_dyn1)):
#            if np.linalg.norm(np.array(pos[0:3])-c_dyn1[m])>8 :#or abs(math.atan2(-pos[1]+c_dyn1[m][1],-pos[0]+c_dyn1[m][0])-pos[3])>math.pi/3:
#                break
            dynobs = Marker()
            dynobs.header.frame_id = "map"
            dynobs.header.stamp = rospy.Time.now()
            dynobs.type = Marker.SPHERE
            dynobs.pose.position.x = c_dyn1[m][0]
            dynobs.pose.position.y = c_dyn1[m][1]
            dynobs.pose.position.z = c_dyn1[m][2]
            dynobs.id=2*m
            dynobs.scale.x = obsd[m]
            dynobs.scale.y = obsd[m]
            dynobs.scale.z = obsd[m]
            dynobs.color.a = 0.5
            dynobs.color.r = 0.1
            dynobs.color.g = 0.8
            dynobs.color.b = 0.1
            
            dynobs.pose.orientation.x = 0
            dynobs.pose.orientation.y = 0
            dynobs.pose.orientation.z = 0
            dynobs.pose.orientation.w = 0
            
            dynv = Marker()
            p1=Point()
            p2=Point()
            dynv.header.frame_id = "map"
            dynv.header.stamp = rospy.Time.now()
            dynv.type = Marker.ARROW
            dynv.id=2*m+1
            p1.x,p1.y,p1.z=c_dyn1[m][0],c_dyn1[m][1],c_dyn1[m][2]
            # p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0]+np.sign(v_dyn1[m][0])*obsd[m]/2,c_dyn1[m][1]+v_dyn1[m][1]+np.sign(v_dyn1[m][1])*obsd[m]/2,v_dyn1[m][2]+c_dyn1[m][2]+np.sign(v_dyn1[m][2])*obsd[m]/2
            p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0],c_dyn1[m][1]+v_dyn1[m][1],v_dyn1[m][2]+c_dyn1[m][2]
            # dynv.points.push_back(p1)
            # dynv.points.push_back(p2)
            dynv.points.append(p1)
            dynv.points.append(p2)
            # dynv.pose.position.x = self.velo_goal[0]
            # dynv.pose.position.y = self.velo_goal[1]
            # dynv.pose.position.z = self.velo_goal[2]
    
            dynv.scale.x = 0.2 #diameter of arrow shaft
            dynv.scale.y = 0.4 #diameter of arrow head
            dynv.scale.z = 0.6
            dynv.color.a = 1  #transparency
            dynv.color.r = 0.8
            dynv.color.g = 0.1
            dynv.color.b = 0.1
            
            
            dyn.markers.append(dynobs)
            dyn.markers.append(dynv)
    
        # dynv.pose.orientation.x = 0
        # dynv.pose.orientation.y = 0
        # dynv.pose.orientation.z = 0
        # dynv.pose.orientation.w = 0
        # Publish the MarkerArray
        # self.dynobs_publisher.publish(dynobs)
        # self.dynv_publisher.publish(dynv)
        self.dyn_publisher.publish(dyn)
    def publish_potential_dyn_obs(self,c_dyn1,v_dyn1,obsd):
        dyn=MarkerArray()
        
        for m in range(len(c_dyn1)):
            dynobs = Marker()
            dynobs.header.frame_id = "map"
            dynobs.header.stamp = rospy.Time.now()
            dynobs.type = Marker.SPHERE
            dynobs.pose.position.x = c_dyn1[m][0]
            dynobs.pose.position.y = c_dyn1[m][1]
            dynobs.pose.position.z = c_dyn1[m][2]
            dynobs.id=2*m
            dynobs.scale.x = obsd[m]
            dynobs.scale.y = obsd[m]
            dynobs.scale.z = obsd[m]
            dynobs.color.a = 0.5
            dynobs.color.r = 0.6
            dynobs.color.g = 0.1
            dynobs.color.b = 0.6
            
            dynobs.pose.orientation.x = 0
            dynobs.pose.orientation.y = 0
            dynobs.pose.orientation.z = 0
            dynobs.pose.orientation.w = 0
            
            dynv = Marker()
            p1=Point()
            p2=Point()
            dynv.header.frame_id = "map"
            dynv.header.stamp = rospy.Time.now()
            dynv.type = Marker.ARROW
            dynv.id=2*m+1
            p1.x,p1.y,p1.z=c_dyn1[m][0],c_dyn1[m][1],c_dyn1[m][2]
            # p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0]+np.sign(v_dyn1[m][0])*obsd[m]/2,c_dyn1[m][1]+v_dyn1[m][1]+np.sign(v_dyn1[m][1])*obsd[m]/2,v_dyn1[m][2]+c_dyn1[m][2]+np.sign(v_dyn1[m][2])*obsd[m]/2
            p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0],c_dyn1[m][1]+v_dyn1[m][1],v_dyn1[m][2]+c_dyn1[m][2]
            # dynv.points.push_back(p1)
            # dynv.points.push_back(p2)
            dynv.points.append(p1)
            dynv.points.append(p2)
            # dynv.pose.position.x = self.velo_goal[0]
            # dynv.pose.position.y = self.velo_goal[1]
            # dynv.pose.position.z = self.velo_goal[2]
    
            dynv.scale.x = 0.1 #diameter of arrow shaft
            dynv.scale.y = 0.2 #diameter of arrow head
            dynv.scale.z = 0.6
            dynv.color.a = 1  #transparency
            dynv.color.r = 0.5
            dynv.color.g = 0.5
            dynv.color.b = 0.1
            
            
            dyn.markers.append(dynobs)
            dyn.markers.append(dynv)
    
        # dynv.pose.orientation.x = 0
        # dynv.pose.orientation.y = 0
        # dynv.pose.orientation.z = 0
        # dynv.pose.orientation.w = 0
        # Publish the MarkerArray
        # self.dynobs_publisher.publish(dynobs)
        # self.dynv_publisher.publish(dynv)
        self.potential_dyn_publisher.publish(dyn)
    def ang_cos(self,c,b,a): #c-> the angle we need
        cb=np.linalg.norm(b-c)
        ca=np.linalg.norm(a-c)
        ab=np.linalg.norm(a-b)
        ang_cc=(cb**2+ca**2-ab**2)/(2*cb*ca)
        return ang_cc
    def correct_c_dyn(self,c_dyn,p_dyn,local_pos): #modify the center of the detected obstacle
        c_orin=np.array(c_dyn)
        p_edge=p_dyn[np.argmax(np.linalg.norm(np.array(p_dyn)-c_orin,axis=1))]
        L=np.linalg.norm(np.array(p_edge)-local_pos)
        alpha=math.acos(self.ang_cos(local_pos, p_edge, c_orin))
        r_obs=L*alpha
        beta=math.pi/2-alpha
        OG=math.sin(beta)/beta*r_obs
        AG=np.linalg.norm(c_orin-local_pos)
        AO=OG+AG
        c_c_dyn=((c_orin-local_pos)*AO/AG*2+(c_orin-local_pos)*1)/3+local_pos
        # print("AO,AG!!!!!!!!!!!",AO,AG)
        return c_c_dyn,r_obs
    
    def f_vector(self,c_dyn,r_obs,p_dyn,rgb_dyn):  # return the feature vector
        num=len(p_dyn)
        vd=(4/3*math.pi*r_obs**3)
        va=np.var(p_dyn)
        rgb=np.mean(rgb_dyn)*1e39
        rgb_va=np.var(rgb_dyn*1e39)
        return [c_dyn[0],c_dyn[1],1*c_dyn[2],2*vd,0.5*va,0.7*num,25*rgb,15*rgb_va]
#        return [c_dyn[0],c_dyn[1],c_dyn[2],num,dens]
        
if __name__ == '__main__':
    # global pub,rate,point2,pos

    convert=convert_plc()
    convert.listener()
    # convert.vel=None
    convert.pos=None
    convert.octo_plc=None
    convert.plc=None
    convert.f=None
    convert.ang_vel=None
    rosrate=100
    point22=None
    c_dyn11=None
    dyn_v11=None
    local_pos=0
    octo_plc=[]
    r0=0
    p0=0
    y0=0
    map_reso=0.2
    fac_dv=1.0  #dynamic obstacle velocity factor
    rate = rospy.Rate(rosrate) # 10hz
    dt=0.25
    n_p=6
    xb,yb,zb = 0.12,0,0.00
    plc_h=[]
    plc_h_l=[]
    rgb_h=[]
    t_plc=[]
    t_plc_l=[]
    pos_h=[]
    t_dh=[]     #time for the moving obj found
    f_dyn=[]
    c_dyn=[]    #center of the moving object
    dyn_v=[]    #speed of the moving object
    dyn_d=[]
    p_dyn=[]
#    c_dyn0=[]
#    dyn_v0=[]
#    d_dyn0=[]
    c_dynkf=[]
    v_dynkf=[]
    octo_plc1=[]
    time22=0
    time_cap=0
    pre_ct=0
    dyn_obs_reset=1
    c_dyn_l,v_dyn_l=[],[]
    plc_sta,plc_sta_h=[],[]
    x_mat=[]
    
    
    while not rospy.is_shutdown():
        starttime1 = time.clock()
        plc1=[]
        clu_p1=[]
        clu_p2=[]
        clu_c1=[]
        clu_c2=[]
        clu_c3=[] #ocro_map point cluster center
        rgb_p1=[]
        rgb_p2=[]
        obsd=[]
        c_dyn1=[]    #center of the moving object,remove in rviz simulation
        dyn_v1=[]    #speed of the moving object
        obsd2=[]
        dyn_v2=[]
        c_dyn2=[]    
        not_dyn_pt = []
        fea_1=[]   #feature array
        fea_2=[]
        if_inherit = 0
        if_stah_clear=0
        plc_c1,plc_c2 = [],[]
        time_itv_if = 0
        # print(pos)
        if convert.pos is not None and (convert.octo_plc is not None) and convert.if_align == 1:
            
            convert.if_align = 0
            px,py,pz,r,p,y=convert.parse_local_position(convert.pos)
            
            local_pos1=np.array([px,py,pz])
            octo_plc=np.array(convert.octo_plc)
            local_pos1 = (convert.plc_time - convert.pos_time)*convert.line_vel + local_pos1
            # print('num-obs',len(octo_plc))
            if len(octo_plc)>0:
                octo_plc1=octo_plc-local_pos1
#                octo_plc=octo_plc[(octo_plc<3).all(axis=1)]
                octo_plc1=convert.distance_filter(octo_plc1,4)+local_pos1
            
                point22=convert.xyz_array_to_pointcloud2(octo_plc1,'map',rospy.Time.now())
            
        if convert.pos is not None and (convert.plc is not None) and (convert.ang_vel is not None): #and len(convert.plc[0])>n_p  and convert.if_align == 1
            convert.if_align = 0
            px,py,pz,r,p,y=convert.parse_local_position(convert.pos)
            # r,p,y = np.array([r,p,y])+(convert.plc_time - convert.pos_time)*convert.ang_vel
            b2e = body_to_earth_frame(r,p,y)
            
            convert.plc_pt=convert.plc[0]
            convert.plc_rgb=convert.plc[1]
            len_plc_pt=len(convert.plc_pt)
            len_plc_rgb=len(convert.plc_rgb)
            # print("number of point and rgb:",len_plc_pt,len_plc_rgb)
            plc_rgb=np.array(convert.plc_rgb)
            

#            length=len(convert.plc)
            plc=np.array(convert.plc_pt)
            plc_c=plc.copy()
#            print(plc)
#            if abs(wr)<0.3 and abs(wp)<0.3 and abs(wy)<1.5 and abs(p) < math.pi/6 and px*py!=0:#and wr+wp+wy!=0:  #when UAV is not flat and stable don't use the pointcloud
            if (abs(convert.ang_vel[0:2])<3).all() and abs(p) < math.pi/3 and abs(r) < math.pi/3 and len(plc)>0:
                if local_pos is 0 or ((local_pos is not 0) and np.linalg.norm(local_pos-np.array([px,py,pz]))<0.3):
                    plc_c[:,0]=plc[:,2]+xb  #distance from camera to uav center, xb yb zb are offset
                    plc_c[:,1]=-plc[:,0]+yb
                    plc_c[:,2]=-plc[:,1]+zb
    #                plc_c=np.array(plc_c)
                    local_pos1=np.array([px,py,pz])
                    local_pos1 = (convert.plc_time - convert.pos_time)*convert.line_vel + local_pos1
                    plc_c=np.matmul(b2e,plc_c.T).T+np.tile(local_pos1,(len(plc_c),1))  # convert coordinate from B to E
                else:
                    plc_c=[]
            elif len(plc)>0:
                print('Rotating too fast!!!',convert.ang_vel)
                plc_c=[]
                # continue
            else:
                print('empty pcl!!')
                plc_c=[]
#            plc1=np.array(plc1)
            local_pos=np.array([px,py,pz])
            # local_pos = (convert.plc_time - convert.pos_time)*convert.line_vel + local_pos
            # print("pcl-pos time diff:",convert.plc_time - convert.pos_time)
            if len(plc_c)>0:
                plc1=plc_c[plc_c[:,2]>0.3]
                if len_plc_pt==len_plc_rgb:
                    plc_rgb=plc_rgb[plc_c[:,2]>0.3]
            #if (rospy.get_rostime().secs + rospy.get_rostime().nsecs*1e-9) % 10 < 0.1 :
           #     if if_stah_clear ==0:
            #        plc_sta_h = []
           #         if_stah_clear = 1
          #  else:
          #      if_stah_clear = 0
            if len(plc_sta) > 200:
                plc_sta_h = plc_sta
            elif len(plc_sta_h) + len(plc_sta) > 200:
                plc_sta_h = plc_sta_h[-min(-len(plc_sta)+200,len(plc_sta_h))::] + plc_sta
            else:
                plc_sta_h = plc_sta_h + plc_sta
            plc_sta=[]  #static points
            if len(plc1)>n_p and len_plc_pt==len_plc_rgb and (len(t_plc) == 0 or convert.plc_time != t_plc[-1]):
                if len(t_plc)>1 and t_plc[-1]-t_plc[-2]>10:
                    plc_h=[]
                    plc_h_l = []
                    t_plc=[]
                    t_plc_l=[]
                    pos_h=[]
                    rgb_h=[]
                    dyn_obs_reset=1
                    c_dynkf=[]
                    v_dynkf=[]
                    obsdkf=[]
                    print('refresh!!!')
                plc_h.append(plc1)
                
                rgb_h.append(plc_rgb)
                pos_h.append(local_pos)
                t_plc.append(convert.plc_time)
            # else:
            #     continue


#            dyn_v1=[]
#            c_dyn1=[]
            # print("t_plc[-1]-t_plc[0]",t_plc[-1]-t_plc[0])
            if len(t_plc)>1 and len_plc_pt==len_plc_rgb and t_plc[-1]-t_plc[0]>=dt:
                for j in range(0,len(t_plc)-1):
                    # try:
                    #     asdas=t_plc[i]
                    # except:
                    #     print("i",i,t_plc)
                    if j<len(t_plc) and t_plc[-1]-t_plc[j]>=dt and t_plc[-1]-t_plc[j+1]<dt:
                        print('pos est time intevel:',t_plc[-1]-t_plc[j])
                        time_itv_if = 1
                        break
                    print("j",j)
            # else:
            #     continue


             #   plc_c2=np.r_[plc_c2,plc_h[-2]]
             #   plc_c1=np.r_[plc_c1,plc_h[j-1]]
             #   rgb_c2=np.r_[rgb_c2,rgb_h[-2]]
             #   rgb_c1=np.r_[rgb_c1,rgb_h[j-1]]
            if time_itv_if ==1:
                plc_c2=plc_h[-1]
                plc_c1=plc_h[j]
                rgb_c2=rgb_h[-1]
                rgb_c1=rgb_h[j]
                
                # plc_c2=np.r_[plc_h[-1],plc_h[-2]]
                # plc_c1=np.r_[plc_h[j],plc_h[j-1]]
                # rgb_c2=np.r_[rgb_h[-1],rgb_h[-2]]
                # rgb_c1=np.r_[rgb_h[j],rgb_h[j-1]]
                # t_c2=(t_plc[-1]+t_plc[-2])/2
                # t_c1=(t_plc[j-1]+t_plc[j])/2
                # pos_c2=(pos_h[-1]+pos_h[-2])/2
                # pos_c1=(pos_h[j-1]+pos_h[j])/2
                
                t_c2=t_plc[-1]
                t_c1=t_plc[j]
                pos_c2=pos_h[-1]
                pos_c1=pos_h[j]
                
                plc_h=plc_h[j+1::]
                rgb_h=rgb_h[j+1::]
                t_plc=t_plc[j+1::]
                pos_h=pos_h[j+1::]
#                            plc_c2=plc_c2.tolist()
#                            for pp in plc_c2:
#                                if np.linalg.norm(pp-pos_c1)>8:
#                                    plc_c2.remove(pp)
                index_c1=(np.linalg.norm(plc_c1-pos_c1,axis=1)<5)
                index_c2=(np.linalg.norm(plc_c2-pos_c1,axis=1)<5)
#                            print("plc_c2 rgb_c2 length",len(plc_c2),len(rgb_c2),len(index_c2))
            # print("plc_c2",plc_c2)
            if (time_cap!=0 and time.time()-time_cap>1):
                dyn_obs_reset=1
                c_dynkf=[]
                v_dynkf=[]
                obsdkf=[]
                x_mat=[]
                print("KF time out reset!")
    
            if len(plc_c2)>0 and len(plc_c1)>0 and len(index_c1)==len(plc_c1) and len(index_c2)==len(plc_c2) and index_c1.any() and index_c2.any():
                plc_c2=plc_c2[index_c2]
                rgb_c2=rgb_c2[index_c2]
                plc_c1=plc_c1[index_c1]
                rgb_c1=rgb_c1[index_c1]
               
                db1 = skc.DBSCAN(eps=0.42, min_samples=n_p).fit(plc_c1)
                db2 = skc.DBSCAN(eps=0.42, min_samples=n_p).fit(plc_c2)
                labels1 = db1.labels_
                labels2 = db2.labels_
                n_clusters1_ = len(set(labels1)) - (1 if -1 in labels1 else 0)
                n_clusters2_ = len(set(labels2)) - (1 if -1 in labels2 else 0)
                if n_clusters1_ * n_clusters2_== 0:
                    continue
                for i in range(n_clusters1_):
                    one_cluster = plc_c1[labels1 == i]
                    rgb_cluster = rgb_c1[labels1 == i]
                    clu_p1.append(one_cluster)
                    rgb_p1.append(rgb_cluster)
                    c_c_dyn1,r_obs1=convert.correct_c_dyn(np.mean(one_cluster,axis=0),one_cluster,pos_c1)
                    r_obs1=max(0.3,r_obs1)
                    fea_dyn1=convert.f_vector(c_c_dyn1,r_obs1,one_cluster,rgb_cluster)
                    # clu_c1.append(np.mean(one_cluster,axis=0))
                    clu_c1.append(c_c_dyn1)
                    fea_1.append(fea_dyn1)
                i=0
                for i in range(n_clusters2_):
                    one_cluster = plc_c2[labels2 == i]
                    rgb_cluster = rgb_c2[labels2 == i]
                    clu_p2.append(one_cluster)
                    
                    rgb_p2.append(rgb_cluster)
                    c_c_dyn2,r_obs2=convert.correct_c_dyn(np.mean(one_cluster,axis=0),one_cluster,pos_c2)
                    r_obs2=max(0.3,r_obs2)
                    fea_dyn2=convert.f_vector(c_c_dyn2,r_obs2,one_cluster,rgb_cluster)
                    # clu_c2.append(np.mean(one_cluster,axis=0))
                    clu_c2.append(c_c_dyn2)
                    fea_2.append(fea_dyn2)
                    obsd2.append(2*r_obs2)
                pcl_clust = copy.copy(clu_p2)
                clu_c1=np.array(clu_c1)
                clu_c2=np.array(clu_c2)
                fea_1=np.array(fea_1)
                fea_2=np.array(fea_2)

                # if clu_c1[0].shape
                print('number of obstacles last and this frame :',n_clusters1_,n_clusters2_)
                if n_clusters1_==0:
                    print(plc_c1,labels1)
                time11=time.time()
                no_cdyn0=2
                i=0

                jj = 0
                not_dyn_pt = list(range(0,n_clusters2_))
                for i in range(n_clusters2_):
#                                cnorm=np.linalg.norm(clu_c1-clu_c2[i],axis=1)
#                                print(fea_1,fea_2[i])
                    cnorm=np.linalg.norm((fea_1-fea_2[i])[:,0:-2],axis=1)   #difference between the feature vector and get the 2-norm
                    cnorm_rgb=np.linalg.norm((fea_1-fea_2[i])[:,4::],axis=1)
#                                if min(cnorm)<0.1 or (abs(clu_c2[i]-clu_c1[np.argmin(cnorm)])<0.08).all():
#                                print(min(cnorm))
                    min_cnorm=min(cnorm)
                    min_cnorm_rgb=min(cnorm_rgb)
                    dis_21=clu_c2[i]-clu_c1[np.argmin(cnorm)]
                    dis_21_rgb=clu_c2[i]-clu_c1[np.argmin(cnorm_rgb)]
                    if local_pos[2]<0.3:
                        vel_coe=0
                    else:
                        vel_coe=0.5
                    #+vel_coe*np.linalg.norm(convert.line_vel)
                    if min_cnorm<dt*(3 +vel_coe*np.linalg.norm(convert.line_vel)) and np.linalg.norm(dis_21)<dt*(0.3) and np.linalg.norm(dis_21) != 0:#and clu_c2[i,2]<1.7:#(dis_21<dt*0.39).all():  7.0 for hardware,2.0 for simulation
                        plc_sta=plc_sta+clu_p2[i].tolist()
                        print('static obs!',min_cnorm,np.linalg.norm(dis_21))
#                                    clu_p2.remove(clu_p2[i])
                    # n_clusters1_ >= n_clusters2_ and
					
                    elif min_cnorm_rgb<dt*(130.0 + 5*np.linalg.norm(convert.line_vel)) and np.linalg.norm(dis_21_rgb)>dt*0.2 and abs(dis_21_rgb[2])/np.linalg.norm(dis_21_rgb[0:2]) < 1 and np.linalg.norm(dis_21_rgb)<4*dt  and np.linalg.norm(clu_c2[i]-pos_c1)<5 and obsd2[i]<2.5:   #find the moving obstacles!!!
                        print("potential dynamic obs!",cnorm_rgb,t_dh,time11)
#                                        not_dyn_pt.remove(i)
                        del pcl_clust[i-(n_clusters2_ - len(pcl_clust))]
#                                        print(pcl_clust,clu_p2)
                        if (len(t_dh)>0 and time11-t_dh[-1]>2.0*dt):
                            print("store reset, can't locate potential dynamic obs in %s s"%(time11-t_dh[-1]))
                            c_dyn=[]    #center of the moving object
                            dyn_v=[]    #speed of the moving object
                            dyn_d=[]
                            t_dh=[]
                            f_dyn=[]
                            

                        if (len(t_dh)<1 or time11==t_dh[0]) and jj ==0:
                            
                            c_c_dyn2=clu_c2[i]
                            c_c_dyn1=clu_c1[np.argmin(cnorm_rgb)]
                            t_dh.append(time11)
                            dyn_v.append((c_c_dyn2-c_c_dyn1)/(t_c2-t_c1))
                            c_dyn.append(c_c_dyn2)      #the matched obstacle center which are probably moving
#                                            print(c_c_dyn2,obsd2,clu_p2,rgb_p2,i)
                            dyn_d.append(obsd2[i])
                            fea_dyn=convert.f_vector(c_c_dyn2,0.5*obsd2[i],clu_p2[i],rgb_p2[i])[4::]
                            f_dyn.append(fea_dyn)   # the feature of the moving obs at first time observed
                            if_inherit = 0
                        elif (len(t_dh)>0 and time11-t_dh[0]<3*dt and time11-t_dh[0]>0.01*dt) or if_inherit ==1:  #in a short time find the moving obs twice
                            # c_dyn2_1=np.linalg.norm(c_dyn-clu_c1[np.argmin(cnorm)],axis=1)
                            jj+=1
                            print("number of potential dyns:",len(c_dyn))
                            c_c_dyn2=clu_c2[i]
                            c_dyn2_1=np.linalg.norm(c_dyn-clu_c2[i],axis=1)
                            which_cdyn=np.argmin(c_dyn2_1)
                            time22=time.time()
                            fnorm=np.linalg.norm(np.array(f_dyn)-np.array(convert.f_vector(c_c_dyn2,0.5*obsd2[i],clu_p2[i],rgb_p2[i])[4::]),axis=1)   #second match
#                                            print("111111111",c_dyn2_1,fnorm)
                            print("111111111",c_dyn2_1,fnorm,time22-t_dh[-1])
                            if min(fnorm)<40 and (if_inherit ==1 or ((min(c_dyn2_1)==0 or (min(c_dyn2_1)>(time22-t_dh[-1])*0.1) and min(c_dyn2_1)<(time22-t_dh[-1])*4))) :  #and min(fnorm)>0.01: #
                                
                                octo_plc1 = np.array(plc_sta_h)    # when don't use octo_map, we use a histroy record of static points
                                print("octo_plc len:",len(octo_plc1))
                                # c_dyn11=clu_c2[np.argmin(cnorm)] #(clu_c1[np.argmin(cnorm)]+c_dyn[which_cdyn])/2
                                dyn_v11=(clu_c2[i]-clu_c1[np.argmin(cnorm_rgb)])/(t_c2-t_c1)
                                # print(dyn_v11)
                                if len(octo_plc1)>10:
#                                                    db3 = skc.DBSCAN(eps=0.4, min_samples=n_p).fit(octo_plc1)
#                                                    labels3 = db3.labels_
#                                                    n_clusters3_ = len(set(labels3)) - (1 if -1 in labels3 else 0)
#                                                    if n_clusters3_>0:
#                                                        for ii in range (n_clusters3_):
#                                                            one_cluster3 = octo_plc1[labels3 == ii]
#                                                            c_c_dyn3,r_obs3=convert.correct_c_dyn(np.mean(one_cluster3,axis=0),one_cluster3,pos_c2)
#                                                            clu_c3.append(c_c_dyn3)
#                                                        clu_c3=np.array(clu_c3)
#                                                    print("octomap obs:",clu_c3)
                                    cnorm_octo=np.linalg.norm(octo_plc1-clu_c2[i],axis=1)
                                    print('cnorm_octo minimum',min(cnorm_octo))
#                                                    if len(clu_c3)>0:
#                                                        cnorm_octo=np.linalg.norm(clu_c3-clu_c2[i],axis=1)
#                                                        print('cnorm_octo minimum',min(cnorm_octo))
                                    # len(clu_c3)>0 and
                                   # max(obsd2[i]-0.1,0.3)
                                    if min(cnorm_octo)>0.4  and np.linalg.norm(dyn_v11) > max(0.3,obsd2[i]*0.5):
                                        c_dyn11=clu_c2[i]
#                                            dyn_v11=((clu_c2[i]-clu_c1[np.argmin(cnorm)]+pos_c1)/(t_c2-t_c1)+0.5*dyn_v[which_cdyn])/1.5*fac_dv #1.8->Correction factor
#                                                        dyn_v11=((c_dyn11-clu_c1[np.argmin(cnorm_rgb)])/(t_c2-t_c1)*1.8 + 0.2*np.array(dyn_v[which_cdyn]))/2
                                        # dyn_v11=(c_dyn11-clu_c1[np.argmin(cnorm_rgb)])/(t_c2-t_c1)
                                        time_cap=time.time()
                                        print("correct!!!!")
                                elif np.linalg.norm(dyn_v11) > max(0.3,obsd2[i]*0.5):
                                    c_dyn11=clu_c2[i]
#                                            dyn_v11=((clu_c2[i]-clu_c1[np.argmin(cnorm)]+pos_c1)/(t_c2-t_c1)+0.5*dyn_v[which_cdyn])/1.5*fac_dv #1.8->Correction factor
#                                                    dyn_v11=((c_dyn11-clu_c1[np.argmin(cnorm_rgb)])/(t_c2-t_c1)*1.8 + 0.2*np.array(dyn_v[which_cdyn]))/2
                                    # dyn_v11=(c_dyn11-clu_c1[np.argmin(cnorm_rgb)])/(t_c2-t_c1)
                                    time_cap=time.time()
 
                                    print("correct!!!!")
                                if (c_dyn11 is not None) and (len(c_dyn11)!=0):
                                    dyn_v1.append(dyn_v11)
                                    c_dyn1.append(c_dyn11)
                                    obsd.append(obsd2[i])
                                    c_dynkf=c_dyn1
                                    v_dynkf=dyn_v1
                                    obsdkf=obsd
                            if (c_dyn11 is not None) and (len(c_dyn11)!=0):
                                c_dyn[which_cdyn] = c_dyn11
                                dyn_v[which_cdyn] = dyn_v11
                                fea_dyn=convert.f_vector(c_dyn11,0.5*obsd2[i],clu_p2[i],rgb_p2[i])[4::]
                                f_dyn[which_cdyn] = fea_dyn   # the feature of the moving obs at first time observed
#                                                t_dh = [time.time()]
                                if_inherit = 1
                            else:
                                c_dyn=[]    #center of the moving object
                                dyn_v=[]    #speed of the moving object
                                dyn_d=[]
                                t_dh=[]
                                f_dyn=[]
                                if_inherit = 0
                if len(pcl_clust) >0:
                    if len(plc_h_l) >0:
                        if len(np.array(pcl_clust)[0]) < len(plc_h_l[-1]) * 1.2 or len(np.array(pcl_clust)[0]) - len(plc_h_l[-1]) <10:
                            plc_h_l.append(np.array(pcl_clust)[0])
                            t_plc_l.append(time.time())
                    else:
                        plc_h_l.append(np.array(pcl_clust)[0])
                        t_plc_l.append(time.time())
                if len(t_plc_l) > 30:
                    plc_h_l = plc_h_l[-30::]
                    t_plc_l = t_plc_l[-30::]
                if len(t_plc_l) > 0:
                    print("point cloud record length",len(plc_h_l),len(t_plc_l),t_plc_l[-1] - t_plc_l[0])

                            # continue
#    
            if len(c_dyn1)>0 and len(c_dyn_l)>0 :
                for c_dyn_i in c_dyn1 :
                    if min(np.linalg.norm(c_dyn_i - np.array(c_dyn_l), axis=1)) > 0.3:
                        # dyn_obs_reset=1
                        # c_dynkf=[]
                        # v_dynkf=[]
                        x_mat=[]
                        # obsdkf=[]
                        # obsd=np.array([])
                        print("KF new dyns reset!")            
            c_dyn_l,v_dyn_l=[],[]
            for kk in range(len(c_dynkf)):
                kf_dt = protime*1.0
                
                # set initial value
                if (len(c_dyn1)!=0): #dyn_obs_reset==1 or 
                    z_mat = np.mat([c_dynkf[kk],v_dynkf[kk]])
                    p_mat = np.mat([[2, 0], [0, 2]])   # initial state Covariance matrix
                elif len(c_dyn1)==0:
                    c_dynkf[kk] = c_dynkf[kk]+v_dynkf[kk]*kf_dt
                    z_mat = np.mat([c_dynkf[kk],v_dynkf[kk]])
                    p_mat = np.mat([[3, 0], [0, 3]])   # initial state Covariance matrix
                    print("no dynamic imformation, KF predict!")
                if len(x_mat) ==0:
                    x_mat = np.mat([c_dynkf[kk],v_dynkf[kk]])
                # z_mat = np.mat(c_dynkf[kk])
#                                    if len(c_dyn1)==0:
#                                        q_mat = np.mat([[0.001, 0], [0, 0.001]])
#
##                                        print("use KF predict as input")
#                                    else:
#                                        q_mat = np.mat([[0.1, 0], [0, 0.1]])  # State transition Covariance matrix
#                                        print("use ground truth as input")
                q_mat = np.mat([[0.01, 0], [0, 0.01]])
                
                f_mat = np.mat([[1, kf_dt], [0, 1]])     # State transition matrix
              
#                                    q_mat = np.mat([[0.1, 0], [0, 0.1]])
               
                h_mat = np.mat([[1, 0],[0,1]])  #State observation matrix
                r_mat = np.mat([3])     #State observation noise Covariance matrix
 
                
                x_predict = f_mat * x_mat
                p_predict = f_mat * p_mat * f_mat.T + q_mat
                kalman = p_predict * h_mat.T / (h_mat * p_predict * h_mat.T + r_mat)
                x_mat = x_predict + kalman *(z_mat - h_mat * x_predict)
                p_mat = (np.eye(2) - kalman * h_mat) * p_predict
                if len(c_dyn_l) == 0 or min(np.linalg.norm(np.array(x_mat)[0]-np.array(c_dyn_l),axis=1)) > 0.3:
                    c_dyn_l.append(list(np.array(x_mat)[0]))
                    v_dyn_l.append(list(np.array(x_mat)[1]))
                print("KF is used!!")
#                                    c_dynkf=c_dyn_l
#                                    v_dynkf=v_dyn_l
                dyn_obs_reset=0
            if len(c_dyn_l)>0:
                print('moving obj!!!!!!!!:',c_dyn_l,v_dyn_l,obsdkf)#,'plc1:',plc1)
                c_dyn1=np.array(c_dyn_l)
                dyn_v1=np.array(v_dyn_l)
                obsd=np.array(obsdkf)
                #obsd[np.where(obsd<0.5)]=0.5
                c_dyn11=[]
                dyn_v11=[]
                # continue
            st_clusters = []
            if len(c_dyn1) > 0:
                # clt = np.where(np.linalg.norm())
                for clt in range(len(clu_p2)):
                    if min(np.linalg.norm(clu_c2[clt]-c_dyn1,axis=1)) > 0.5:
                       print(clu_c2[clt],c_dyn1)
                       if len(st_clusters)==0:
                           st_clusters = np.array(clu_p2[clt])
                       else:
                           st_clusters= np.r_[st_clusters , np.array(clu_p2[clt])]
                plc1 = st_clusters
                # print("st_clusters",st_clusters)
            if len(plc1)>0:
                plc1=plc1-local_pos
                
                if len(plc_sta_h) > 0:
                    plc_sta_h = plc_sta_h[::int(len(plc_sta_h)/50)+1]
                    plc1=np.r_[plc1,plc_sta_h-local_pos]
                
                plc1=convert.distance_filter(plc1,4)
                plc1=plc1+local_pos
                
            if len(c_dyn1)>0 and len(obsd)>0 and len(plc1)>0:
                print("obsd:",obsd,c_dyn1)
                # if len(plc1)>0:
                #     plc1=np.r_[plc1,c_dyn1]
                # else:
                #     plc1=c_dyn1
                plc1=np.r_[plc1,c_dyn1]
                plc1=np.r_[plc1,dyn_v1]
                obsd_pcl=np.c_[obsd.T,np.zeros([len(obsd),2])]
#                        print("plc1,obsd_pcl",plc1,obsd_pcl)
                plc1=np.r_[plc1,obsd_pcl]
            if len(plc1)>0:
                plc1=np.r_[plc1,np.array([[len(c_dyn1),0,0]])] #attach the number of dynamic obstacles
            # else:
            #     plc1=np.array([[len(c_dyn1),0,0]])

            point_al=convert.xyz_array_to_pointcloud2(np.array(plc1),'map',convert.plc_timestamp)
            if point_al is not None:
                # print(point2)
                convert.alpub.publish(point_al)
                print("plc_all length",len(plc1))
            point_sta=convert.xyz_array_to_pointcloud2(np.array(plc_sta_h),'map',rospy.Time.now())
            if point_sta is not None:
                # print(point2)
                convert.pub.publish(point_sta)

            protime=time.clock()-starttime1
            # print('octo_plc-local num:',len(octo_plc))
            print('protime:',protime)
            if point22 is not None:
                # print(point2)
                convert.octo_pub.publish(point22)
#                
            if c_dyn1 !=[] and dyn_v1 !=[] and len(obsd) != 0:
                # print("obsd:",obsd,c_dyn1)
                convert.publish_dyn_obs(c_dyn1,dyn_v1,obsd,[px,py,pz,y])
            if c_dyn !=[]:
                convert.publish_potential_dyn_obs(c_dyn,dyn_v,dyn_d)
            else:
                rate.sleep()
