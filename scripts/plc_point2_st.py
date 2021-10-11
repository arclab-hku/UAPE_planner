#!/usr/bin/env python2
import rospy
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
import math
from message_filters import TimeSynchronizer, Subscriber,ApproximateTimeSynchronizer
from nav_msgs.msg import Path
class convert_plc():
    def callback(self,data):
        #y.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        assert isinstance(data, PointCloud2)
        # global point2,pos,pub
        self.plc = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    
    def thread_job():
        rospy.spin()
        
    def local_position_callback(self,data):
        self.pos=data
        # parse_local_position(data)
        
    def velocity_callback(self, data):
        self.ang_vel = np.array([data.twist.angular.x,data.twist.angular.y,data.twist.angular.z])
    def octo_callback(self,data):
        assert isinstance(data, PointCloud2)
        # global point2,pos,pub
        self.octo_plc = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        self.octo_time =data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
    def pos_pcl(self,pcl,pos,vel):
        # self.pos=pos.pose
        self.pos=pos
        self.pos_time = pos.header.stamp.secs + pos.header.stamp.nsecs * 1e-9
#        assert isinstance(pcl, PointCloud2)
        # global point2,pos,pub
        self.plc = list(point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True))
        self.plc_time =pcl.header.stamp.secs + pcl.header.stamp.nsecs * 1e-9
        self.plc_timestamp = pcl.header.stamp
        # self.octo_plc = list(point_cloud2.read_points(octo, field_names=("x", "y", "z"), skip_nans=True))
        # self.octo_time =octo.header.stamp.secs + octo.header.stamp.nsecs * 1e-9
        self.ang_vel = np.array([vel.twist.angular.x,vel.twist.angular.y,vel.twist.angular.z])
        self.line_vel = np.array([vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z])
        self.vel_time = vel.header.stamp.secs + vel.header.stamp.nsecs * 1e-9
        print("alighed",self.vel_time,self.plc_time)
        self.if_align = 1

    def listener(self):
    
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('point_transfrer', anonymous=True)
        self.pub = rospy.Publisher('/points_global', PointCloud2, queue_size=1)  #static points
        # self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity',
        #                                TwistStamped,
        #                                self.velocity_callback)
        # self.local_vel_sub1 = rospy.Subscriber('mavros/local_position/velocity_local',
        #                                TwistStamped,
        #                                self.velocity_callback)
        self.alpub = rospy.Publisher('/points_global_all', PointCloud2, queue_size=1)    #all points
        self.octo_pub = rospy.Publisher('/octomap_point_cloud_centers_local', PointCloud2, queue_size=1)
        # self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
        #                                           PoseStamped,
        #                                           self.local_position_callback,queue_size=1,buff_size=52428800)
        # self.plc_sub = rospy.Subscriber('/filtered_RadiusOutlierRemoval',
        #                               PointCloud2,
        #                               self.callback,queue_size=1,buff_size=52428800) #/filtered_RadiusOutlierRemoval',
        self.octo_plc_sub = rospy.Subscriber('/octomap_point_cloud_centers',
                                          PointCloud2,
                                          self.octo_callback,queue_size=1,buff_size=52428800) #/octomap_point_cloud_centers


        self.tss = ApproximateTimeSynchronizer([
                                                Subscriber('/filtered_RadiusOutlierRemoval',PointCloud2),
                                                # Subscriber('/gt_iris_base_link_imu', Odometry),
                                                Subscriber('mavros/local_position/pose', PoseStamped),
                                                Subscriber('mavros/local_position/velocity_local',TwistStamped)
                                                ],
        5,0.03, allow_headerless=True)
        self.tss.registerCallback(self.pos_pcl)
    
    def parse_local_position(self,local_position, mode="e"):
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
            return (rx,ry,rz,qx,qy,qz,qw)
    
        elif mode == "e":
            rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
            return (rx,ry,rz)+ rpy_eular
        else:
            raise UnboundLocalError
            
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
    def distance_filter(self,plc,dis):

        
        d_point=np.linalg.norm(plc,axis=1)
        filtered_plc=np.c_[plc,d_point]
        filtered_plc=filtered_plc[d_point<dis]
        if len(filtered_plc)>0:
            filtered_plc=filtered_plc[np.lexsort(filtered_plc.T)]
            # filtered_plc=filtered_plc[:,0:3]
        return filtered_plc[:,0:3]



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
        c_c_dyn=(c_orin-local_pos)*AO/AG+local_pos
        # print("AO,AG!!!!!!!!!!!",AO,AG)
        return c_c_dyn
if __name__ == '__main__':
    # global pub,rate,point2,pos

    convert=convert_plc()
    convert.listener()
    # convert.vel=None
    convert.pos=None
    convert.octo_plc=None
    convert.plc=None
    convert.f=None
    rosrate=80
    r0=0
    p0=0
    y0=0
    map_reso=0.2
    rate = rospy.Rate(rosrate) # 10hz
    dt=0.2
    n_p=6
    plc_h=[]
    t_plc=[]
    pos_h=[]
    c_dyn0=[]
    dyn_v0=[]
    time22=0
    pre_ct=0
    c_dyn1=[]
    while not rospy.is_shutdown():
        starttime1 = time.clock()
        plc1=[]
        clu_p1=[]
        clu_p2=[]
        clu_c1=[]
        clu_c2=[]
        plc_c=[]
        plc=[]
        # print(pos)
        if convert.pos is not None and (convert.plc is not None) :#and (convert.vel is not None)
            px,py,pz,r,p,y=convert.parse_local_position(convert.pos)
            
#            if len(t_plc)>0:
#                ddt=time.time()-t_plc[-1]
#            else:
#                ddt=0.014
#            
#            # print(r,p,y)
#            # print(r0,p0,y0)
#            wr=(r-r0)/ddt
#            wp=(p-p0)/ddt
#            wy=(y-y0)/ddt
#            r0=r
#            p0=p
#            y0=y
            
#            print('rotating velocity',convert.ang_vel)
#            print('uav status',px,py,pz,r,p,y)
            # vel=convert.parse_velocity(convert.vel)
            # print(px,py,pz,r,p,y)

            length=len(convert.plc)
            plc=convert.plc
            # print("wr,wp",wr,wp)
            plc=np.array(plc)
            plc_c=plc.copy()
#            if abs(wr)<0.3 and abs(wp)<0.3 and abs(wy)<1.5 and abs(p) < math.pi/6 and px*py!=0:#and wr+wp+wy!=0:  #when UAV is not flat and stable don't use the pointcloud
            if abs(convert.ang_vel[0])<3 and abs(convert.ang_vel[1])<3 and abs(convert.ang_vel[2])<3 and abs(p) < math.pi/3 :
#                for i in range(length):
#                    # print(i,len(convert.plc),len(convert.plc[i]))
#                        point=np.array([plc[i][2],-plc[i][0],-plc[i][1]])
#                    
#                        point=np.matmul(b2e,point)+local_pos
#                        if point[2]>0.3:
#                            plc1.append(point)
                if len(pos_h)!=0 and np.linalg.norm(np.array([px,py,pz])-pos_h[-1])<0.1 and len(plc)>n_p :
                    plc_c[:,0]=plc[:,2]+0.12  #distance from camera to uav center
                    plc_c[:,1]=-plc[:,0]
                    plc_c[:,2]=-plc[:,1]
    #                plc_c=np.array(plc_c)
                    r,p,y = np.array([r,p,y])+(convert.plc_time - convert.pos_time)*convert.ang_vel
                    b2e = body_to_earth_frame(r,p,y)
                    local_pos1=(convert.plc_time - convert.pos_time)*convert.line_vel + np.array([px,py,pz])
                    plc_c=np.matmul(b2e,plc_c.T).T+np.tile(local_pos1,(length,1))
            else:
                print('Rotating too fast!!!',convert.ang_vel)
#            plc1=np.array(plc1)
            if len(plc_c)>0:
                plc1=plc_c[plc_c[:,2]>0.3]
            plc_sta=[]  #static points
            
            if len(plc1)>n_p:
                if len(t_plc)>1 and t_plc[-1]-t_plc[-2]>dt*rosrate:
                    plc_h=[]
                    t_plc=[]
                    pos_h=[]
                    print('refresh!!!')
       
                plc_h.append(plc1)
                local_pos=(convert.plc_time - convert.pos_time)*convert.line_vel + np.array([px,py,pz])
                pos_h.append(local_pos)
                t_plc.append(time.time())
            # dyn_v1=[]
            # c_dyn1=[]
            if len(t_plc)>1 :
                for i in range(len(t_plc)-1,-1,-1):
                    # try:
                    #     asdas=t_plc[i]
                    # except:
                    #     print("i",i,t_plc)
                    if i<len(t_plc) and t_plc[-1]-t_plc[i]>=dt:

                        if t_plc[-1]-t_plc[i+1]<dt:
                            plc_c2=plc_h[-1]
                            plc_c1=plc_h[i]
                            t_c2=t_plc[-1]
                            t_c1=t_plc[i]
                            pos_c2=pos_h[-1]
                            pos_c1=pos_h[i]
                            plc_h=plc_h[i+1::]
                            t_plc=t_plc[i+1::]
                            pos_h=pos_h[i+1::]

#                            plc_c2=plc_c2.tolist()
#                            for pp in plc_c2:
#                                if np.linalg.norm(pp-pos_c1)>8:
#                                    plc_c2.remove(pp)
#                            plc_c2=np.array(plc_c2)
                            plc_c2=plc_c2[np.linalg.norm(plc_c2-pos_c1,axis=1)<4]
#                            print(plc_c2)
                            if len(plc_c2)>0:
                                db1 = skc.DBSCAN(eps=0.4, min_samples=n_p).fit(plc_c1)
                                db2 = skc.DBSCAN(eps=0.4, min_samples=n_p).fit(plc_c2)
                                labels1 = db1.labels_
                                labels2 = db2.labels_
                                n_clusters1_ = len(set(labels1)) - (1 if -1 in labels1 else 0)
                                n_clusters2_ = len(set(labels2)) - (1 if -1 in labels2 else 0)
                                if n_clusters1_ == 0:
                                    break
                                for i in range(n_clusters1_):
                                    one_cluster = plc_c1[labels1 == i]
                                    clu_p1.append(one_cluster)
                                    c_c_dyn1=convert.correct_c_dyn(np.mean(one_cluster,axis=0),one_cluster,local_pos)
                                    # clu_c1.append(np.mean(one_cluster,axis=0))
                                    clu_c1.append(c_c_dyn1)
                                for i in range(n_clusters2_):
                                    one_cluster = plc_c2[labels2 == i]
                                    clu_p2.append(one_cluster)
                                    c_c_dyn2=convert.correct_c_dyn(np.mean(one_cluster,axis=0),one_cluster,local_pos)
                                    # clu_c2.append(np.mean(one_cluster,axis=0))
                                    clu_c2.append(c_c_dyn2)
                                clu_c1=np.array(clu_c1)
                                clu_c2=np.array(clu_c2)
                                
    
                                # if clu_c1[0].shape
                                print('number of obstacles this and last frame:',n_clusters1_,n_clusters2_)
                                if n_clusters1_==0:
                                    print(plc_c1,labels1)
                                time11=time.time()
                                no_cdyn0=2
                                for i in range(n_clusters2_):
                                    cnorm=np.linalg.norm(clu_c1-clu_c2[i],axis=1)
                                    if (min(cnorm)<0.1 or (abs(clu_c2[i]-clu_c1[np.argmin(cnorm)])<0.08).all()) and clu_c2[i,2]<1.7:
                                        plc_sta=plc_sta+clu_p2[i].tolist()
                                    

            
            if len(plc1)>0:
                plc1=plc1-local_pos
                plc1=convert.distance_filter(plc1,4)
                plc1=plc1+local_pos
                if len(plc1)>0:
                    plc1=np.r_[plc1,np.array([[len(c_dyn1),0,0]])] #attach the number of dynamic obstacles

            point_al=convert.xyz_array_to_pointcloud2(np.array(plc1),'map',rospy.Time.now())
            if point_al is not None:
                # print(point2)
                convert.alpub.publish(point_al)
            point_sta=convert.xyz_array_to_pointcloud2(np.array(plc_sta),'map',rospy.Time.now())
            if point_sta is not None:
                # print(point2)
                convert.pub.publish(point_sta)
        if convert.pos is not None and (convert.octo_plc is not None) :
            
            octo_plc1=[]
            octo_plc=np.array(convert.octo_plc)
            # print('num-obs',len(octo_plc))
            px,py,pz,r,p,y=convert.parse_local_position(convert.pos)
            local_pos=(convert.octo_time - convert.pos_time)*convert.line_vel + np.array([px,py,pz])
            
            if len(octo_plc)>0:
                octo_plc=octo_plc-local_pos
                octo_plc=octo_plc[(abs(octo_plc)<4).all(axis=1)]
                octo_plc=convert.distance_filter(octo_plc,4)+local_pos
            
            point22=convert.xyz_array_to_pointcloud2(octo_plc,'map',rospy.Time.now())
            protime=time.clock()-starttime1
            # print('octo_plc-local num:',len(octo_plc))
            print('protime:',protime)
            if point22 is not None:
                # print(point2)
                convert.octo_pub.publish(point22)
                rate.sleep()