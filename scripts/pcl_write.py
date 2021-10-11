#!/usr/bin/env python2
import rospy
from roslib import message
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs import point_cloud2
import tf
import time
import getpass
class write_plc():
    def callback(self,data):
        #y.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        assert isinstance(data, PointCloud2)
        # global point2,pos,pub
        self.plc = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        self.plc_time =data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
            # print(point2)
        # print(type(gen))
        # for p in gen:
        #   print(" x : %.3f  y: %.3f  z: %.3f" %(p[2],p[0],-p[1]))
        #   print(gen[1])
    
    
    # def talker(point2):
    
    #     # while not rospy.is_shutdown():
    #     pub.publish(hello_str)
    #     rate.sleep()      
    def thread_job():
        rospy.spin()
        

    def listener(self):
    
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('point_cloud_write', anonymous=True)
        self.tflistener = tf.TransformListener()
        self.pointsub = rospy.Subscriber('/global_map', PointCloud2, self.callback, queue_size=1)    #all points



if __name__ == '__main__':
    # global pub,rate,point2,pos
    
    write=write_plc()
    write.plc = None
    write.listener()
    f3 = open("/home/" + getpass.getuser() +"/catkin_ws/src/fuxi-planner/data/"+time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()))+"PCL.txt", 'a')
    while write.plc is None:
        time.sleep(0.1)
    print("start to write!")
    for pcl in write.plc:
        if pcl[2] >0:
            f3.write(str(pcl)+"\n")
    f3.close()