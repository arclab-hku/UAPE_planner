#include <ros/ros.h>
// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加引用

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <string>

using namespace std;

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::Normal> Normal;

ros::Publisher pub;
string input;
double cut_dis;
double voxel_size;
double n_r;
int n_n;
int MK;
double stdthr;
bool use_time;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //read raw point cloud data
    //and convert to raw_cloud
    sensor_msgs::PointCloud2 output;
    PointCloud::Ptr raw_cloud  (new PointCloud);
    output = *cloud_msg;
    pcl::fromROSMsg(output, *raw_cloud);   //

    //create temp point cloud ptr
    PointCloud::Ptr    cloud_filtered   (new PointCloud);
    PointCloud::Ptr    voxel_filtered   (new PointCloud);
    PointCloud::Ptr    r_filtered   (new PointCloud);
    PointCloud::Ptr    sta_filtered   (new PointCloud);


    // Perform the actual filtering-1
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (raw_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, cut_dis);
    pass.filter (*cloud_filtered);





    // Perform the actual filtering-2
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setLeafSize (voxel_size,voxel_size, voxel_size);
    sor.filter (*voxel_filtered);


    // Perform the actual filtering-3
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(voxel_filtered);
    outrem.setRadiusSearch(n_r);
    outrem.setMinNeighborsInRadius (n_n);
    outrem.filter (*r_filtered);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sta;
    sta.setInputCloud(r_filtered);
    sta.setMeanK(MK);
    sta.setStddevMulThresh(stdthr);
    sta.filter(*sta_filtered);


    // Convert to ROS data type
//    sensor_msgs::PointCloud2 cloud_pt;
//    pcl_conversions::moveFromPCL(r_filtered, cloud_pt);
//      pcl_conversions::toROSMsg(r_filtered, cloud_pt)
//    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    // Publish the data
    if (use_time)
    {
      pcl_conversions::toPCL(ros::Time::now(), sta_filtered->header.stamp);
    }
    pub.publish (sta_filtered);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "chen_filter");
  ros::NodeHandle nh("~");
  nh.getParam("input", input);
  nh.getParam("cut_dis", cut_dis);
  nh.getParam("voxel_size", voxel_size);
  nh.getParam("n_r", n_r);
  nh.getParam("n_n", n_n);
  nh.getParam("MK", MK);
  nh.getParam("std", stdthr);
  nh.getParam("use_current_time", use_time);
//  cout<<""
  // Create a ROS subscriber for the input point cloud 输入
//  cout<<"input topic:"<<input<<endl<<cut_dis<<endl;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (input, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud 输出
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/filtered_RadiusOutlierRemoval", 1);

  // Spin
  ros::spin ();
}
