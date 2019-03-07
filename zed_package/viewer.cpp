#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/io/io.h>

class cloudHandler
{
protected:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  pcl::visualization::CloudViewer viewer;
  ros::Timer viewer_timer;
public:
  cloudHandler() : viewer("Nuvem Acumulada")
  {
    pcl_sub = nh.subscribe("zed_neymar", 100, &cloudHandler::cloudCB, this);
    viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);
  }
  void cloudCB(const sensor_msgs::PointCloud2 &input)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(input, cloud);
    viewer.showCloud(cloud.makeShared());
  }
  void timerCB(const ros::TimerEvent&)
  {
    if(viewer.wasStopped())
      ros::shutdown();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"pcl_visualize");
  cloudHandler handler;
  ros::spin();
  return 0;
}
