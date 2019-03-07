#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <nav_msgs/Odometry.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>
#include <numeric>

#include <sensor_msgs/JointState.h>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



using namespace pcl;
using namespace pcl::visualization;
using namespace std;
using namespace nav_msgs;
using namespace Eigen;


using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Odometry, JointState> syncPolicy;

//PointCloud<PointXYZRGB>::Ptr nuvem;
//PointXYZRGB point;

boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> pub2;
boost::shared_ptr<ros::Publisher> pub3;
//ros::Publisher pub;
//ros::Publisher pub2;

std::vector<double> j;

void callback(const JointStateConstPtr& msg_joint2)
{
  cout << "Callback!!!!!!!!!!!!!!!! " << endl;
  sleep(10);
  pub3->publish(*msg_joint2);
}

void sincroniza(const OdometryConstPtr& msg_odo, const sensor_msgs::JointStateConstPtr& msg_joint)
{
  // Publicando Odometria
  pub2->publish(*msg_odo);
//  pub2.publish(*msg_odo);

  // Publicando Estados das Juntas
  pub->publish(*msg_joint);
//  pub.publish(*msg_joint);


  cout << "Entramos no Callback??????????????????????????? "  << endl;


//  Eigen::Quaternion<float> q;
//  q.x() = (float)msg_odo->pose.pose.orientation.x;
//  q.y() = (float)msg_odo->pose.pose.orientation.y;
//  q.z() = (float)msg_odo->pose.pose.orientation.z;
//  q.w() = (float)msg_odo->pose.pose.orientation.w;
//  // Translacao
//  Eigen::Vector3f offset(msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z);
//  // Print para averiguar
//  if(false){
//    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
//    cout << "Roll: " << RAD2DEG(euler[0]) << "\tPitch: " << RAD2DEG(euler[1]) << "\tYaw: " << RAD2DEG(euler[2]) << endl;
//    cout << "X   : " << offset(0)         << "\tY    : " << offset(1)         << "\tZ  : " << offset(2)         << endl;
//  }
//  for (int i=0; i<msg_joint->position.size(); i++)
//          j.push_back(msg_joint->position[i]);
//  cout << "Juntas " << j[0] << endl;




}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sincroniza_odom");
  ros::NodeHandle nh;
  cout << "Erro??????????????????????????? "  << endl;
//  nuvem = (PointCloud<PointXYZRGB>::Ptr) new PointCloud<PointXYZRGB>;

  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<JointState>("/joint2", 100);

  pub3 = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub3 = nh.advertise<JointState>("/joint3", 100);

  pub2 = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub2 = nh.advertise<Odometry>("/odom3", 100);

  message_filters::Subscriber<Odometry>                 subodo   (nh, "/odom2" , 100);
  message_filters::Subscriber<JointState>                 subjoint   (nh, "/joint3" , 100);

  ros::Subscriber                subjoint2 =    nh.subscribe("/joint_states" , 100, callback);


  // Sincroniza as leituras dos topicos em um so callback
  Synchronizer<syncPolicy> sync(syncPolicy(100), subodo, subjoint);
  sync.registerCallback(boost::bind(&sincroniza, _1, _2));


  ros::spin();
//  ROS_INFO("Hello world!");
}
