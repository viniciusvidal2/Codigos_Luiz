/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////            Grupo de Robótica Inteligente - GRIn                    ///////////////////
//////////////                      Projeto Virtual                               ///////////////////
//////////////               Luiz Augusto Zillmann da Silva                       ///////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// Sincronizando nuvem Astra ///////////////////////////////////////////
///                                                                                               ///

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Diversos
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/io/ply_io.h>

// EIGEN
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

// Messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>


using namespace pcl;
using namespace std;
using namespace tf;
using namespace message_filters;
//using namespace sensor_msgs;
using namespace nav_msgs;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis e definicoes globais
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definicoes
typedef PointXYZRGB       PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, Odometry> syncPolicy;

// Variaveis
PointCloud<PointT>::Ptr nuvem_colorida;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;
boost::shared_ptr<ros::Publisher> pub2;
// Matriz intrinseca K para Depth cam
Eigen::Matrix3f K1; //  MATLAB

float fxd, fyd, Cxd, Cyd;// = getIntrinsicsFromK(K, "D");

// Matriz intrinseca K para RGB cam
Eigen::Matrix3f K2;

float fxrgb, fyrgb, Cxrgb, Cyrgb;// = getIntrinsicsFromK(K, "D");

// Calculando Params. Extrisecos
Eigen::MatrixXf RT(3,4);

Eigen::MatrixXf P;

Eigen::Matrix3f F;

cv::Mat image = cv::imread("/home/luiz/Pictures/deadline.jpeg");

int contagem = 0;

time_t t_init, t_fim;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Medições de tempo
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tempo(const time_t t_start, time_t t_end)
{
  t_end = time(0);
  time_t duration = t_end - t_start;

  ofstream fs;

  fs.open("/home/luiz/ZED_WS/build/zed_package/Tempos_de_Aquisicao.txt");
  for (int k = 0; k == contagem; k++)
  {
     fs << "\n\n";
  }

  if (fs.is_open())
  {
    contagem++;
    fs << "Tempo de aquisição para envio numero " << contagem <<"\n \n";

    fs << duration << " ";
//      fs << "\n";
    fs.close();
  }
  else
  {
    std::cout << "erro\n" << std::endl;
  }
}

Eigen::Matrix3f getIntrinsicsCalibration(const string cam)
{
  Eigen::Matrix3f K;
  if(cam == "RGB")
  {
    float fxrgb, fyrgb, Cxrgb, Cyrgb, Srgb;
    fxrgb = fxrgb/Srgb;
    fyrgb = fyrgb/Srgb;
    K << fxrgb,     0, Cxrgb,
             0, fyrgb, Cyrgb,
             0,     0,     1;
  }
  else if(cam == "Depth")
  {
    float fxd, fyd, Cxd, Cyd, Sd;
    fxd = fxd/Sd;
    fyd = fyd/Sd;
    K << fxd,   0, Cxd,
           0, fyd, Cyd,
           0,   0,   1;
  }

  return K;
}

float getIntrinsicsFromK( Eigen::Matrix3f K, const string cam)
{
  if(cam == "RGB")
  {
    float fxrgb, fyrgb, Cxrgb, Cyrgb;

    fxrgb = K(0,0);
    fyrgb = K(1,1);
    Cxrgb = K(0,3);
    Cyrgb = K(1,3);

    return fxrgb, fyrgb, Cxrgb, Cyrgb;
  }
  else if(cam == "D")
  {
    float fxd, fyd, Cxd, Cyd;

    fxd = K(0,0);
    fyd = K(1,1);
    Cxd = K(0,2);
    Cyd = K(1,2);

    return fxd, fyd, Cxd, Cyd;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Filtro para distâncias
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(pcl::PointCloud<PointT>::Ptr in, std::string field, float min, float max){
  pcl::PassThrough<PointT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Voxel Grid
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void voxelgrid_nuvem(PointCloud<PointT>::Ptr in, float lf){
  VoxelGrid<PointT> grid;
  grid.setLeafSize(lf, lf, lf);
  grid.setInputCloud(in);
  grid.filter(*in);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Removendo Outliers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*in);
}

void callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth, const OdometryConstPtr& msg_odo)
{
  // Imagens RGB e Depth
//  cv::Mat cv_image_rgb;
//  cv::Mat cv_image_d(cv::Size(640,480),CV_16UC1); //CV_32FC1

  cv_bridge::CvImagePtr cv_ptr_d;
  cv_bridge::CvImagePtr cv_ptr_rgb;
  cv_ptr_d = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  cv_ptr_rgb = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);

  // Nuvem de pontos colorida
//  PointCloud<PointT>::Ptr nuvem_colorida (new PointCloud<PointT>());
  sensor_msgs::PointCloud2 msg_cor;

//  cv_image_rgb = cv_bridge::toCvShare(msg_rgb, "bgr16")->image;
//  cv_image_d   = cv_bridge::toCvShare(msg_depth)->image;


  //cout << fxd << endl;

  int depthHeight = cv_ptr_d->image.rows;  //cv_image_d.rows;
  int depthWidth  = cv_ptr_d->image.cols;  //cv_image_d.cols;

  nuvem_colorida->resize(depthHeight*depthWidth);

  int aux = 0;
  cv::Vec3b intensity;

  for(int v = 0; v < depthHeight; v=v+2)
  {
    for(int u = 0; u < depthWidth; u=u+2)
    {

      float x, y, z;
      z = cv_ptr_d->image.at<short int>(v, u);
      if(z!=0) // eliminando pontos de prof. zero
      {
        x = ((u - Cxd)*z)/fxd;
        y = ((v - Cyd)*z)/fyd;

        // Reprojetando...
        Eigen::MatrixXf X_(4,1);
        X_ << x,
              y,
              z,
              1;

        Eigen::MatrixXf X = P*X_;
        X = X/X(2,0);

        if( floor(X(0,0)) >= 0 && floor(X(0,0))<depthWidth &&  floor(X(1,0)) >= 0 && floor(X(1,0))<depthHeight)
        {
          Eigen::MatrixXf x1(3,1);
          x1 << u,
                v,
                1;
          Eigen::MatrixXf Fx1 = F*x1;
          Eigen::MatrixXf Ftx2 = F.transpose() * X;
          Eigen::MatrixXf Err = (X.transpose() * F * x1)/(Fx1(0,0)*Fx1(0,0) + Fx1(1,0)*Fx1(1,0) + Ftx2(0,0)*Ftx2(0,0) + Ftx2(1,0)*Ftx2(1,0));

          if(Err(0,0) < 0.2)
          {
            float s = 1000;
            nuvem_colorida->points[aux].z = z/s;
            nuvem_colorida->points[aux].x = x/s;
            nuvem_colorida->points[aux].y = y/s;

            intensity = cv_ptr_rgb->image.at<cv::Vec3b>(floor(X(1,0)), floor(X(0,0)));

            nuvem_colorida->points[aux].r = intensity.val[0];
            nuvem_colorida->points[aux].g = intensity.val[1];
            nuvem_colorida->points[aux].b = intensity.val[2];

            aux++;
          }
        }
      }
    }
  }
  nuvem_colorida->resize(aux);

//  cv::namedWindow("RGB", CV_NORMAL);
//  cv::imshow("RGB", cv_image_rgb);
//  cv::namedWindow("Depth", CV_NORMAL);
//  cv::imshow("Depth", cv_image_d);
//  cv::waitKey(1);

  //  ROS_INFO("Publicando Nuvem colorida");
  nuvem_colorida->header.frame_id = "camera_depth_optical_frame";
//  nuvem_colorida->header.stamp = ros::Time::now;

  // Publicando Nuvem
//  cout << "Esperando tecla... " << endl;
//  cv::namedWindow( "Posiciona o robô", cv::WINDOW_AUTOSIZE );// Create a window for display.
//  cv::imshow( "Posiciona o robô", image );
//  if(cv::waitKey(0) != 'p')
//  {
    toROSMsg(*nuvem_colorida, msg_cor);
    msg_cor.header.frame_id = "camera_depth_optical_frame";
    msg_cor.header.stamp = msg_odo->header.stamp;
    pub->publish(msg_cor);

    // Publicando Odometria
    pub2->publish(*msg_odo);

    tempo(t_init, t_fim);

    nuvem_colorida->clear();
//    cv::destroyAllWindows();
//  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sync_node");

  ros::NodeHandle nh;

  t_init = time(0);

  // Variáveis
  K1 <<  582.9937,   -2.8599,  308.9297,
               0,  582.6690,  246.1634,
               0,         0,    1.0000;

  fxd = K1(0,0);
  fyd = K1(1,1);
  Cxd = K1(0,2);
  Cyd = K1(1,2);

  K2 <<   525.1389,    1.4908,  324.1741,
                 0,  521.6805,  244.8827,
                 0,         0,    1.0000;

  fxrgb = K2(0,0); //cout << fxrgb << endl;
  fyrgb = K2(1,1);
  Cxrgb = K2(0,2);
  Cyrgb = K2(1,2);

  RT <<    0.999812767546507,  -0.013049436148855,  -0.014287829337980, -25.056528502229849, // MATLAB -25.056528502229849
           0.012849751269106,   0.999819711122249,  -0.013979597409931, -10.308878899329242, // MATLAB -35.308878899329242
           0.014467679265050,   0.013793384922440,   0.999800194433400,  -0.890397736650369;

  P = K2*RT;

  F << 0.000000051968032,   0.000002923620641,  -0.000171378176749,
       -0.000001735294842,   0.000001158366503,   0.048294523803484,
       -0.000673889418073,  -0.044225087946452,  -0.498521482515482;

  nuvem_colorida = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  //  *pub = nh.advertise<sensor_msgs::PointCloud2>("/depth_map", 100);
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/color_map", 100);

  pub2 = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub2 = nh.advertise<Odometry>("/odom2", 100);

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 100);
  message_filters::Subscriber<Odometry>           subodo   (nh, "/zed/odom", 100);
  Synchronizer<syncPolicy> sync(syncPolicy(100), rgb_sub, depth_sub, subodo);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;

  //  ROS_INFO("Hello world!");


}
