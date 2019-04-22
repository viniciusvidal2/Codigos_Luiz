/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////            Grupo de Robótica Inteligente - GRIn                    ///////////////////
//////////////                      Projeto Virtual                               ///////////////////
//////////////               Luiz Augusto Zillmann da Silva                       ///////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////// Includes ////////////////////////////////////////////////////
///                                                                                               ///

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ctime>

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
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, Odometry> syncPolicy;

// Variaveis
PointCloud<PointT>::Ptr nuvem_acumulada;
PointCloud<PointT>::Ptr nuvem_aux;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;
int flag  = 0;
int cont2 = 1;
int     s = 0;
Eigen::Matrix4f Tc;

long double tstart, tend, duration;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void salvaNuvem()
{
  // Salvar PLY
  if(cont2 == 0)
  {
    cout << "Salvando..." <<endl;
    string filename = "/home/luiz/ZED_WS/build/zed_package/Nuvem_2_Cameras.ply";
    pcl::io::savePLYFileASCII(filename, *nuvem_acumulada);
    cout << "Salvo!" <<endl;
  }
  if(cont2 == 1)
  {
    cout << "Salvando..." <<endl;
    string filename = "/home/luiz/ZED_WS/build/zed_package/Nuvem_RGBD.ply";
    pcl::io::savePLYFileASCII(filename, *nuvem_acumulada);
    cout << "Salvo!" <<endl;
  }
  if(cont2 == 2)
  {
    cout << "Salvando..." <<endl;
    string filename = "/home/luiz/ZED_WS/build/zed_package/Nuvem_ZED.ply";
    pcl::io::savePLYFileASCII(filename, *nuvem_acumulada);
    cout << "Salvo!" <<endl;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_color(pcl::PointCloud<PointT>::Ptr cloud_in){

  // Limites RGB
  int rMax = 200;
  int rMin = 0;
  int gMax = 120;
  int gMin = 0;
  int bMax = 120;
  int bMin = 0;

  pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT> ());
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::LT, rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::GT, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::LT, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::GT, gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::LT, bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::GT, bMin)));

  // Constroi o filtro
  pcl::ConditionalRemoval<PointT> condrem (color_cond);
  condrem.setInputCloud (cloud_in);
  condrem.setKeepOrganized(true);

  // Aplica o filtro
  condrem.filter(*cloud_in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(pcl::PointCloud<PointT>::Ptr in, std::string field, float min, float max){
  pcl::PassThrough<PointT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void voxelgrid_nuvem(PointCloud<PointT>::Ptr in, float lf){
  VoxelGrid<PointT> grid;
  grid.setLeafSize(lf, lf, lf);
  grid.setInputCloud(in);
  grid.filter(*in);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////                                            ICP                                                       ///////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void icp(const pcl::PointCloud<PointT>::Ptr icp_src,
         const pcl::PointCloud<PointT>::Ptr icp_tgt,
               pcl::PointCloud<PointT>::Ptr icp_final,
               Eigen::Matrix4f T)
{
  ROS_INFO("Entrando no ICP");
  pcl::IterativeClosestPoint<PointT, PointT> registration;
  pcl::PointCloud<PointT>::Ptr src (new pcl::PointCloud<PointT>());
  registration.setUseReciprocalCorrespondences(true);
  *src = *icp_src;
  for(int i = 0; i < 1; i++)
  {
//    registration.setMaxCorrespondenceDistance(0.1);
//    registration.setMaxCorrespondenceDistance(0.0009);
    registration.setMaximumIterations(700);
    registration.setRANSACIterations(700);
    registration.setTransformationEpsilon(1*1e-9);
    registration.setEuclideanFitnessEpsilon(0.00000000001);
    registration.setInputCloud(src);
    registration.setInputTarget(icp_tgt);
    registration.align(*icp_final, T);
    T = registration.getFinalTransformation()*T;
    transformPointCloud(*src, *src, T);
  }

  std::cout << "Convergiu: " << registration.hasConverged() << " score: " <<
               registration.getFitnessScore() << std::endl;
  std::cout << registration.getFinalTransformation() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////// Callback acumulador ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void acumular_nuvem(const sensor_msgs::PointCloud2ConstPtr& msg_ptc_ef, const sensor_msgs::PointCloud2ConstPtr& msg_ptc, const OdometryConstPtr& msg_odo){
  // Variaveis locais
  PointCloud<PointT>::Ptr nuvem_ef (new PointCloud<PointT>());
  PointCloud<PointT>::Ptr nuvem_z (new PointCloud<PointT>());
  PointCloud<PointT>::Ptr nuvem_transformada (new PointCloud<PointT>());
  PointCloud<PointT>::Ptr reg_result (new PointCloud<PointT>());
  sensor_msgs::PointCloud2 nuvem_mensagem;

  int cont = 1; // variável para escolha da acumulação
  cont2 = cont;
  if(nuvem_acumulada->points.size()<1025000) // Limitação de Tamanho da nuvem para não saturar ERA 1600000
  {
    cout << "Acumulando " << endl;

    //////// TESTE TF
    ros::Time tic = ros::Time::now();
    ros::Time t = msg_ptc->header.stamp;
    tf::StampedTransform trans;
    try
    {
      p_listener->waitForTransform(ros::names::remap("camera_rgb_optical_frame"), ros::names::remap("zed_current_frame"), t, ros::Duration(3.0));
      p_listener->lookupTransform(ros::names::remap("camera_rgb_optical_frame"), ros::names::remap("zed_current_frame"), t, trans);
    }
    catch (tf::TransformException& ex){
//      ROS_ERROR("%s",ex.what());
//      //ros::Duration(1.0).sleep();
//      ROS_WARN("Falhou!");
      return;
    }
    Eigen::Affine3d eigen_trf;
    tf::transformTFToEigen(trans, eigen_trf);
//    cout << "T \n" << eigen_trf.translation() <<"\n"<< endl;
//    cout << "R \n" << eigen_trf.rotation() << endl;
    /////////////////////

    // Converter a mensagem em nuvem
    fromROSMsg (*msg_ptc_ef, *nuvem_ef);
//    fromROSMsg (*msg_ptc, *nuvem_z);

    // Remover NaN se existir
    vector<int> indicesNAN;
    removeNaNFromPointCloud(*nuvem_ef, *nuvem_ef, indicesNAN);
//    removeNaNFromPointCloud(*nuvem_z, *nuvem_z, indicesNAN);

    eigen_trf = eigen_trf.inverse();
    transformPointCloud<PointT>(*nuvem_ef, *nuvem_ef, eigen_trf);
    transformPointCloud<PointT>(*nuvem_ef, *nuvem_transformada, Tc);

    //// Filtrar a regiao da nuvem que passa
    passthrough(nuvem_transformada, "z", -3,  2.2); // Vertical
    passthrough(nuvem_transformada, "x",  0, 5.2); // Profundidade
    passthrough(nuvem_transformada, "y", -3,  2.1); // Horizontal

    /// Obter a odometria da mensagem
    // Rotacao
    Eigen::Quaternion<float> q;
    q.x() = (float)msg_odo->pose.pose.orientation.x;
    q.y() = (float)msg_odo->pose.pose.orientation.y;
    q.z() = (float)msg_odo->pose.pose.orientation.z;
    q.w() = (float)msg_odo->pose.pose.orientation.w;
    // Translacao
    Eigen::Vector3f offset(msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z);
    // Print para averiguar
    if(false){
      auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      cout << "Roll: " << RAD2DEG(euler[0]) << "\tPitch: " << RAD2DEG(euler[1]) << "\tYaw: " << RAD2DEG(euler[2]) << endl;
      cout << "X   : " << offset(0)         << "\tY    : " << offset(1)         << "\tZ  : " << offset(2)         << endl;
    }

    Eigen::Matrix3f R = q.matrix();//cout << "R " << R << endl;
    Eigen::MatrixXf T(3,1);
    T = offset;//cout << "T " << T << endl;
    Eigen::Matrix4f RT;
    RT << R,T,
          0,0,0,1;

    cout << "RT " << RT << endl;

    //    (*nuvem_acumulada) = *nuvem_z + *nuvem_ef;
    //     Início da acumulação
    switch (cont)
    {
//    case 0: // Acumula as duas nuvens
//      if(nuvem_acumulada->points.empty())
//      {
//        //       Com a transformação Tc, levo a nuvem do EF para o frame da ZED
//        (*nuvem_acumulada) = *nuvem_z + *nuvem_transformada;
//        nuvem_transformada.reset();
//      }
//      else
//      {
//        *nuvem_z = *nuvem_z + *nuvem_transformada;
//        //    nuvem_transformada.reset();
//        //    cout << "Transformação para frame map " << eigen_trf.affine() << endl;
//        // Transformar a nuvem
//        transformPointCloud<PointT>(*nuvem_z, *nuvem_transformada, offset, q);

//        // Acumular a nuvem de forma simples
//        ROS_INFO("Tamanho da nuvem atual = %ld", nuvem_transformada->points.size());
//        (*nuvem_acumulada) += (*nuvem_transformada);
//        ROS_INFO("Tamanho da nuvem acumulada = %ld", nuvem_acumulada->points.size());
//      }break;

    case 1: // Só RGBD
      if(nuvem_acumulada->points.empty())
      {
        (*nuvem_acumulada) = *nuvem_transformada;
        (*nuvem_aux) = *nuvem_transformada;

//        nuvem_transformada.reset();
      }
      else
      {
//        s = s++;
        // Remover NAN e Transformar a nuvem
//        if(s > 2)
//        {
//          removeNaNFromPointCloud(*nuvem_transformada, *nuvem_transformada, indicesNAN);
//          s = 0;
//        }
//        transformPointCloud<PointT>(*nuvem_transformada, *nuvem_transformada, offset, q);
//        tstart = time(0);
        icp(nuvem_transformada, nuvem_aux, reg_result, RT); // Aplicando icp com chute inicial da odom
        transformPointCloud<PointT>(*nuvem_transformada, *nuvem_transformada, RT);

        // Acumular a nuvem de forma simples
        ROS_INFO("Tamanho da nuvem atual = %ld", nuvem_transformada->points.size());
        (*nuvem_acumulada) += (*nuvem_transformada);
        ROS_INFO("Tamanho da nuvem acumulada = %ld", nuvem_acumulada->points.size());
        (*nuvem_aux) = *nuvem_transformada;
        nuvem_transformada.reset();

//        tend = time(0);
//        duration = tend - tstart;
//        cout << "Tempo de duração com icp " << duration << " segundos."<< endl;
      }break;

//    case 2: // Só ZED
//      if(nuvem_acumulada->points.empty())
//      {
//        //       Com a transformação Tc, levo a nuvem do EF para o frame da ZED
//        (*nuvem_acumulada) = *nuvem_z;
//        nuvem_transformada.reset();
//      }
//      else
//      {
////        *nuvem_z = *nuvem_z;
//        //    nuvem_transformada.reset();
//        //    cout << "Transformação para frame map " << eigen_trf.affine() << endl;
//        // Transformar a nuvem
//        transformPointCloud<PointT>(*nuvem_z, *nuvem_transformada, offset, q);

//        // Acumular a nuvem de forma simples
//        ROS_INFO("Tamanho da nuvem atual = %ld", nuvem_transformada->points.size());
//        (*nuvem_acumulada) += (*nuvem_transformada);
//        ROS_INFO("Tamanho da nuvem acumulada = %ld", nuvem_acumulada->points.size());
//      }break;
    }    

    // Converter de volta a ros msg e enviar
    toROSMsg(*nuvem_acumulada, nuvem_mensagem);
    nuvem_mensagem.header.stamp = msg_ptc->header.stamp; // Mesmo tempo para sincronizar
    pub->publish(nuvem_mensagem);

    nuvem_z.reset();
//    nuvem_transformada.reset();
  }
  else
  {
    //// Simplificar a nuvem por voxel grid
    //      float tamanho_leaf = 0.000005;
    //      voxelgrid_nuvem(nuvem_acumulada, tamanho_leaf);
    //// Filtrar a regiao da nuvem que passa
    //      passthrough(nuvem_z, "z", -1,  1); // Vertical
    //      passthrough(nuvem_z, "x",  1, 10); // Profundidade
    //      passthrough(nuvem_z, "y", -1,  1); // Horizontal
    //// Filtrar por cor
    //      filter_color(cloud);
////     Remover outliers
//    remove_outlier(nuvem_acumulada, 20, 2);

    // Converter de volta a ros msg e enviar
    toROSMsg(*nuvem_acumulada, nuvem_mensagem);
    nuvem_mensagem.header.stamp = msg_ptc->header.stamp; // Mesmo tempo para sincronizar
    pub->publish(nuvem_mensagem);
    vector<int> indicesNAN2;
    removeNaNFromPointCloud(*nuvem_acumulada,*nuvem_acumulada, indicesNAN2);
    salvaNuvem();

    tend = time(0);
    duration = tend - tstart;
    cout << "Tempo de duração " << duration << " segundos."<< endl;

    nuvem_z.reset();
//    nuvem_transformada.reset();
    nuvem_acumulada->clear();

//    ros::shutdown();
    flag = 1;

  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// Principal //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "acumular_zed_node");
  ros::NodeHandle nh;
  tstart = time(0);
  cout << "COMEÇANDO!!" << endl;
//  while(ros::ok())
//  {
    // Iniciando a listener para a transformada que vem da odometria da ZED
    p_listener = (tf::TransformListener*) new tf::TransformListener;
    ros::Duration(2).sleep();

    // Iniciando nuvem acumulada com o frame segundo vindo do parametro
    nuvem_acumulada = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    nuvem_acumulada->header.frame_id = ros::names::remap("/camera_rgb_optical_frame");
    nuvem_aux =  (PointCloud<PointT>::Ptr) (new PointCloud<PointT>());

    // Iniciar o publisher no topico da nuvem acumulada
    pub  = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
    *pub = nh.advertise<sensor_msgs::PointCloud2>("/zed_neymar", 100);


    Tc<<  1,              0,           0,          -0.001,
          0,              1,           0,          -0.090,
          0,              0,           1,          -0.025,
          0,              0,           0,              1; // MATRIZ DA TRANSFORMAÇÂO DA CALIBRAÇÃO

    //  cout << "Estou aqui 1" << endl;
    // Subscriber para a nuvem do elastic fusion, nuvem instantanea da zed e odometria zed
//    message_filters::Subscriber<sensor_msgs::PointCloud2> subptc_ef(nh, "/camera/depth_registered/points"  , 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subptc_ef(nh, "/color_map"  , 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subptc   (nh, "/zed/point_cloud/cloud_registered", 10);
    message_filters::Subscriber<Odometry>                 subodo   (nh, "/odom2" , 10);
    // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
    Synchronizer<syncPolicy> sync(syncPolicy(150), subptc_ef, subptc, subodo);
    sync.registerCallback(boost::bind(&acumular_nuvem, _1, _2, _3));
//  }

  while(ros::ok())
  {
    ros::spinOnce();
    if(flag == 1)
    {
      ros::shutdown();
      break;
    }
  }
  ros::shutdown();
  return 0;
}
