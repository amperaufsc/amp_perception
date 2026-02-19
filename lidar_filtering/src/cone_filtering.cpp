#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <functional> // sei nao
#include <stdio.h>
#include "yolov8_msgs/msg/yolov8_inference.hpp"
#include "fs_msgs/msg/track_stamped.hpp"
#include "fs_msgs/msg/cone.hpp"
#include <vector>
#include <cmath>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// LOGICA DA FUSÃO ENTRE CAMERA E LIDAR BASEADO INTEIRAMENTE NO ARTIGO ABAIXO:
// TechLabs Aachen - Visual & LiDAR-based Tracking of Traffic Cones (2021)
// https://techlabs-aachen.medium.com/visual-lidar-based-tracking-of-traffic-cones-20e83f6067f8

#define IMAGE_WIDTH 768
#define IMAGE_HEIGHT 480
#define MAX_DISTANCE 0.02

using namespace message_filters;

// cria um apelido menor pro tipo (MySyncPolicy)
typedef sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    yolov8_msgs::msg::Yolov8Inference> MySyncPolicy; 


class PointCloudHandler : public rclcpp::Node {
public:

  // Construtor 
  PointCloudHandler() : rclcpp::Node("lidar_fusion")
  , sub_pointcloud{this, "/velodyne_points", rmw_qos_profile_sensor_data}
  , sub_inference{this, "/Yolov8_Inference", rmw_qos_profile_sensor_data} 

  {
    sync_ = std::make_shared<Synchronizer<MySyncPolicy>>(
      MySyncPolicy(1000), sub_pointcloud, sub_inference);
    sync_->registerCallback(
      std::bind(&PointCloudHandler::cloud_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    
    pub_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_pub", 10);
    pub_track = this->create_publisher<fs_msgs::msg::TrackStamped>("track_lidar", 10);

    // Carrega os yaml's
    std::string path_intrinsic = ament_index_cpp::get_package_share_directory("lidar_filtering") + "/config/matrix_intrinsic.yaml";
    YAML::Node config_intrinsic = YAML::LoadFile(path_intrinsic);
    std::string path_extrinsinc = ament_index_cpp::get_package_share_directory("lidar_filtering") + "/config/matrix_extrinsic.yaml";
    YAML::Node config_extrinsic = YAML::LoadFile(path_extrinsinc);

    // Carrega cada matriz especificada do yaml
    auto rot_data = config_extrinsic["rotation_matrix"]["data"].as<std::vector<float>>();
    auto trans_data = config_extrinsic["translation_matrix"]["data"].as<std::vector<float>>();
    auto rrect_data = config_intrinsic["rectification_matrix"]["data"].as<std::vector<float>>();
    auto proj_data = config_intrinsic["projection_matrix"]["data"].as<std::vector<float>>();


    // Carrega as matrizes em matrizes da biblioteca Eigen, permitindo manipulação 
    Eigen::Matrix4f R_rect;
    for (int i = 0; i < 16; ++i)
      R_rect(i / 4, i % 4) = rrect_data[i]; // Matriz de rectificação

    Eigen::Matrix<float, 3, 4> P;
    for (int i = 0; i < 12; ++i)
      P(i / 4, i % 4) = proj_data[i]; // Matriz de projeção

    Eigen::Matrix3f R;
    for (int i = 0; i < 9; ++i)
      R(i / 3, i % 3) = rot_data[i]; // Matriz de rotação

    for (int i = 0; i < 3; ++i)
      t(i) = trans_data[i]; // Matriz de translação

    // Cria a matriz RT (Concatenação de R e T)
    RT = Eigen::Matrix4f::Identity();
    RT.block<3,3>(0,0) = R;
    RT.block<3,1>(0,3) = t;
    
    // Corrige a rotação padrão LiDAR → Camera optical frame
    Eigen::Matrix4f lidar_to_cam_fix;
    lidar_to_cam_fix <<
        0, -1,  0, 0,
        0,  0,  -1, 0,
        1,  0,  0, 0,
        0,  0,  0, 1;

    RT = lidar_to_cam_fix * RT;

    // Matriz final
    camera_matrix = P * R_rect * RT;
  }

private:
    // Callback principal. Recebe uma pointcloud crua do LiDAR e uma inferencia
    void cloud_callback(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> pointcloud_msg
                      , const std::shared_ptr<const yolov8_msgs::msg::Yolov8Inference> inference_msg) {

      // Transforma a mensagem ROS2 da pointcloud em uma pointcloud da biblioteca pcl
      // permitindo quaisquer manipulações na pointcloud                  
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*pointcloud_msg, *cloud_in);
      
      //declara uma pointcloud (pcl) vazia que vai ser a que será publicada
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>());
      cloud_final->header   = cloud_in->header;   // mantém frame_id, stamp
      cloud_final->is_dense = cloud_in->is_dense; //mantem is_dense
      
      // mensagem de track que sera publicada
      fs_msgs::msg::TrackStamped track_final;

      // pointcloud pcl auxiliar (explicação esta mais abaixo)
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux(new pcl::PointCloud<pcl::PointXYZ>());;
    
      for (const auto& inf : inference_msg->yolov8_inference) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ highest_point;
        bool first = true;

        for (const auto& pt : cloud_in->points) {
            Eigen::Vector4f X(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector3f Y = camera_matrix * X; // Pointcloud no espaço 2D
            if (Y(2) <= 0) continue; // TEM QUE MANTER ISSO
            
            // Normaliza os pontos
            float u = Y(0) / Y(2);
            float v = Y(1) / Y(2);
            
            // Se esta dentro dos limites da BoundingBox o ponto entra na pointcloud filtrada
            // Se esse ponto for o primeiro a ser analisado no loop ou a altura dele (eixo z)
            // for maior que a variavel highest_point (que guarda o ponto mais alto registrado ate o momento do loop)
            // entao esse ponto sera o novo highest_point 
            // O objetivo disso é guardar o maior ponto z dos pontos projetados no cone que esta sendo processado
            if (u >= inf.top && u <= inf.bottom && 
                v >= inf.left && v <= inf.right) {

                cloud_filt->points.push_back(pt);
                if (first || pt.z > highest_point.z) {
                    highest_point = pt;
                    first = false;
                }
            }
        }

        // Para cada ponto da pointcloud filtrada resultante, se o ponto estiver
        // no maximo MAX_DISTANCE centrimetros abaixo do do maior ponto (highest_point)
        // entao ele é considerado e adicionado na pointcloud final
        // Isso é uma maneira de filtrar pontos que estão sendo de fato projetados no cone
        // assim evita pontos que estao nos limites da boundingbox mas estao 
        // sendo projetados no chão e nao no cone
        for (const auto& point : cloud_filt->points) {
          
          if (point.z >= highest_point.z - MAX_DISTANCE) {

            cloud_final->points.push_back(point);
            cloud_aux->points.push_back(point);
          }
        }

        // Cria um cone, que recebe o retorno da função que clusteriza a pointcloud auxiliar
        // essa pointcloud auxiliar serve pra receber os pontos de um cone so. Ja a cloud_final
        // recebe todos os pontos filtrados de todos os cones ate o momento
        // por isso se usa a auxiliar que recebe os pontos de um cone so, no caso, o que esta
        // sendo processado no momento, por isso tambem que é chamado a funcao clear() no final do loop da inference
        fs_msgs::msg::Cone cone = clusterize(cloud_aux, inf.class_name);
        
        if (cone.color != fs_msgs::msg::Cone::UNKNOWN){


          //MUDA A TRACK PRO FRAME DA CAMERA EM VEZ DO LIDAR  
          Eigen::Vector4f p_l(cone.location.x, cone.location.y, cone.location.z, 1.0f);
          Eigen::Vector4f p_c = RT * p_l;   

          cone.location.x = p_c(0);
          cone.location.y = p_c(1);
          cone.location.z = p_c(2);


          track_final.track.push_back(cone);
        }
        cloud_aux->points.clear();
      }

    cloud_final->width  = static_cast<uint32_t>(cloud_final->points.size());
    cloud_final->height = 1;
    RCLCPP_INFO(this->get_logger(), "PointCloud recebida com %zu pontos", cloud_final->points.size());

    // CONVERSAO PCL PARA ROS2 POINTCLOUD
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud_final, out_msg);
    out_msg.header = pointcloud_msg->header;
    pub_pointcloud->publish(out_msg);

    pub_track->publish(track_final);
    
  }

  // Essa função recebe uma pointcloud filtrada de um cone
  // e a cor detectada do cone pela YOLO.
  // É feita a mediana de todos os eixos entre os pontos da cloud_aux
  // Dessa forma é criado um cone de output com a cor detectada e uma localização propria
  // Essa função é uma das que talvez mais precise de manutenção futura pois é muito simples 
  // e provavelmente não efetiva, outra coisa é a questão do eixo z (de altura) talvez não deva
  // ser feito dessa forma e sim apenas os eixos 2D (x,y)
  fs_msgs::msg::Cone clusterize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_aux,
    const std::string& cone_class)
  {
      fs_msgs::msg::Cone cone_out;

      // Se não tem ponto, retorna cone UNKNOWN em (0,0,0)
      if (cloud_aux->points.size() <= 0) {
          cone_out.color = fs_msgs::msg::Cone::UNKNOWN;
          return cone_out;
      }

      float mx = mediana_coord(cloud_aux, 'x');
      float my = mediana_coord(cloud_aux, 'y');
      float mz = mediana_coord(cloud_aux, 'z');
      // Preenche cone_out
      cone_out.location.x = mx;
      cone_out.location.y = my;
      cone_out.location.z = mz;

      if (mx == 0.0 || my == 0.0 || mz == 0.0){
        cone_out.color = fs_msgs::msg::Cone::UNKNOWN;
        return cone_out;
      }

      if (cone_class == "yellow_cone")
          cone_out.color = fs_msgs::msg::Cone::YELLOW;
      else if (cone_class == "blue_cone")
          cone_out.color = fs_msgs::msg::Cone::BLUE;
      else
          cone_out.color = fs_msgs::msg::Cone::UNKNOWN;

      return cone_out;
  }

  // funcao chamada dentro de clusterize que de fato faz a mediana de cada eixo
  double mediana_coord(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, char coord) {
      std::vector<float> vals;
      vals.reserve(cloud->size());

      for (const auto& p : cloud->points) {
          switch (coord) {
              case 'x': vals.push_back(p.x); break;
              case 'y': vals.push_back(p.y); break;
              case 'z': vals.push_back(p.z); break;
              default: throw std::runtime_error("coord inválido (use 'x', 'y' ou 'z')");
          }
      }

      if (vals.empty())
          return 0.0;

      std::sort(vals.begin(), vals.end());

      int n = vals.size();
      if (n % 2 == 1) {
          return vals[n / 2];
      } else {
          return (vals[n/2 - 1] + vals[n/2]) / 2.0;
      }
  }

  Eigen::Matrix4f RT;
  Eigen::Matrix<float, 3, 4> camera_matrix; 
  Eigen::Vector3f t;
  
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pointcloud;
  message_filters::Subscriber<yolov8_msgs::msg::Yolov8Inference> sub_inference;
  
  std::shared_ptr<Synchronizer<MySyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
  rclcpp::Publisher<fs_msgs::msg::TrackStamped>::SharedPtr pub_track;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}