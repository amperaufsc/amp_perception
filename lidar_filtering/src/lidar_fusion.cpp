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

#define IMAGE_WIDTH 768
#define IMAGE_HEIGHT 480

struct Cluster {
    std::vector<pcl::PointXYZ> points;
};

using namespace message_filters;
typedef sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    yolov8_msgs::msg::Yolov8Inference> MySyncPolicy;

class PointCloudHandler : public rclcpp::Node {
public:
  PointCloudHandler() : rclcpp::Node("pcl_transform_from_yaml")
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

    std::string path_intrinsic = "/home/lucasmoro/ws/src/amp_perception/lidar_filtering/config/matrix_intrinsic.yaml";  // substitua pelo caminho real
    YAML::Node config_intrinsic = YAML::LoadFile(path_intrinsic);
    std::string path_extrinsinc = "/home/lucasmoro/ws/src/amp_perception/lidar_filtering/config/matrix_extrinsic.yaml"; 
    YAML::Node config_extrinsic = YAML::LoadFile(path_extrinsinc);

    auto rot_data = config_extrinsic["rotation_matrix"]["data"].as<std::vector<float>>();
    auto trans_data = config_extrinsic["translation_matrix"]["data"].as<std::vector<float>>();
    auto rrect_data = config_intrinsic["rectification_matrix"]["data"].as<std::vector<float>>();
    auto proj_data = config_intrinsic["projection_matrix"]["data"].as<std::vector<float>>();

    Eigen::Matrix4f R_rect;
    for (int i = 0; i < 16; ++i)
      R_rect(i / 4, i % 4) = rrect_data[i];

    Eigen::Matrix<float, 3, 4> P;
    for (int i = 0; i < 12; ++i)
      P(i / 4, i % 4) = proj_data[i];

    Eigen::Matrix3f R;
    for (int i = 0; i < 9; ++i)
      R(i / 3, i % 3) = rot_data[i];

    Eigen::Vector3f t;
    for (int i = 0; i < 3; ++i)
      t(i) = trans_data[i];

    RT = Eigen::Matrix4f::Identity();
    RT.block<3,3>(0,0) = R;
    RT.block<3,1>(0,3) = t;
    
    // SE O LIDAR FOR VELODYNE O 1 DA SEGUNDA LINHA DEVE SER NEGATIVO
    // SE FOR OUSTER DEVE SER POSITIVO
    Eigen::Matrix4f lidar_to_cam_fix;
    lidar_to_cam_fix <<
        0, -1,  0, 0,
        0,  0,  -1, 0,
        1,  0,  0, 0,
        0,  0,  0, 1;

    // Aplica essa rotação adicional
    RT = lidar_to_cam_fix * RT;
    camera_matrix = P * R_rect * RT;

    RCLCPP_INFO(this->get_logger(), "Transform loaded from YAML.");
    std::cout<<RT<<std::endl;
  }

private:
    void cloud_callback(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> pointcloud_msg
                      , const std::shared_ptr<const yolov8_msgs::msg::Yolov8Inference> inference_msg) {
      
      int iterador = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*pointcloud_msg, *cloud_in);
      
      //declara uma pointcloud vazia que vai ser a que será publicada
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>()); 
      cloud_filt->header   = cloud_in->header;   // mantém frame_id, stamp
      cloud_filt->is_dense = cloud_in->is_dense; //mantem is_dense

      //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");        
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>());
      cloud_final->header   = cloud_in->header;   // mantém frame_id, stamp
      cloud_final->is_dense = cloud_in->is_dense; //mantem is_dense

      //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");  
      
      //std::vector<uint8_t> color_bin;                  
      fs_msgs::msg::TrackStamped track_final;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux(new pcl::PointCloud<pcl::PointXYZ>());;
    
      //RCLCPP_INFO(this->get_logger(), "Imagem: %d x %d", image_msg->width, image_msg->height);
      cloud_final->points.clear();
      for (const auto& inf : inference_msg->yolov8_inference) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ highest_point;
        bool first = true;
        fs_msgs::msg::Cone cone;

        for (const auto& pt : cloud_in->points) {
            Eigen::Vector4f X(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector3f Y = camera_matrix * X;
            
            // SE O LIDAR FOR VELODYNE DEVE SER '<=' 
            // SE FOR OUSTER DEVE SER '=>'
            if (Y(2) <= 0) continue;
            float u = Y(0) / Y(2);
            float v = Y(1) / Y(2);

            if (u >= inf.top && u <= inf.bottom && 
                v >= inf.left && v <= inf.right) {

                cloud_filt->points.push_back(pt);
                if (first || pt.z > highest_point.z) {
                    highest_point = pt;
                    first = false;
                }
            }
        }

        for (const auto& point : cloud_filt->points) {
          if (point.z >= highest_point.z - 0.02){
            cloud_final->points.push_back(point);
            cloud_aux->points.push_back(point);
          }
        }
        cone = clusterize(cloud_aux, inf.class_name);
        if (cone.color != fs_msgs::msg::Cone::UNKNOWN){
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
    
    // CONVERSAO OPENCV PRA ROS2 IMAGE
    // auto image_msg_painted = cv_ptr->toImageMsg();
    // pub_image->publish(*image_msg_painted);
  }

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
          return 0.0;  // ou trate como quiser

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
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pointcloud;
  message_filters::Subscriber<yolov8_msgs::msg::Yolov8Inference> sub_inference;
  std::shared_ptr<Synchronizer<MySyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
  rclcpp::Publisher<fs_msgs::msg::TrackStamped>::SharedPtr pub_track;
  //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// if (u >= inf.top + (inf.bottom-inf.top)/3 && u <= inf.bottom - (inf.bottom-inf.top)/3  && 
//           v >= inf.left + (inf.right - inf.left)/2 && v <= inf.right) 
//136

// cone.position.x = highest_point.x;
//     cone.position.y = highest_point.y;
//     cone.position.z = highest_point.z;
//     cone.color = (inf.class_name == "yellow_cone")
//                    ? fs_msgs::msg::Cone::YELLOW
//                    : fs_msgs::msg::Cone::BLUE;

//     track.track.push_back(cone);