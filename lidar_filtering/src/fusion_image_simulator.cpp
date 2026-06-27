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
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace message_filters;
typedef sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> MySyncPolicy;

class PointCloudHandler : public rclcpp::Node {
public:
  PointCloudHandler() : rclcpp::Node("pcl_transform_from_yaml")
  , sub_pointcloud{this, "/fsds/lidar/Lidar2", rmw_qos_profile_sensor_data}
  , sub_image{this, "/fsds/cam2/image_color", rmw_qos_profile_sensor_data} 
  {
    sync_ = std::make_shared<Synchronizer<MySyncPolicy>>(
      MySyncPolicy(100), sub_pointcloud, sub_image);
    sync_->registerCallback(
      std::bind(&PointCloudHandler::cloud_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    pub_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_pub", 10);
    pub_image = this->create_publisher<sensor_msgs::msg::Image>("image_lidar", 10);

    std::string path_intrinsic = ament_index_cpp::get_package_share_directory("lidar_filtering") + "/config/intrinsic_simulator.yaml";
    YAML::Node config_intrinsic = YAML::LoadFile(path_intrinsic);
    std::string path_extrinsinc = ament_index_cpp::get_package_share_directory("lidar_filtering") + "/config/extrinsic_simulator.yaml";
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
    
    // Corrige a rotação padrão LiDAR → Camera optical frame
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
                      , const std::shared_ptr<const sensor_msgs::msg::Image> image_msg) {
  
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*pointcloud_msg, *cloud_in);
      
      //declara uma pointcloud vazia que vai ser a que será publicada
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>()); 
      cloud_filt->header   = cloud_in->header;   // mantém frame_id, stamp
      cloud_filt->is_dense = cloud_in->is_dense; //mantem is_dense

      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");        
      
      for (const auto& pt : cloud_in->points) {
        // if (pt.x >= 0 && pt.z >= 0){ // isso eh pra filtrar se o ponto eh da frente do carro pra n ter chance de colocar no plano um ponto que estava la atras
        Eigen::Vector4f X(pt.x, pt.y, pt.z, 1.0f); // ponto em coordenadas homogêneas
        Eigen::Vector3f Y = camera_matrix * X;     // aplica P * R_rect * RT

        if (Y(2) <= 0) continue;
        float u = Y(0) / Y(2);
        float v = Y(1) / Y(2);
      
        RCLCPP_INFO(this->get_logger(), "u = %f  v = %f", u, v);

        if (u >= 0 && u <= static_cast<float>((image_msg->width)) && 
             v >= 0 && v < static_cast<float>(image_msg->height)) {
              std::cout<< "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
              cloud_filt->points.push_back(pt); //se estiver dentro dos limites da camera, preenche a pointcloud
              cv::circle(cv_ptr->image, cv::Point(static_cast<int>(u), static_cast<int>(v)), 1,cv::Scalar(0, 255, 0), -1);
        }        
      }
      cloud_filt->width  = static_cast<uint32_t>(cloud_filt->points.size());
      cloud_filt->height = 1;
      RCLCPP_INFO(this->get_logger(), "PointCloud recebida com %zu pontos", cloud_filt->points.size());

      // CONVERSAO PCL PARA ROS2 POINTCLOUD
      sensor_msgs::msg::PointCloud2 out_msg;
      pcl::toROSMsg(*cloud_filt, out_msg);
      out_msg.header = pointcloud_msg->header;
      pub_pointcloud->publish(out_msg);

      // CONVERSAO OPENCV PRA ROS2 IMAGE
      auto image_msg_painted = cv_ptr->toImageMsg();
      pub_image->publish(*image_msg_painted);
    }

  Eigen::Matrix4f RT;
  Eigen::Matrix<float, 3, 4> camera_matrix; 
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pointcloud;
  message_filters::Subscriber<sensor_msgs::msg::Image>sub_image;
  std::shared_ptr<Synchronizer<MySyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudHandler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}