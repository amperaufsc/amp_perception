#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_service_client.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

#include <keyboard_client/cl_keyboard.hpp>
#include <ros_timer_client/cl_ros_timer.hpp>
#include <check_sm/clients/cl_can_topic.hpp>
namespace check_sm
{
class or_check : public smacc2::Orthogonal<or_check>
{
public:
    void onInitialize() override
    {
        // 1. CLIENTE DE CICLO DE VIDA (O ALVO ESPECÍFICO)
        // Substitua "/nome_do_seu_no_especifico" pelo nome real do nó 
        // que você quer ativar (ex: "/yolo_perception_node", "/lidar_driver", etc.)
        this->createClient<
            smacc2::client_bases::SmaccServiceClient<lifecycle_msgs::srv::ChangeState>
        >("/depthai_position_estimator/change_state");

        // 2. Cliente do Temporizador (15 segundos)
        this->createClient<cl_ros_timer::ClRosTimer>(
            rclcpp::Duration(std::chrono::seconds(15))
        );

        // 4. Cliente da CAN
        this->createClient<check_sm::ClCanTopic>();
    }
};
} // namespace check_sm