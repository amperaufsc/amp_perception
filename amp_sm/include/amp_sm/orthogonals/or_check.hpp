#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_service_client.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

#include <ros_timer_client/cl_ros_timer.hpp>
#include <amp_sm/clients/cl_topic_listener.hpp>
#include <amp_sm/clients/cl_lifecycle_monitor.hpp>
#include <amp_sm/clients/cl_check_lifecycle.hpp>

namespace amp_sm
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
        >("/float_publisher/change_state");

        this->createClient<
            smacc2::client_bases::SmaccServiceClient<lifecycle_msgs::srv::ChangeState>
        >("/lifecycle_perception_node/change_state");

        this->createClient<amp_sm::ClLifecycleMonitor>(std::vector<std::string>{
            "/float_publisher",
            //...
        });

        this->createClient<amp_sm::ClCheckLifecycle>();
        this->createClient<amp_sm::ClStartListener>();
        this->createClient<amp_sm::ClCheckListener>();
        this->createClient<amp_sm::ClMissionSelectListener>();
        this->createClient<amp_sm::ClStartMissionListener>();
    }
};
} // namespace amp_sm