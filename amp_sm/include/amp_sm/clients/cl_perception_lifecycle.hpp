#pragma once

#include <smacc2/client_bases/smacc_service_client.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

namespace amp_sm
{
/**
 * @brief Cliente exclusivo para gerenciar o ciclo de vida do subsistema de Check.
 */
class ClCheckLifecycle : public smacc2::client_bases::SmaccServiceClient<lifecycle_msgs::srv::ChangeState>
{
public:
    // ATENÇÃO: Substitua "/check_node" pelo namespace/nome real que você deu a esse nó no seu launch file
    ClCheckLifecycle() 
        : smacc2::client_bases::SmaccServiceClient<lifecycle_msgs::srv::ChangeState>("/lifecycle_perception_node/change_state")
    {
    }
};
} // namespace amp_sm