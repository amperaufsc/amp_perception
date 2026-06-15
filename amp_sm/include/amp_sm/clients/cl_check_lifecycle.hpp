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
    ClCheckLifecycle() 
        : smacc2::client_bases::SmaccServiceClient<lifecycle_msgs::srv::ChangeState>("/float_publisher/change_state")
    {
    }
    void async_change_state(std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request)
    {
        this->client_->async_send_request(request);
    }
};
} // namespace amp_sm