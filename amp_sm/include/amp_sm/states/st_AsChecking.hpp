#pragma once

#include <smacc2/smacc.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <ros_timer_client/cl_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include "../client_behaviors/cb_change_lifecycle.hpp"
#include <amp_sm/clients/cl_check_lifecycle.hpp>
#include <amp_sm/clients/cl_topic_listener.hpp>

namespace amp_sm
{
struct st_AsReady;

struct st_AsChecking : smacc2::SmaccState<st_AsChecking, Amp_sm>
{
    using SmaccState::SmaccState;

    typedef smacc2::Transition<amp_sm::EvCheckListener, amp_sm::st_AsReady
    > reactions;

    static void staticConfigure()
    {
        // 1. Envia o sinal Activate para o serviço /.../change_state
        configure_orthogonal<or_check, CbChangeLifecycle<ClCheckLifecycle>>(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE
        );
  }
    
    void onEntry()
    {
        RCLCPP_INFO(getLogger(), "Estado checking: Inicializando...");
    }
    
    void onExit()
    {   
        RCLCPP_INFO(getLogger(), "Saindo do Estado checking...");
    }
};
} // namespace amp_sm