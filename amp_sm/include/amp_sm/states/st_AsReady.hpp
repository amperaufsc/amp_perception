#pragma once

#include <smacc2/smacc.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

// 1. INCLUDES NECESSÁRIOS
#include <amp_sm/client_behaviors/cb_change_lifecycle.hpp>
#include <amp_sm/clients/cl_check_lifecycle.hpp>
// Substitua pelo cliente real que você quer configurar primeiro
#include <amp_sm/clients/cl_topic_listener.hpp> 

namespace amp_sm
{
struct st_AsChecking;

struct st_AsReady : smacc2::SmaccState<st_AsReady, Amp_sm>
{
    using SmaccState::SmaccState;

    typedef boost::mpl::list<
    smacc2::Transition<amp_sm::EvCheckListener, amp_sm::st_AsChecking>
    > reactions;

    static void staticConfigure()
    {
        configure_orthogonal<or_check, CbChangeLifecycle<ClCheckLifecycle>>(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE
        );
    }

    void onEntry()
    {
        RCLCPP_INFO(getLogger(), "Estado StAsReady: Disparando comandos de configuração...");
        
    }

    void onExit()
    {
        RCLCPP_INFO(getLogger(), "Estado StAsReady: Saltando automaticamente para st_AsChecking!");
        
    }
};
} // namespace amp_sm