#pragma once

#include <smacc2/smacc.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <ros_timer_client/cl_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include "../client_behavior/cb_change_lifecycle.hpp"

namespace check_sm
{
struct st_off;

struct st_finished : smacc2::SmaccState<st_finished, CheckSm>
{
    using SmaccState::SmaccState;

    typedef smacc2::Transition<check_sm::EvResetTrigger, check_sm::st_off
    > reactions;

    static void staticConfigure()
    {
        // 1. Envia o sinal Activate para o serviço /.../change_state
        configure_orthogonal<or_check, CbChangeLifecycle>(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN
        );

        // 2. Inicia o timer
        configure_orthogonal<or_check, cl_ros_timer::CbTimer>();
    }

    void onExit()
    {
        RCLCPP_INFO(getLogger(), "Estado finished: Finalizando verificações...");
    }
};
} // namespace check_sm