#pragma once

#include <smacc2/smacc.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <ros_timer_client/cl_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include "../client_behavior/cb_change_lifecycle.hpp"

namespace check_sm
{
struct st_off;

struct st_checking : smacc2::SmaccState<st_checking, CheckSm>
{
    using SmaccState::SmaccState;

    // CORREÇÃO: As reações ficam AQUI, fora da função!
    typedef smacc2::Transition<
        cl_ros_timer::EvTimer<cl_ros_timer::ClRosTimer, or_check>,
        st_off
    > reactions;

    static void staticConfigure()
    {
        // 1. Ativa o nó de percepção via Lifecycle
        configure_orthogonal<or_check, CbChangeLifecycle>(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE
        );

        // 2. Inicia o timer
        configure_orthogonal<or_check, cl_ros_timer::CbTimer>();
    }

    void onExit()
    {
        // ... sua lógica de segurança ...
    }
};
} // namespace check_sm