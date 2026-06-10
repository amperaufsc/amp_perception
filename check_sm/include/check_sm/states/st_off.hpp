#pragma once

#include <smacc2/smacc.hpp>
#include <keyboard_client/cl_keyboard.hpp>
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

// CORREÇÃO: Você precisa incluir o header que define o EvCanTrigger aqui!
// #include "../clients/cl_can_listener.hpp" 

namespace check_sm
{
struct st_setup;

struct st_off : smacc2::SmaccState<st_off, CheckSm>
{
    using SmaccState::SmaccState;

    // CORREÇÃO: As reações ficam AQUI, no corpo da struct!
    typedef boost::mpl::list<
        smacc2::Transition<check_sm::EvCanTrigger, check_sm::st_setup>  
    > reactions;

    static void staticConfigure()
    {
        //..
    }

    void onEntry()
    {
        RCLCPP_INFO(getLogger(), "Estado off: aguardando pulso no tópico /can_msg...");
    }
};
} // namespace check_sm