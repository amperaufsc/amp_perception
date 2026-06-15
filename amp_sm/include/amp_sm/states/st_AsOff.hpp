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
struct st_AsReady;

struct st_off : smacc2::SmaccState<st_off, Amp_sm>
{
    using SmaccState::SmaccState;

    typedef boost::mpl::list<
        smacc2::Transition<amp_sm::EvSaltoAutomatico, amp_sm::st_AsReady>  
    > reactions;

    static void staticConfigure()
    {
        //
    }

    void onEntry()
    {
        RCLCPP_INFO(getLogger(), "Estado off: Disparando comandos de configuração...");
        
        // ... (Se tiver mais algum código para rodar aqui, ele roda primeiro) ...

        this->postEvent<EvSaltoAutomatico>();
    }

    void onExit()
    {
        RCLCPP_INFO(getLogger(), "Estado off: Saltando automaticamente para st_AsReady!");
    }
};
} // namespace amp_sm