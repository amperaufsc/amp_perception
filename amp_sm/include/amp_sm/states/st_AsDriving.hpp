#pragma once

#include <smacc2/smacc.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <amp_sm/clients/cl_topic_listener.hpp>

// #include "../client_behavior/..." // Substitua pelo cliente real que você quer configurar primeiro

namespace amp_sm
{
struct st_AsFinished;

struct st_AsDriving : smacc2::SmaccState<st_AsDriving, Amp_sm>
{
    using SmaccState::SmaccState;

    //typedef smacc2::Transition<amp_sm::EvDrivingListener, amp_sm::st_AsFinished // Substitua pelo evento real que você vai usar para sair desse estado
    //> reactions;

    static void staticConfigure()
    {
        // Configure o comportamento de cliente para enviar o comando de ativação
  }
    
    void onEntry()
    {
        RCLCPP_INFO(getLogger(), "Estado driving: Verificando o sistema...");
    }
};
} // namespace amp_sm