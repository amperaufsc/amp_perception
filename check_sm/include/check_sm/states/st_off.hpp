#pragma once

#include <smacc2/smacc.hpp>
#include <keyboard_client/cl_keyboard.hpp>
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

// CORREÇÃO: Você precisa incluir o header que define o EvCanTrigger aqui!
// #include "../clients/cl_can_listener.hpp" 

namespace check_sm
{
struct st_checking;

struct st_off : smacc2::SmaccState<st_off, CheckSm>
{
    using SmaccState::SmaccState;

    // CORREÇÃO: As reações ficam AQUI, no corpo da struct!
    typedef boost::mpl::list<
        smacc2::Transition<check_sm::EvCanTrigger, check_sm::st_checking>,
        
        // CORREÇÃO: O primeiro parâmetro deve ser o Cliente (ClKeyboard)
        smacc2::Transition<
            cl_keyboard::EvKeyPressE<cl_keyboard::ClKeyboard, or_check>, 
            check_sm::st_checking
        >
    > reactions;

    static void staticConfigure()
    {
        configure_orthogonal<or_check, cl_keyboard::CbDefaultKeyboardBehavior>();
    }

    void onEntry()
    {
        RCLCPP_INFO(getLogger(), "[Check SM] ST_OFF: Aguardando pulso no tópico /can_msg ou tecla 'e'...");
    }
};
} // namespace check_sm