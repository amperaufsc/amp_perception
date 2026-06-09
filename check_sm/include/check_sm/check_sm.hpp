#pragma once

#include <smacc2/smacc.hpp>

namespace check_sm
{
    struct st_off;
    struct st_seup;
    struct st_checking;
    struct st_finished;
}

#include "orthogonals/or_check.hpp"

namespace check_sm
{
struct CheckSm : public smacc2::SmaccStateMachineBase<CheckSm, st_off>
{
    // ESTA LINHA É OBRIGATÓRIA: Ela herda os construtores do SMACC2 que o Boost exige
    using SmaccStateMachineBase::SmaccStateMachineBase;

    void onInitialize() override
    {
        RCLCPP_INFO(getLogger(), "[Check SM] Iniciando a Máquina de Estados...");

        // Instancia o ortogonal usando o nome correto em minúsculo
        this->createOrthogonal<or_check>();
    }
};
} // namespace check_sm

#include "states/st_off.hpp"
#include "states/st_setup.hpp"
#include "states/st_checking.hpp"
#include "states/st_finished.hpp"