#pragma once

#include <smacc2/smacc.hpp>

namespace amp_sm
{
    struct st_off;
    struct st_AsReady;
    struct st_AsChecking;
    struct st_AsDriving;
    struct st_Emergency;
    struct st_AsFinished;
    struct EvCanTrigger : boost::statechart::event<EvCanTrigger> {};
}

#include "orthogonals/or_check.hpp"

namespace amp_sm
{
struct Amp_sm : public smacc2::SmaccStateMachineBase<Amp_sm, st_off>
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
} // namespace amp_sm

#include "states/st_AsOff.hpp"
#include "states/st_AsReady.hpp"
#include "states/st_AsDriving.hpp"
#include "states/st_AsChecking.hpp"
#include "states/st_AsFinished.hpp"
#include "states/st_AsEmergency.hpp"