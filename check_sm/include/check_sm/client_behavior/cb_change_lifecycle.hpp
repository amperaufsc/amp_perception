#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_service_client.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

namespace check_sm
{
// A classe herda de SmaccClientBehavior, indicando que é uma ação executável
class CbChangeLifecycle : public smacc2::SmaccClientBehavior
{
private:
    // Ponteiro para o cliente de serviço que vai se comunicar com o nó externo
    smacc2::client_bases::SmaccServiceClient<lifecycle_msgs::srv::ChangeState>* lifecycle_client_;
    
    // Variável para guardar qual transição queremos fazer (Ativar, Configurar, Desativar, etc.)
    uint8_t transition_id_;

public:
    // Construtor: Recebe o ID da transição diretamente do configure_orthogonal no Estado
    CbChangeLifecycle(uint8_t transition_id) 
    {
        transition_id_ = transition_id;
    }

    // onEntry é executado automaticamente assim que o Estado que possui este Behavior é ativado
    void onEntry() override
    {
        // 1. OBRIGATÓRIO: Pede ao SMACC2 para conectar o ponteiro local ao cliente real lá no Ortogonal
        this->requiresClient(lifecycle_client_);

        if (lifecycle_client_ != nullptr)
        {
            RCLCPP_INFO(getLogger(), "[CbChangeLifecycle] Preparando comando de transição (ID: %d)...", transition_id_);

            // 2. Monta a requisição no formato padrão da mensagem de serviço do ROS 2
            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id = transition_id_;

            // 3. Executa a chamada do serviço
            lifecycle_client_->call(request);
        }
        else
        {
            // Proteção crucial caso você esqueça de instanciar o cliente no or_check.hpp
            RCLCPP_ERROR(getLogger(), "[CbChangeLifecycle] FALHA FATAL: Cliente de Lifecycle não encontrado no Ortogonal!");
        }
    }
};
} // namespace check_sm