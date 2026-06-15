#pragma once

#include <smacc2/smacc_client.hpp> // <-- Mudamos a biblioteca base
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <vector>
#include <string>

namespace amp_sm
{
// 1. EVENTOS: O que a máquina de estados vai escutar?
struct EvNodeActivated : boost::statechart::event<EvNodeActivated> {};
struct EvNodeDeactivated : boost::statechart::event<EvNodeDeactivated> {};
struct EvNodeCrashed : boost::statechart::event<EvNodeCrashed> {};

// 2. O CLIENTE: Herdando da classe base genérica do SMACC2
class ClLifecycleMonitor : public smacc2::ISmaccClient
{
public:
    // O construtor agora recebe uma LISTA com os nomes dos nós
    ClLifecycleMonitor(const std::vector<std::string> & target_nodes) 
        : node_names_(target_nodes)
    {
    }

    void onInitialize() override
    {
        // Cria um "ouvinte" (Subscriber) para cada nó da lista
        for (const std::string & name : node_names_)
        {
            std::string topic_name = name + "/transition_event";

            // Usamos uma função Lambda [this, name] para repassar o nome do nó pro Callback
            auto sub = getNode()->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
                topic_name, 10,
                [this, name](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
                    this->messageCallback(msg, name);
                }
            );

            // Guardamos o subscriber na memória para ele não ser destruído
            subs_.push_back(sub);
        }
    }

private:
    std::vector<std::string> node_names_;
    std::vector<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> subs_;

    // O Callback agora recebe a mensagem E o nome exato do nó que a enviou
    void messageCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg, const std::string & node_name)
    {
        std::string novo_estado = msg->goal_state.label;

        // LOG PERSONALIZADO: Indica qual nó mudou de estado
        RCLCPP_INFO(getLogger(), "[Monitor -> %s] mudou para o estado: %s", 
                    node_name.c_str(), novo_estado.c_str());

        // Dispara o evento correspondente para a Máquina de Estados
        if (novo_estado == "active")
        {
            this->postEvent<EvNodeActivated>();
        }
        else if (novo_estado == "inactive")
        {
            this->postEvent<EvNodeDeactivated>();
        }
        else if (novo_estado == "unconfigured" || novo_estado == "errorprocessing")
        {
            // LOG DE ERRO: Aponta o dedo exatamente para o sensor que falhou
            RCLCPP_ERROR(getLogger(), "[Monitor -> %s] ALERTA CRÍTICO! O nó sofreu falha ou foi resetado!", node_name.c_str());
            this->postEvent<EvNodeCrashed>();
        }
    }
};
} // namespace amp_sm