#pragma once

#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/string.hpp> 

namespace check_sm
{
// 1. NOVO EVENTO: Exclusivo para o sinal de Reset
struct EvResetTrigger : boost::statechart::event<EvResetTrigger> {};

// 2. CLASSE DO CLIENTE: Escuta o tópico de reset
class ClResetListener : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>
{
public:
    // Construtor: Aponta para o novo tópico "/check_sm/reset"
    ClResetListener() 
        : smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>("/check_sm/reset")
    {
    }

    void onInitialize() override
    {
        smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>::onInitialize();
        
        // Vincula a função de callback deste cliente
        this->onMessageReceived(&ClResetListener::messageCallback, this);
    }

private:
    void messageCallback(const std_msgs::msg::String & msg)
    {
        // Filtra a mensagem exata "RESET"
        if (msg.data == "RESET")
        {
            RCLCPP_INFO(
                getLogger(),
                "[ClResetListener] Comando RESET recebido! Disparando evento de emergência/reset...");

            // Dispara o evento específico de reset
            this->postEvent<EvResetTrigger>();
        }
    }
};
} // namespace check_sm