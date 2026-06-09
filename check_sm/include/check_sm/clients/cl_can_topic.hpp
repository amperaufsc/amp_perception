#pragma once

#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/string.hpp> 

namespace check_sm
{
// O evento continua o mesmo
struct EvCanTrigger : boost::statechart::event<EvCanTrigger> {};

// CORREÇÃO: A classe agora se chama ClCanTopic
class ClCanTopic : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>
{
public:
    // Atualizado para o novo nome
    ClCanTopic() 
        : smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>("/can_msg")
    {
    }

    void onInitialize() override
    {
        smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>::onInitialize();
        
        // Atualizado para o novo nome da classe
        this->onMessageReceived(&ClCanTopic::messageCallback, this);
    }

private:
    void messageCallback(const std_msgs::msg::String & msg)
    {
        if (msg.data == "START")
        {
            RCLCPP_INFO(
                getLogger(),
                "[ClCanTopic] Comando START recebido! Disparando evento...");

            this->postEvent<EvCanTrigger>();
        }
    }
};
} // namespace check_sm