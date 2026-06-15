#pragma once

#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/string.hpp> 

namespace amp_sm
{

    struct EvStartTopicListener : boost::statechart::event<EvStartTopicListener> {};
    struct EvCheckListener : boost::statechart::event<EvCheckListener> {};
    struct EvMissionSelectListener : boost::statechart::event<EvMissionSelectListener> {};
    struct EvStop : boost::statechart::event<EvCanTrigger> {};
    struct EvStartMissionListener : boost::statechart::event<EvStartMissionListener> {};
    struct EvSaltoAutomatico : boost::statechart::event<EvSaltoAutomatico> {}; // Evento para pular automaticamente entre Estados.


class ClStartListener : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>
{
public:
    ClStartListener() 
        : smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>("/as_amp/start_as")
    {
    }

    void onInitialize() override
    {
        smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>::onInitialize();
        
        // Atualizado para o novo nome da classe
        this->onMessageReceived(&ClStartListener::messageCallback, this);
    }
private:
    void messageCallback(const std_msgs::msg::String & msg)
    {
        if (msg.data == "START_AS")
        {
            RCLCPP_INFO(
                getLogger(),
                "[ClCanTopic] Comando START recebido! Disparando evento...");

            this->postEvent<EvStartTopicListener>();
        }
    }
};

class ClCheckListener : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>
{
public:
    ClCheckListener() 
        : smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>("/as_amp/check")
    {
    }

    void onInitialize() override
    {
        smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>::onInitialize();
        
        // Atualizado para o novo nome da classe
        this->onMessageReceived(&ClCheckListener::messageCallback, this);
    }
private:
    void messageCallback(const std_msgs::msg::String & msg)
    {
        if (msg.data == "CHECK")
        {
            RCLCPP_INFO(
                getLogger(),
                "[ClTopiclistener] Comando CHECK recebido! Disparando evento...");

            this->postEvent<EvCheckListener>();
        }
    }
};

class ClMissionSelectListener : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>
{
public:
    ClMissionSelectListener() 
        : smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>("/as_amp/mission_select")
    {
    }

    void onInitialize() override
    {
        smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>::onInitialize();
        
        // Atualizado para o novo nome da classe
        this->onMessageReceived(&ClMissionSelectListener::messageCallback, this);
    }
private:
    void messageCallback(const std_msgs::msg::String & msg)
    {
        if (msg.data == "MISSION_SELECT")
        {
            RCLCPP_INFO(
                getLogger(),
                "[ClCanTopic] Comando MISSION_SELECT recebido! Disparando evento...");

            this->postEvent<EvMissionSelectListener>();
        }
    }
};
class ClStartMissionListener : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>
{
public:
    ClStartMissionListener() 
        : smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>("/as_amp/start_mission")
    {
    }

    void onInitialize() override
    {
        smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::String>::onInitialize();
        
        // Atualizado para o novo nome da classe
        this->onMessageReceived(&ClStartMissionListener::messageCallback, this);
    }
private:
    void messageCallback(const std_msgs::msg::String & msg)
    {
        if (msg.data == "START_AS")
        {
            RCLCPP_INFO(
                getLogger(),
                "[ClCanTopic] Comando START recebido! Disparando evento...");

            this->postEvent<EvStartTopicListener>();
        }
    }
};
} // namespace amp_sm