
#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include "std_msgs/msg/float32.hpp"
#include "smbus/smbus.hpp"

#include <sensor_msgs/msg/joy.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

class EmsNode : public blackbox::BlackBoxNode
{
private:
    blackbox::Logger _stop_info;
    blackbox::Logger _stop_err;
    Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;

public:
    EmsNode(const std::string &name_space = "") 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::RELEASE, "ems_node", name_space)
    {
        _stop_info.init(this, blackbox::INFO, "stop");
        _stop_err.init(this, blackbox::ERR, "stop");

        TAGGER(_stop_info, "EMS Start!")

        this->_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy",rclcpp::QoS(10),std::bind(&EmsNode::joy_callback, this, std::placeholders::_1));
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        if(msg->buttons[10]){
            ESCtrl_safetyOn();
            TAGGER(_stop_err, "on");
        }
    }
};
