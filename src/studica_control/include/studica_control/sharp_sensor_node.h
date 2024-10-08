#ifndef SHARP_SENSOR_NODE_HPP_
#define SHARP_SENSOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "VMXPi.h"
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>
#include "studica_control/device.h"

class SharpSensor : public Device {
public:
    SharpSensor(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex ping);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) override;
    
    void start_publishing();  // Method to start publishing
    void stop_publishing();   // Method to stop publishing

private:
    void publish_analog_data();
    void Spin();
    void publish_message();

    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sharp_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    int count_;
    bool is_publishing_;

    VMXResourceHandle accumulator_res_handle;
    VMXChannelIndex ping_channel_index;
};

 #endif // SHARP_NODE_HPP_