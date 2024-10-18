#ifndef SHARP_SENSOR_NODE_HPP_
#define SHARP_SENSOR_NODE_HPP_

#include "studica_control/analog_input.h"
#include <rclcpp/rclcpp.hpp>
#include "VMXPi.h"
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>
#include "studica_control/device.h"
#include <cmath>

class SharpSensor : public Device {
public:
    // Updated constructor to accept AnalogInput
    SharpSensor(std::shared_ptr<VMXPi> vmx, const std::string &name, std::shared_ptr<AnalogInput> analog_input);

    void start_publishing();
    void stop_publishing();
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    double get_voltage();
private:
    void publish_analog_data();
    void Spin();

    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<AnalogInput> analog_input_;  // Analog input for Sharp sensor
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sharp_publisher_;
    bool is_publishing_;
    int count_;  // Counter for future use if needed
};

#endif // SHARP_SENSOR_NODE_HPP_
