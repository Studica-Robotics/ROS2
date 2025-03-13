#ifndef TITAN_COMPONENT_H
#define TITAN_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include "VMXPi.h" 

#include "studica_control/srv/set_data.hpp"
#include "titan.h"

namespace studica_control {

class Titan : public rclcpp::Node {
public:
    Titan(
        std::shared_ptr<VMXPi> vmx,
        const std::string &name,
        const uint8_t &canID,
        const uint16_t &motor_freq,
        const float &ticks_per_rotation,
        const float &wheel_radius);
    explicit Titan(const rclcpp::NodeOptions & options);
    ~Titan();
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Titan> titan_;

    std::string name_;
    uint8_t canID_;
    uint16_t motor_freq_;
    float dist_per_tick_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
};

}  // namespace studica_control

#endif  // TITAN_COMPONENT_H
