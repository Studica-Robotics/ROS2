#ifndef TITAN_COMPONENT_H
#define TITAN_COMPONENT_H

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "titan.h"
#include "VMXManager.h"
#include "VMXPi.h"  
#include "studica_control/srv/set_data.hpp" 
#include "studica_control/msg/initialize_params.hpp"

namespace studica_control
{

class Titan : public rclcpp::Node {
public:
    Titan(const std::string &name, const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, const float &speed);
    explicit Titan(const rclcpp::NodeOptions & options);
    ~Titan();
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Titan> titan_;
    std::string name_;
    uint8_t canID_;
    uint16_t motorFreq_;
    float distPerTick_;
    float speed_;
};
}  // namespace studica_control

#endif  // TITAN_COMPONENT_H
