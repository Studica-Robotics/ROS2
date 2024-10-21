#ifndef COBRA_COMPONENT_H
#define COBRA_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cobra.h"
#include "studica_control/srv/set_data.hpp" 
#include "VMXPi.h"  

namespace studica_control
{

class Cobra : public rclcpp::Node {
public:
    Cobra(const std::string &name, const float& vref, const int& muxch, std::shared_ptr<VMXPi> vmx);
    explicit Cobra(const rclcpp::NodeOptions & options);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Cobra> cobra_;
    std::string name_;
    float vref_;
    int muxch_;
};
}  // namespace studica_control

#endif  // COBRA_COMPONENT_H
