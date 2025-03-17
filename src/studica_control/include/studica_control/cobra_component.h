#ifndef COBRA_COMPONENT_H
#define COBRA_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/Float32.hpp"

#include "cobra.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class Cobra : public rclcpp::Node {
public:
    explicit Cobra(const rclcpp::NodeOptions & options);
    Cobra(std::shared_ptr<VMXPi> vmx, const std::string &name, const float &vref);
    ~Cobra();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Cobra> cobra_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    std::string name_;
    float vref_;
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_float();
};

}  // namespace studica_control

#endif  // COBRA_COMPONENT_H
