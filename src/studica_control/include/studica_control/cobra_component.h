#ifndef COBRA_COMPONENT_H
#define COBRA_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "cobra.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class Cobra : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit Cobra(const rclcpp::NodeOptions & options);
    Cobra(std::shared_ptr<VMXPi> vmx, const std::string &name, const float &vref, const std::string &topic);
    ~Cobra();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Cobra> cobra_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float vref_;
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_float();
};

}  // namespace studica_control

#endif  // COBRA_COMPONENT_H
