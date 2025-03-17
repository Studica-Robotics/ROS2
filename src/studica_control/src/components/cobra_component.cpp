#include "studica_control/cobra_component.h"

namespace studica_control {

Cobra::Cobra(const rclcpp::NodeOptions & options) : Node("cobra", options) {}

Cobra::Cobra(std::shared_ptr<VMXPi> vmx, const std::string &name, const float& vref)
    : Node(name), vmx_(vmx), name_(name), vref_(vref) {
    cobra_ = std::make_shared<studica_driver::Cobra>(vmx_, vref_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "cobra_cmd",
        std::bind(&Cobra::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("cobra", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Cobra::publish_float, this));
}

Cobra::~Cobra() {}

void Cobra::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, response);
}

void Cobra::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_raw") {
        response->success = true;
        response->message = std::to_string(cobra_->GetRawValue(1));
    } else if (params == "get_voltage") {
        response->success = true;
        response->message = std::to_string(cobra_->GetVoltage(1));
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void Cobra::publish_float() {
    float voltage = cobra_->GetVoltage(1);

    std_msgs::msg::Float32 msg;
    msg.data = voltage;

    publisher_->publish(msg);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Cobra)
