#include "studica_control/servo_component.h"

namespace studica_control {

Servo::Servo(const rclcpp::NodeOptions &options) : Node("servo_", options) {
    service_ = this->create_service<studica_control::srv::SetData>(
        "set_servo_angle",
        std::bind(&Servo::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Servo component is ready.");
}

Servo::Servo(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, studica_driver::ServoType type, int min, int max)
    : rclcpp::Node("servo_component"), vmx_(vmx), name_(name), port_(port), type_(type) {
    servo_ = std::make_shared<studica_driver::Servo>(port, type_, min, max, vmx_);

    service_ = this->create_service<studica_control::srv::SetData>(
        "set_servo_angle",
        std::bind(&Servo::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Servo component is ready.");
}

Servo::~Servo() {}

void Servo::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (request->name != name_) return;
    try {
        int param_angle = std::stoi(request->params);
        servo_->SetAngle(param_angle);
        response->success = true;
        response->message = "Servo angle set to " + std::to_string(param_angle) + " degrees.";
        RCLCPP_INFO(this->get_logger(), "Set servo angle to %d degrees.", param_angle);
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "Failed to set servo angle: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to set servo angle: %s", e.what());
    }
}

void Servo::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *err_str = GetVMXErrorString(vmxerr);
    printf("VMX Error %d: %s\n", vmxerr, err_str);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Servo)
