#include "studica_control/dc_encoder_component.h"

namespace studica_control {

DutyCycleEncoder::DutyCycleEncoder(const rclcpp::NodeOptions &options) : Node("duty_cycle_encoder", options) {}

DutyCycleEncoder::DutyCycleEncoder(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port) 
    : rclcpp::Node(name), vmx_(vmx), name_(name), port_(port) {
    duty_cycle_encoder_ = std::make_shared<studica_driver::DutyCycleEncoder>(port_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "duty_cycle_encoder_cmd",
        std::bind(&DutyCycleEncoder::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("encoder_count", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DutyCycleEncoder::publish_data, this));
}

DutyCycleEncoder::~DutyCycleEncoder() {}

void DutyCycleEncoder::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
    std::shared_ptr<studica_control::srv::SetData::Response> response) {}

void DutyCycleEncoder::cmd() {}

void DutyCycleEncoder::publish_data() {
    std_msgs::msg::Int32 msg;
    msg.data = duty_cycle_encoder_->GetTotalRotation();
    publisher_->publish(msg);
}

void DutyCycleEncoder::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("VMX Error %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DutyCycleEncoder)
