#include "studica_control/servo_component.h"

namespace studica_control
{

Servo::Servo(const rclcpp::NodeOptions &options) : Node("servo_", options) {
    // servo_ = std::make_shared<studica_driver::Servo>(0, type_, 0, 180, std::make_shared<VMXPi>(true, 50));
}

Servo::Servo(VMXChannelIndex port, studica_driver::ServoType type, int min, int max) : Node("servo_") {
    auto& vmx_manager = studica_driver::VMXManager::getInstance();
    vmx_ = vmx_manager.getVMX();
    if (vmx_manager.isPinUsed(port)) {
        printf("Port %d is already in use\n", port);
        return;
    }
    vmx_manager.setPinUsed(port);
    servo_ = std::make_shared<studica_driver::Servo>(port, type, min, max, vmx_);
}

void Servo::initialize(VMXChannelIndex port, studica_driver::ServoType type, int min, int max) {
    auto& vmx_manager = studica_driver::VMXManager::getInstance();
    vmx_ = vmx_manager.getVMX();
    if (vmx_manager.isPinUsed(port)) {
        printf("Port %d is already in use\n", port);
        return;
    }
    vmx_manager.setPinUsed(port);
    servo_ = std::make_shared<studica_driver::Servo>(port, type, min, max, vmx_);
}
    
// Servo::Servo(std::shared_ptr<VMXPi> vmx, VMXChannelIndex port, studica_driver::ServoType type, int min, int max) 
//     : rclcpp::Node("servo_"), vmx_(vmx), port_(port), type_(type) {
//     servo_ = std::make_shared<studica_driver::Servo>(port, type_, min, max, vmx_);
// }

Servo::~Servo() {}

void Servo::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    int param_angle = (int)strtol(params.c_str(), NULL, 10);
    servo_->SetAngle(param_angle);

    // if (params == "set_angle")
    // {
    //     SetAngle(angle);
    //     response->success = true;
    //     response->message = "Servo set to 90 degrees";
    // }
    // else if (params == "set_speed")
    // {
    //     SetSpeed(50);
    //     response->success = true;
    //     response->message = "Servo set to 50 speed";
    // }
    // else
    // {
    //     response->success = false;
    //     response->message = "Invalid command";
    // }
}

void Servo::DisplayVMXError(VMXErrorCode vmxerr) {
    const char* err_str = GetVMXErrorString(vmxerr);
    printf("VMX Error %d: %s\n", vmxerr, err_str);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Servo)