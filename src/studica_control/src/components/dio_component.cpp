#include "studica_control/dio_component.h"

namespace studica_control
{

DIO::DIO(const rclcpp::NodeOptions & options) : Node("dio", options) {
    // Constructor
}

DIO::~DIO() {
    // Destructor
    auto& vmx_manager = studica_driver::VMXManager::getInstance();
    vmx_manager.setPinUnused(pin_mode_);
}

void DIO::Initialize(VMXChannelIndex pin, PinMode pin_mode) {
    pin_ = pin;
    pin_mode_ = pin_mode;
    auto& vmx_manager = studica_driver::VMXManager::getInstance();
    vmx_ = vmx_manager.getVMX();
    if (vmx_manager.isPinUsed(pin)) {
        printf("Pin %d is already in use\n", pin);
        return;
    }
    vmx_manager.setPinUsed(pin);

    dio_ = std::make_shared<studica_driver::DIO>(pin_, pin_mode_, vmx_);
}

void DIO::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_count") {
        response->success = true;
        response->message = std::to_string(dio_->GetCount());
    } else if (params == "get_direction") {
        response->success = true;
        response->message = dio_->GetDirection();
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DIO)