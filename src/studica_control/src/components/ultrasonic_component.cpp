#include "studica_control/ultrasonic_component.h"

namespace studica_control
{

Ultrasonic::Ultrasonic(const rclcpp::NodeOptions &options) : Node("ultrasonic_", options) {
}

Ultrasonic::Ultrasonic(VMXChannelIndex ping, VMXChannelIndex echo) : Node("ultrasonic_") {
    auto& vmx_manager = studica_driver::VMXManager::getInstance();
    vmx_ = vmx_manager.getVMX();
    if (vmx_manager.isPinUsed(ping)) {
        printf("Port %d is already in use\n", ping);
        return;
    }
    if (vmx_manager.isPinUsed(echo)) {
        printf("Port %d is already in use\n", echo);
        return;
    }
    vmx_manager.setPinUsed(ping);
    vmx_manager.setPinUsed(echo);
    ultrasonic_ = std::make_shared<studica_driver::Ultrasonic>(ping, echo, vmx_);
}

void Ultrasonic::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Ultrasonic)