#include "studica_control/encoder_component.h"

namespace studica_control
{

Encoder::Encoder(const rclcpp::NodeOptions & options) : Node("encoder", options) {
    // Constructor
}

Encoder::~Encoder() {
    // Destructor
    auto& vmx_manager = studica_driver::VMXManager::getInstance();
    vmx_manager.setPinUnused(port_a_);
    vmx_manager.setPinUnused(port_b_);
}

void Encoder::Initialize(VMXChannelIndex port_a, VMXChannelIndex port_b) {
    port_a_ = port_a;
    port_b_ = port_b;
    auto& vmx_manager = studica_driver::VMXManager::getInstance();
    vmx_ = vmx_manager.getVMX();
    if (vmx_manager.isPinUsed(port_a)) {
        printf("Port %d is already in use\n", port_a);
        return;
    }
    if (vmx_manager.isPinUsed(port_b)) {
        printf("Port %d is already in use\n", port_b);
        return;
    }
    vmx_manager.setPinUsed(port_a);
    vmx_manager.setPinUsed(port_b);

    encoder_ = std::make_shared<studica_driver::Encoder>(port_a_, port_b_, vmx_);
}

void Encoder::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_count") {
        response->success = true;
        response->message = std::to_string(encoder_->GetCount());
    } else if (params == "get_direction") {
        response->success = true;
        response->message = encoder_->GetDirection();
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
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Encoder)