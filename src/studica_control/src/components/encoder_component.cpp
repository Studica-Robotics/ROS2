#include "studica_control/encoder_component.h"

namespace studica_control
{
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

Encoder::Encoder(const rclcpp::NodeOptions & options) : Node("encoder", options) {
    Encoder("encoder", 0, 1, std::make_shared<VMXPi>(true, 50));
}

Encoder::Encoder(const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b, std::shared_ptr<VMXPi> vmx)
    : rclcpp::Node(name), vmx_(vmx), name_(name), port_a_(port_a), port_b_(port_b) {
    encoder_ = std::make_shared<studica_driver::Encoder>(port_a_, port_b_, vmx_);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Encoder)