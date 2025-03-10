#include "studica_control/sharp_component.h"

namespace studica_control {
void Sharp::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_distance") {
        response->success = true;
        response->message = std::to_string(sharp_->GetDistance());
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

Sharp::Sharp(const rclcpp::NodeOptions & options) : Node("sharp", options) {
    Sharp("sharp", 0, std::make_shared<VMXPi>(true, 50));
}


Sharp::Sharp(const std::string &name, VMXChannelIndex port, std::shared_ptr<VMXPi> vmx)
    : rclcpp::Node(name), vmx_(vmx), name_(name), port_(port) {
    sharp_ = std::make_shared<studica_driver::Sharp>(port_, vmx_);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Sharp)
