#include "studica_control/encoder_component.h"

namespace studica_control {

Encoder::Encoder(const rclcpp::NodeOptions & options) : Node("encoder", options) {}

Encoder::Encoder(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b) 
    : Node(name), vmx_(vmx), port_a_(port_a), port_b_(port_b) {
    encoder_ = std::make_shared<studica_driver::Encoder>(port_a_, port_b_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "encoder_cmd",
        std::bind(&Encoder::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    count_publisher_ = this->create_publisher<std_msgs::msg::Int32>("encoder_count", 10);
    direction_publisher_ = this->create_publisher<std_msgs::msg::String>("encoder_direction", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Encoder::publish_data, this));
}

Encoder::~Encoder() {}

void Encoder::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, response);
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

void Encoder::publish_data() {
    std_msgs::msg::Int32 count_msg;
    count_msg.data = encoder_->GetCount();
    count_publisher_->publish(count_msg);

    std_msgs::msg::String direction_msg;
    direction_msg.data = encoder_->GetDirection();
    direction_publisher_->publish(direction_msg);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Encoder)
