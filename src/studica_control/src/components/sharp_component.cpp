#include "studica_control/sharp_component.h"

namespace studica_control {

Sharp::Sharp(const rclcpp::NodeOptions & options) : Node("sharp", options) {}

Sharp::Sharp(const std::string &name, VMXChannelIndex port, std::shared_ptr<VMXPi> vmx)
    : rclcpp::Node(name), vmx_(vmx), name_(name), port_(port) {
    sharp_ = std::make_shared<studica_driver::Sharp>(port_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "sharp_cmd",
        std::bind(&Sharp::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("ir_range", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Sharp::publish_range, this));
}

Sharp::~Sharp() {}

void Sharp::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, response);
}

void Sharp::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_distance") {
        response->success = true;
        response->message = std::to_string(sharp_->GetDistance());
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void Sharp::publish_range() {
    float distance = sharp_->GetDistance();

    sensor_msgs::msg::Range msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "sharp_sensor";
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = 0.5;
    msg.min_range = 0.1;
    msg.max_range = 0.8;
    msg.range = distance;

    publisher_->publish(msg);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Sharp)
