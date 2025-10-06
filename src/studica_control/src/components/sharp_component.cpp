#include "studica_control/sharp_component.h"

namespace studica_control {

std::vector<std::shared_ptr<rclcpp::Node>> Sharp::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> sharp_nodes;
    control->declare_parameter<std::vector<std::string>>("sharp.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("sharp.sensors").as_string_array();
    for (const auto &sensor : sensor_ids) {
        std::string port_param = "sharp." + sensor + ".port";
        std::string topic_param = "sharp." + sensor + ".topic";

        control->declare_parameter<int>(port_param, -1);
        control->declare_parameter<std::string>(topic_param, "unknown");

        int port = control->get_parameter(port_param).as_int();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> port: %d, topic: %s", sensor.c_str(), port, topic.c_str());

        auto sharp = std::make_shared<Sharp>(vmx, sensor, port, topic);
        sharp_nodes.push_back(sharp);
    }
    return sharp_nodes;
}

Sharp::Sharp(const rclcpp::NodeOptions & options) : Node("sharp", options) {}

Sharp::Sharp(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, const std::string &topic)
    : rclcpp::Node(name), vmx_(vmx), port_(port) {
    sharp_ = std::make_shared<studica_driver::Sharp>(port_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "sharp_cmd",
        std::bind(&Sharp::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>(topic, 10);
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
    double min_range = 0.1, max_range = 0.8;
    float distance = sharp_->GetDistance() / 100.0;

    if (distance < min_range || distance > max_range) distance = INFINITY;

    sensor_msgs::msg::Range msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "sharp_sensor";
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = 0.26;
    msg.min_range = min_range;
    msg.max_range = max_range;
    msg.range = distance;

    publisher_->publish(msg);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Sharp)
