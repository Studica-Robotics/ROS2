/*
 * sharp_component.cpp
 *
 * ros2 component for a sharp gp2y infrared range sensor.
 * measures distance using infrared light reflection. effective range is
 * roughly 10cm to 80cm. readings outside this range are reported as infinity.
 *
 * topic:   <topic> (sensor_msgs/Range)
 *            distance in metres, published at 20hz.
 *            radiation_type: infrared
 *            valid range: 0.1m – 0.8m
 *
 * service: sharp_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_distance — returns the current distance in centimetres as a string
 */

#include "studica_control/sharp_component.h"

namespace studica_control {


// reads sharp parameters from params.yaml and creates one node per entry
// in the sensors list
std::vector<std::shared_ptr<rclcpp::Node>> Sharp::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> sharp_nodes;

    control->declare_parameter<std::vector<std::string>>("sharp.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("sharp.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string port_param  = "sharp." + sensor + ".port";
        std::string topic_param = "sharp." + sensor + ".topic";

        control->declare_parameter<int>(port_param, -1);
        control->declare_parameter<std::string>(topic_param, "unknown");

        int port          = control->get_parameter(port_param).as_int();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> port: %d, topic: %s",
                    sensor.c_str(), port, topic.c_str());

        auto sharp = std::make_shared<Sharp>(vmx, sensor, port, topic);
        sharp_nodes.push_back(sharp);
    }

    return sharp_nodes;
}


// composable node constructor — used when loading as a plugin
Sharp::Sharp(const rclcpp::NodeOptions & options) : Node("sharp", options) {}


// main constructor — connects to the sensor and sets up the publisher,
// service, and periodic timer
Sharp::Sharp(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, const std::string &topic)
    : rclcpp::Node(name), vmx_(vmx), port_(port) {

    sharp_ = std::make_shared<studica_driver::Sharp>(port_, vmx_);

    // service for reading distance on demand
    service_ = this->create_service<studica_control::srv::SetData>(
        "sharp_cmd",
        std::bind(&Sharp::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes range data at 20hz
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Sharp::publish_range, this));
}

Sharp::~Sharp() {}


void Sharp::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    cmd(request->params, response);
}


// dispatches incoming service commands
void Sharp::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_distance") {
        response->success = true;
        response->message = std::to_string(sharp_->GetDistance());
    } else {
        response->success = false;
        response->message = "unknown command '" + params + "'";
    }
}


// reads distance from the sensor and publishes a standard range message.
// distance is converted from centimetres to metres. readings outside the
// sensor's valid range are set to infinity so downstream nodes can filter them.
void Sharp::publish_range() {
    constexpr double min_range    = 0.1;   // metres — gp2y0a21yk minimum
    constexpr double max_range    = 0.8;   // metres — gp2y0a21yk maximum
    constexpr double field_of_view = 0.26; // radians — approximate cone width

    float distance = sharp_->GetDistance() / 100.0; // cm to metres
    if (distance < min_range || distance > max_range) distance = INFINITY;

    sensor_msgs::msg::Range msg;
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = this->get_name();
    msg.radiation_type  = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view   = field_of_view;
    msg.min_range       = min_range;
    msg.max_range       = max_range;
    msg.range           = distance;

    publisher_->publish(msg);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Sharp)
