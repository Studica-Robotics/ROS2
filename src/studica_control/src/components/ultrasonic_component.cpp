/*
 * ultrasonic_component.cpp
 *
 * ros2 component for an hc-sr04-style ultrasonic range sensor.
 * measures distance by timing how long a sound pulse takes to bounce back.
 * effective range is roughly 2cm to 4m. readings outside this range are
 * reported as infinity. supports multiple sensors by adding entries to the
 * sensors list in params.yaml.
 *
 * topic:   <topic> (sensor_msgs/Range)
 *            distance in metres, published at 20hz.
 *            radiation_type: ultrasound
 *            valid range: 0.02m – 4.0m
 *
 * service: ultrasonic_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_distance             — returns distance in millimetres as a string
 *     get_distance_inches      — returns distance in inches as a string
 *     get_distance_millimeters — returns distance in millimetres as a string
 */

#include "studica_control/ultrasonic_component.h"

namespace studica_control {


// reads ultrasonic parameters from params.yaml and creates one node per entry
// in the sensors list
std::vector<std::shared_ptr<rclcpp::Node>> Ultrasonic::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> ultrasonic_nodes;

    control->declare_parameter<std::vector<std::string>>("ultrasonic.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("ultrasonic.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string ping_param  = "ultrasonic." + sensor + ".ping";
        std::string echo_param  = "ultrasonic." + sensor + ".echo";
        std::string topic_param = "ultrasonic." + sensor + ".topic";

        control->declare_parameter<int>(ping_param, -1);
        control->declare_parameter<int>(echo_param, -1);
        control->declare_parameter<std::string>(topic_param, "unknown");

        int ping          = control->get_parameter(ping_param).as_int();
        int echo          = control->get_parameter(echo_param).as_int();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> ping: %d, echo: %d, topic: %s",
                    sensor.c_str(), ping, echo, topic.c_str());

        auto ultrasonic = std::make_shared<Ultrasonic>(vmx, sensor, ping, echo, topic);
        ultrasonic_nodes.push_back(ultrasonic);
    }

    return ultrasonic_nodes;
}


// composable node constructor — used when loading as a plugin
Ultrasonic::Ultrasonic(const rclcpp::NodeOptions &options) : Node("ultrasonic", options) {}


// main constructor — connects to the sensor and sets up the publisher,
// service, and periodic timer
Ultrasonic::Ultrasonic(std::shared_ptr<VMXPi> vmx, const std::string &name,
                       VMXChannelIndex ping, VMXChannelIndex echo, const std::string &topic)
    : Node(name), vmx_(vmx), ping_(ping), echo_(echo) {

    ultrasonic_ = std::make_shared<studica_driver::Ultrasonic>(ping_, echo_, vmx_);

    // service for reading distance on demand
    service_ = this->create_service<studica_control::srv::SetData>(
        "ultrasonic_cmd",
        std::bind(&Ultrasonic::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes range data at 20hz
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Ultrasonic::publish_range, this));
}

Ultrasonic::~Ultrasonic() {}


void Ultrasonic::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                              std::shared_ptr<studica_control::srv::SetData::Response> response) {
    cmd(request->params, response);
}


// dispatches incoming service commands. each command triggers a ping first
// to get a fresh reading before returning the result.
void Ultrasonic::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_distance") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceMM());

    } else if (params == "get_distance_inches") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceIN());

    } else if (params == "get_distance_millimeters") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceMM());

    } else {
        response->success = false;
        response->message = "unknown command '" + params + "'";
    }
}


// triggers a ping, converts the result to metres, and publishes a standard
// range message. readings outside the sensor's valid range are set to infinity.
void Ultrasonic::publish_range() {
    constexpr double min_range     = 0.02;  // metres — hc-sr04 minimum
    constexpr double max_range     = 4.0;   // metres — hc-sr04 maximum
    constexpr double field_of_view = 0.26;  // radians — approximate cone width

    ultrasonic_->Ping();
    double distance_m = ultrasonic_->GetDistanceMM() / 1000.0;
    if (distance_m < min_range || distance_m > max_range) distance_m = INFINITY;

    sensor_msgs::msg::Range msg;
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = this->get_name();
    msg.radiation_type  = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view   = field_of_view;
    msg.min_range       = min_range;
    msg.max_range       = max_range;
    msg.range           = distance_m;

    publisher_->publish(msg);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Ultrasonic)
