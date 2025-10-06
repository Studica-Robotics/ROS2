#include "studica_control/ultrasonic_component.h"

namespace studica_control {

std::vector<std::shared_ptr<rclcpp::Node>> Ultrasonic::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> ultrasonic_nodes;
    control->declare_parameter<std::vector<std::string>>("ultrasonic.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("ultrasonic.sensors").as_string_array();
    for (const auto &sensor : sensor_ids) {        
        std::string ping_param = "ultrasonic." + sensor + ".ping";
        std::string echo_param = "ultrasonic." + sensor + ".echo";
        std::string topic_param = "ultrasonic." + sensor + ".topic";

        control->declare_parameter<int>(ping_param, -1);
        control->declare_parameter<int>(echo_param, -1);
        control->declare_parameter<std::string>(topic_param, "unknown");

        int ping = control->get_parameter(ping_param).as_int();
        int echo = control->get_parameter(echo_param).as_int();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> ping: %d, echo: %d, topic: %s", sensor.c_str(), ping, echo, topic.c_str());

        auto ultrasonic = std::make_shared<Ultrasonic>(vmx, sensor, ping, echo, topic);
        ultrasonic_nodes.push_back(ultrasonic);
    }
    return ultrasonic_nodes;
}

Ultrasonic::Ultrasonic(const rclcpp::NodeOptions &options) : Node("ultrasonic", options) {}

Ultrasonic::Ultrasonic(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex ping, VMXChannelIndex echo, const std::string &topic) 
    : Node(name), vmx_(vmx), ping_(ping), echo_(echo) {
    ultrasonic_ = std::make_shared<studica_driver::Ultrasonic>(ping_, echo_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "ultrasonic_cmd",
        std::bind(&Ultrasonic::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Ultrasonic::publish_range, this));
}

Ultrasonic::~Ultrasonic() {}

void Ultrasonic::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, response);
}

void Ultrasonic::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_distance_inches") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceIN());
        ultrasonic_->Ping();
    } else if (params == "get_distance_millimeters") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceMM());
        ultrasonic_->Ping();
    } else if (params == "get_distance") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceMM());
        ultrasonic_->Ping();
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void Ultrasonic::publish_range() {
    double min_range = 0.02, max_range = 4.0;

    ultrasonic_->Ping();
    double distance_m = ultrasonic_->GetDistanceMM() / 1000.0;

    if (distance_m < min_range || distance_m > max_range) distance_m = INFINITY;
    
    sensor_msgs::msg::Range msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "ultrasonic_sensor";
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.26;
    msg.min_range = min_range;
    msg.max_range = max_range;
    msg.range = distance_m;

    publisher_->publish(msg);
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
