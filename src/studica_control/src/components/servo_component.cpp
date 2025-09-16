#include "studica_control/servo_component.h"

namespace studica_control {

std::vector<std::shared_ptr<rclcpp::Node>> Servo::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> servo_nodes;
    control->declare_parameter<std::vector<std::string>>("servo.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("servo.sensors").as_string_array();
    for (const auto &sensor : sensor_ids) {
        std::string port_param = "servo." + sensor + ".port";
        std::string type_param = "servo." + sensor + ".type";
        std::string topic_param = "servo." + sensor + ".topic";

        control->declare_parameter<int>(port_param, -1);
        control->declare_parameter<std::string>(type_param, "");
        control->declare_parameter<std::string>(topic_param, "unknown");

        int port = control->get_parameter(port_param).as_int();
        std::string type = control->get_parameter(type_param).as_string();
        std::string topic = control->get_parameter(topic_param).as_string();

        studica_driver::ServoType servo_type;
        int min_angle = 0, max_angle = 0;

        if (type == "standard") {
            servo_type = studica_driver::ServoType::Standard;
            min_angle = -150;
            max_angle = 150;
        } else if (type == "continuous") {
            servo_type = studica_driver::ServoType::Continuous;
            min_angle = -100;
            max_angle = 100;
        } else if (type == "linear") {
            servo_type = studica_driver::ServoType::Linear;
            min_angle = 0;
            max_angle = 100;
        } else {
            RCLCPP_ERROR(control->get_logger(), "Invalid servo type. Allowed values are 'standard', 'continuous', or 'linear'.");
        }

        RCLCPP_INFO(control->get_logger(), "%s -> port: %d, type: %s, topic: %s", sensor.c_str(), port, type.c_str(), topic.c_str());

        auto servo = std::make_shared<Servo>(vmx, sensor, port, servo_type, min_angle, max_angle, topic);
        servo_nodes.push_back(servo);
    }
    return servo_nodes;
}

Servo::Servo(const rclcpp::NodeOptions &options) : Node("servo", options) {}

Servo::Servo(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, studica_driver::ServoType type, int min, int max, const std::string &topic)
    : rclcpp::Node(name), vmx_(vmx), port_(port), type_(type) {
    servo_ = std::make_shared<studica_driver::Servo>(port, type_, min, max, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/set_servo_angle",
        std::bind(&Servo::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<std_msgs::msg::Float32>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Servo::publish_angle, this));
}

Servo::~Servo() {}

void Servo::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        int param_angle = std::stoi(request->params);
        servo_->SetAngle(param_angle);
        response->success = true;
        response->message = "Servo angle set to " + std::to_string(param_angle) + " degrees.";
        RCLCPP_INFO(this->get_logger(), "Set servo angle to %d degrees.", param_angle);
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "Failed to set servo angle: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to set servo angle: %s", e.what());
    }
}

void Servo::publish_angle() {
    std_msgs::msg::Float32 msg;
    msg.data = servo_->GetLastAngle();
    publisher_->publish(msg);
}

void Servo::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *err_str = GetVMXErrorString(vmxerr);
    printf("VMX Error %d: %s\n", vmxerr, err_str);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Servo)
