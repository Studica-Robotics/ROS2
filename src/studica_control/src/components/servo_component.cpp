// servo_component.cpp
//
// ros2 component for a servo motor connected to a vmx-pi pwm output channel.
// supports three servo types, each with different angle ranges:
//   standard   — position servo, range: -150 to 150 degrees
//   continuous — spinning servo, range: -100 to 100 (speed, not angle)
//   linear     — linear actuator, range: 0 to 100 percent extension
//
// the type is set in params.yaml. supports multiple servos by adding
// entries to the sensors list.
//
// topic:   <topic> (std_msgs/Float32)
//            last commanded angle or speed value, published at 20hz
//
// service: <name>/set_servo_angle (studica_control/SetData)
//   pass the target value as a plain number string in the params field.
//   example: params = "90" moves a standard servo to 90 degrees.

#include "studica_control/servo_component.h"

namespace studica_control {


// reads servo parameters from params.yaml and creates one node per entry
// in the sensors list. angle limits are set automatically based on type.
std::vector<std::shared_ptr<rclcpp::Node>> Servo::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> servo_nodes;

    control->declare_parameter<std::vector<std::string>>("servo.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("servo.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string port_param  = "servo." + sensor + ".port";
        std::string type_param  = "servo." + sensor + ".type";
        std::string topic_param = "servo." + sensor + ".topic";

        control->declare_parameter<int>(port_param, -1);
        control->declare_parameter<std::string>(type_param, "");
        control->declare_parameter<std::string>(topic_param, "unknown");

        int port          = control->get_parameter(port_param).as_int();
        std::string type  = control->get_parameter(type_param).as_string();
        std::string topic = control->get_parameter(topic_param).as_string();

        studica_driver::ServoType servo_type;
        int min_angle = 0, max_angle = 0;

        if (type == "standard") {
            servo_type = studica_driver::ServoType::Standard;
            min_angle  = -150;
            max_angle  = 150;
        } else if (type == "continuous") {
            servo_type = studica_driver::ServoType::Continuous;
            min_angle  = -100;
            max_angle  = 100;
        } else if (type == "linear") {
            servo_type = studica_driver::ServoType::Linear;
            min_angle  = 0;
            max_angle  = 100;
        } else {
            RCLCPP_ERROR(control->get_logger(),
                         "invalid servo type '%s' — use 'standard', 'continuous', or 'linear'", type.c_str());
        }

        RCLCPP_INFO(control->get_logger(), "%s -> port: %d, type: %s, topic: %s",
                    sensor.c_str(), port, type.c_str(), topic.c_str());

        auto servo = std::make_shared<Servo>(vmx, sensor, port, servo_type, min_angle, max_angle, topic);
        servo_nodes.push_back(servo);
    }

    return servo_nodes;
}


// composable node constructor — used when loading as a plugin
Servo::Servo(const rclcpp::NodeOptions &options) : Node("servo", options) {}


// main constructor — connects to the servo and sets up the publisher,
// service, and periodic timer
Servo::Servo(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port,
             studica_driver::ServoType type, int min, int max, const std::string &topic)
    : rclcpp::Node(name), vmx_(vmx), port_(port), type_(type) {

    servo_ = std::make_shared<studica_driver::Servo>(port, type_, min, max, vmx_);

    // service name is prefixed with the servo name so multiple servos
    // each get their own unique service path
    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/set_servo_angle",
        std::bind(&Servo::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes the last commanded angle at 20hz
    publisher_ = this->create_publisher<std_msgs::msg::Float32>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Servo::publish_angle, this));
}

Servo::~Servo() {}


// receives a target angle as a string in request->params and sends it to the servo.
// example: params = "90" for 90 degrees on a standard servo
void Servo::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        int param_angle = std::stoi(request->params);
        servo_->SetAngle(param_angle);
        response->success = true;
        response->message = "servo angle set to " + std::to_string(param_angle) + " degrees";
        RCLCPP_INFO(this->get_logger(), "servo angle set to %d degrees", param_angle);
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "failed to set servo angle: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "failed to set servo angle: %s", e.what());
    }
}


// publishes the last angle that was commanded to the servo
void Servo::publish_angle() {
    std_msgs::msg::Float32 msg;
    msg.data = servo_->GetLastAngle();
    publisher_->publish(msg);
}


void Servo::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("vmx error %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Servo)
