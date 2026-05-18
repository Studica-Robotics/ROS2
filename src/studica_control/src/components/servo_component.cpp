// servo_component.cpp
//
// ros2 component for a servo motor connected to a vmx-pi pwm output channel.
// supports three servo types:
//   standard   — position servo, -150 to 150 degrees
//   continuous — spinning servo, -100 to 100 (speed)
//   linear     — linear actuator, 0 to 100 (percent extension)
//
// topic (subscribes): <name>/cmd  (std_msgs/Float64) — set value directly
// topic (publishes):  <name>/state (std_msgs/Float64) — last commanded value, 20hz
// service: <name>/set_servo (studica_control/SetData) — initparams.speed sets value

#include "studica_control/servo_component.hpp"

namespace studica_control {


std::vector<std::shared_ptr<rclcpp::Node>> Servo::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> servo_nodes;

    control->declare_parameter<std::vector<std::string>>("servo.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("servo.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string port_param = "servo." + sensor + ".port";
        std::string type_param = "servo." + sensor + ".type";

        control->declare_parameter<int>(port_param, -1);
        control->declare_parameter<std::string>(type_param, "standard");

        int port         = control->get_parameter(port_param).as_int();
        std::string type = control->get_parameter(type_param).as_string();

        studica_driver::ServoType servo_type;
        int min_val = 0, max_val = 0;

        if (type == "standard") {
            servo_type = studica_driver::ServoType::Standard;
            min_val    = -150;
            max_val    = 150;
        } else if (type == "continuous") {
            servo_type = studica_driver::ServoType::Continuous;
            min_val    = -100;
            max_val    = 100;
        } else if (type == "linear") {
            servo_type = studica_driver::ServoType::Linear;
            min_val    = 0;
            max_val    = 100;
        } else {
            RCLCPP_ERROR(control->get_logger(),
                         "invalid servo type '%s' for '%s' — use 'standard', 'continuous', or 'linear'",
                         type.c_str(), sensor.c_str());
            continue;
        }

        RCLCPP_INFO(control->get_logger(), "%s -> port: %d, type: %s", sensor.c_str(), port, type.c_str());

        auto servo = std::make_shared<Servo>(vmx, sensor, port, servo_type, min_val, max_val);
        servo_nodes.push_back(servo);
    }

    return servo_nodes;
}


Servo::Servo(const rclcpp::NodeOptions &options) : Node("servo", options) {}


Servo::Servo(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port,
             studica_driver::ServoType type, int min, int max)
    : rclcpp::Node(name), vmx_(vmx), port_(port), type_(type) {

    servo_ = std::make_shared<studica_driver::Servo>(port_, type_, min, max, vmx_);

    // command subscriber — publish target value directly to this topic
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        name + "/cmd", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
            set_value(msg->data);
        });

    // service — set value via initparams.speed
    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/set_servo",
        std::bind(&Servo::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // state publisher
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(name + "/state", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Servo::publish_state, this));

    RCLCPP_INFO(this->get_logger(), "servo ready on port %d. cmd: /%s/cmd  state: /%s/state",
                port_, name.c_str(), name.c_str());
}

Servo::~Servo() {}


void Servo::set_value(double value) {
    servo_->SetAngle(static_cast<int>(value));
    RCLCPP_DEBUG(this->get_logger(), "servo set to %.1f", value);
}


void Servo::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        set_value(static_cast<double>(request->initparams.speed));
        response->success = true;
        response->message = "servo set to " + std::to_string(request->initparams.speed);
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "failed to set servo: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "failed to set servo: %s", e.what());
    }
}


void Servo::publish_state() {
    std_msgs::msg::Float64 msg;
    msg.data = static_cast<double>(servo_->GetLastAngle());
    publisher_->publish(msg);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Servo)
