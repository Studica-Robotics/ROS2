#include "studica_control/titan_component.h"

namespace studica_control {

std::vector<std::shared_ptr<rclcpp::Node>> Titan::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> titan_nodes;
    control->declare_parameter<std::vector<std::string>>("titan.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("titan.sensors").as_string_array();
    for (const auto &sensor : sensor_ids) {
        std::string can_id_param = "titan." + sensor + ".can_id";
        std::string motor_freq_param = "titan." + sensor + ".motor_freq";
        std::string topic_param = "titan." + sensor + ".topic";

        control->declare_parameter<int>(can_id_param, -1);
        control->declare_parameter<int>(motor_freq_param, -1);
        control->declare_parameter<std::string>(topic_param, "");

        uint8_t can_id = control->get_parameter(can_id_param).as_int();
        uint16_t motor_freq = control->get_parameter(motor_freq_param).as_int();
        std::string topic = control->get_parameter(topic_param).as_string();

        auto titan = std::make_shared<Titan>(vmx, sensor, can_id, motor_freq, topic);
        titan_nodes.push_back(titan);
    }
    return titan_nodes;
}

Titan::Titan(const rclcpp::NodeOptions & options) : Node("titan_", options) {}

Titan::Titan(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID, const uint16_t &motor_freq, const std::string &topic)
    : Node(name), vmx_(vmx), canID_(canID), motor_freq_(motor_freq) {
    titan_ = std::make_shared<studica_driver::Titan>(canID_, motor_freq_, 1, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "titan_cmd",
        std::bind(&Titan::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Titan::publish_encoders, this));

    for (int i=0; i<4; i++) {
        titan_->ConfigureEncoder(i, 1);
        titan_->ResetEncoder(i);
    }
    titan_->Enable(true);
}

Titan::~Titan() {}

void Titan::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, request, response);
}

void Titan::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "enable") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan enabled";
    } else if (params == "disable") {
        titan_->Enable(false);
        response->success = true;
        response->message = "Titan disabled";
    } else if (params == "start") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan started";
    } else if (params == "setup_encoder") {
        titan_->SetupEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan encoder setup complete";
    } else if (params == "configure_encoder") {
        titan_->ConfigureEncoder(request->initparams.n_encoder, request->initparams.dist_per_tick);
        response->success = true;
        response->message = "Titan encoder configured";
    } else if (params == "stop") {
        titan_->SetSpeed(request->initparams.n_encoder, 0.0);
        response->success = true;
        response->message = "Titan stopped";
    } else if (params == "reset") {
        titan_->ResetEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan reset";
    } else if (params == "set_speed") {
        response->success = true;
        float speed = request->initparams.speed;
        RCLCPP_INFO(this->get_logger(), "Setting speed to %f", speed);
        titan_->SetSpeed(request->initparams.n_encoder, speed);
        response->message = "Encoder " + std::to_string(request->initparams.n_encoder) + " speed set to " + std::to_string(request->initparams.speed);
    } else if (params == "invert_motor") {
        titan_->InvertMotor(request->initparams.n_encoder);
        response->success = true;
        response->message = "Encoder " + std::to_string(request->initparams.n_encoder) + " direction has been inverted";
    } else if (params == "get_encoder_distance") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderDistance(request->initparams.n_encoder));
    } else if (params == "get_rpm") {
        response->success = true;
        response->message = std::to_string(titan_->GetRPM(request->initparams.n_encoder));
    } else if (params == "get_encoder_count") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderCount(request->initparams.n_encoder));
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void Titan::publish_encoders() {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(4);

    for (int i=0; i<4; i++) {
        msg.data[i] = static_cast<float>(titan_->GetEncoderCount(i));
    }

    publisher_->publish(msg);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Titan)
