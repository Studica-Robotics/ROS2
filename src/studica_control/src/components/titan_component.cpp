/*
 * titan_component.cpp
 *
 * ros2 component for the titan motor controller.
 * the titan is a can bus motor controller that supports up to 4 motors.
 *
 * encoder mode is configured per motor in params.yaml.
 *
 *   quadrature (default):
 *     publishes /name/m_N/encoder (distance) and /name/m_N/rpm at 20hz
 *
 *   absolute (cypher encoder):
 *     publishes /name/m_N/angle (degrees) at 20hz
 *
 * command topics (always active):
 *     subscribe /name/m_N/cmd (Float64, duty cycle -1.0 to 1.0)
 *
 * service: /name/titan_cmd (studica_control/SetData)
 *   see titan_component.h for the full command reference.
 */

#include "studica_control/titan_component.hpp"
#include <thread>

namespace studica_control {


std::vector<std::shared_ptr<rclcpp::Node>> Titan::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> titan_nodes;

    control->declare_parameter<std::vector<std::string>>("titan.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("titan.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string can_id_param     = "titan." + sensor + ".can_id";
        std::string motor_freq_param = "titan." + sensor + ".motor_freq";

        control->declare_parameter<int>(can_id_param, -1);
        control->declare_parameter<int>(motor_freq_param, -1);

        uint8_t  can_id     = static_cast<uint8_t>(control->get_parameter(can_id_param).as_int());
        uint16_t motor_freq = static_cast<uint16_t>(control->get_parameter(motor_freq_param).as_int());

        std::array<MotorConfig, 4> motor_configs;
        for (int m = 0; m < 4; m++) {
            std::string prefix = "titan." + sensor + ".m_" + std::to_string(m);
            control->declare_parameter<std::string>(prefix + ".encoder_mode", "quadrature");
            control->declare_parameter<double>     (prefix + ".dist_per_tick", 1.0);
            control->declare_parameter<bool>       (prefix + ".invert_motor",   false);
            control->declare_parameter<bool>       (prefix + ".invert_encoder", false);
            control->declare_parameter<bool>       (prefix + ".invert_rpm",     false);

            std::string mode_str = control->get_parameter(prefix + ".encoder_mode").as_string();
            motor_configs[m].encoder_mode  = (mode_str == "absolute") ? EncoderMode::Absolute : EncoderMode::Quadrature;
            motor_configs[m].dist_per_tick = control->get_parameter(prefix + ".dist_per_tick").as_double();
            motor_configs[m].invert_motor   = control->get_parameter(prefix + ".invert_motor").as_bool();
            motor_configs[m].invert_encoder = control->get_parameter(prefix + ".invert_encoder").as_bool();
            motor_configs[m].invert_rpm     = control->get_parameter(prefix + ".invert_rpm").as_bool();
        }

        RCLCPP_INFO(control->get_logger(), "%s -> can_id: %d, motor_freq: %d hz",
                    sensor.c_str(), can_id, motor_freq);

        auto titan = std::make_shared<Titan>(vmx, sensor, can_id, motor_freq, motor_configs);
        titan_nodes.push_back(titan);
    }

    return titan_nodes;
}


Titan::Titan(const rclcpp::NodeOptions &options) : Node("titan_", options) {}


Titan::Titan(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID,
             const uint16_t &motor_freq, const std::array<MotorConfig, 4> &motor_configs)
    : Node(name), vmx_(vmx), canID_(canID), motor_freq_(motor_freq), motor_configs_(motor_configs) {

    titan_ = std::make_shared<studica_driver::Titan>(canID_, motor_freq_, 1, vmx_);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/titan_cmd",
        std::bind(&Titan::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    for (int i = 0; i < 4; i++) {
        std::string prefix = name + "/m_" + std::to_string(i);

        // command subscriber — always created
        cmd_subs_[i] = this->create_subscription<std_msgs::msg::Float64>(
            prefix + "/cmd", 10,
            [this, i](std_msgs::msg::Float64::SharedPtr msg) {
                if (!enabled_) return;
                float speed = static_cast<float>(msg->data);
                speeds_[i] = speed;
                titan_->SetSpeed(i, speed);
            });

        // feedback publishers — depend on encoder mode
        if (motor_configs_[i].encoder_mode == EncoderMode::Quadrature) {
            encoder_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(prefix + "/encoder", 10);
            rpm_pubs_[i]     = this->create_publisher<std_msgs::msg::Float64>(prefix + "/rpm", 10);
            RCLCPP_INFO(this->get_logger(), "  m_%d: quadrature -> /%s/encoder, /%s/rpm",
                        i, prefix.c_str(), prefix.c_str());
        } else {
            angle_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(prefix + "/angle", 10);
            RCLCPP_INFO(this->get_logger(), "  m_%d: absolute   -> /%s/angle", i, prefix.c_str());
        }
    }

    encoder_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Titan::publish_encoders, this));

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Titan::resend_speeds, this));

    for (int i = 0; i < 4; i++) {
        titan_->ConfigureEncoder(i, motor_configs_[i].dist_per_tick);
        titan_->ResetEncoder(i);
        if (motor_configs_[i].invert_motor)   titan_->InvertMotorDirection(i);
        if (motor_configs_[i].invert_encoder) titan_->InvertEncoderDirection(i);
        if (motor_configs_[i].invert_rpm)     titan_->InvertMotorRPM(i);
    }

    titan_->SetPIDType(0);

    // The Titan requires ~1s after initial CONFIG_MOTOR frames before it will
    // accept the enable command. See drivers/examples/titan_example/titan_example.cpp.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    titan_->Enable(true);
    enabled_ = true;

    RCLCPP_INFO(this->get_logger(), "titan ready. can_id: %d, freq: %d hz", canID_, motor_freq_);
}

Titan::~Titan() {}


void Titan::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    cmd(request->params, request, response);
}


void Titan::cmd(std::string params,
                std::shared_ptr<studica_control::srv::SetData::Request> request,
                std::shared_ptr<studica_control::srv::SetData::Response> response) {

    uint8_t motor = static_cast<uint8_t>(request->initparams.n_encoder);

    // --- basic control ---

    if (params == "enable") {
        titan_->Enable(true);
        enabled_ = true;
        response->success = true;
        response->message = "titan enabled";

    } else if (params == "disable") {
        for (int i = 0; i < 4; i++) speeds_[i] = 0.0f;
        enabled_ = false;
        titan_->Enable(false);
        response->success = true;
        response->message = "titan disabled";

    } else if (params == "set_speed") {
        float speed = request->initparams.speed;
        speeds_[motor] = speed;
        titan_->SetSpeed(motor, speed);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " speed set to " + std::to_string(speed);

    } else if (params == "set_speed_all") {
        float speed = request->initparams.speed;
        for (int i = 0; i < 4; i++) speeds_[i] = speed;
        titan_->SetSpeedAll(static_cast<double>(speed));
        response->success = true;
        response->message = "all motors set to " + std::to_string(speed);

    } else if (params == "stop") {
        speeds_[motor] = 0.0f;
        titan_->SetSpeed(motor, 0.0);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " stopped";

    } else if (params == "disable_motor") {
        titan_->DisableMotor(motor);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " disabled";

    } else if (params == "set_motor_stop_mode") {
        titan_->SetMotorStopMode(static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = "stop mode set to " + std::to_string(request->initparams.int_value);

    // --- closed loop velocity / position control (titan2 firmware) ---

    } else if (params == "set_target_velocity") {
        int16_t rpm = static_cast<int16_t>(request->initparams.speed);
        titan_->SetTargetVelocity(motor, rpm);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " target velocity set to " + std::to_string(rpm) + " rpm";

    } else if (params == "set_target_distance") {
        titan_->SetTargetDistance(motor, static_cast<int32_t>(request->initparams.int_value));
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " target distance set to "
                            + std::to_string(request->initparams.int_value) + " counts";

    } else if (params == "set_target_angle") {
        titan_->SetTargetAngle(motor, static_cast<double>(request->initparams.speed));
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " target angle set to "
                            + std::to_string(request->initparams.speed) + " degrees";

    } else if (params == "set_position_hold") {
        titan_->SetPositionHold(motor, request->initparams.hold);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " position hold "
                            + std::string(request->initparams.hold ? "enabled" : "disabled");

    // --- pid tuning (titan2 firmware) ---

    } else if (params == "set_pid_type") {
        titan_->SetPIDType(static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = "pid type set to " + std::to_string(request->initparams.int_value);

    } else if (params == "set_sensitivity") {
        titan_->SetSensitivity(motor, static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " sensitivity set to "
                            + std::to_string(request->initparams.int_value);

    } else if (params == "autotune") {
        titan_->AutotuneAll();
        response->success = true;
        response->message = "autotune started on all motors";

    // --- encoder configuration ---

    } else if (params == "setup_encoder") {
        titan_->SetupEncoder(motor);
        response->success = true;
        response->message = "encoder " + std::to_string(motor) + " setup complete";

    } else if (params == "configure_encoder") {
        titan_->ConfigureEncoder(motor, static_cast<double>(request->initparams.dist_per_tick));
        response->success = true;
        response->message = "encoder " + std::to_string(motor) + " configured";

    } else if (params == "set_encoder_resolution") {
        titan_->SetEncoderResolution(motor, static_cast<uint16_t>(request->initparams.int_value));
        response->success = true;
        response->message = "encoder " + std::to_string(motor) + " resolution set to "
                            + std::to_string(request->initparams.int_value) + " cpr";

    } else if (params == "reset_encoder") {
        titan_->ResetEncoder(motor);
        response->success = true;
        response->message = "encoder " + std::to_string(motor) + " reset";

    // --- current limiting ---

    } else if (params == "set_current_limit") {
        titan_->SetCurrentLimit(motor, request->initparams.speed);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " current limit set to "
                            + std::to_string(request->initparams.speed) + " amps";

    } else if (params == "set_current_limit_mode") {
        titan_->SetCurrentLimitMode(motor, static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " current limit mode set to "
                            + std::to_string(request->initparams.int_value);

    // --- motor direction / inversion ---

    } else if (params == "invert_motor") {
        titan_->InvertMotor(motor);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " direction inverted";

    } else if (params == "invert_motor_direction") {
        titan_->InvertMotorDirection(motor);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " hardware direction inverted";

    } else if (params == "invert_motor_rpm") {
        titan_->InvertMotorRPM(motor);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " rpm sign inverted";

    } else if (params == "invert_encoder_direction") {
        titan_->InvertEncoderDirection(motor);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " encoder direction inverted";

    // --- read back ---

    } else if (params == "get_rpm") {
        response->success = true;
        response->message = std::to_string(titan_->GetRPM(motor));

    } else if (params == "get_encoder_count") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderCount(motor));

    } else if (params == "get_encoder_distance") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderDistance(motor));

    } else if (params == "get_target_rpm") {
        int16_t targets[4] = {0, 0, 0, 0};
        bool ok = titan_->GetTargetRPMFromDevice(targets);
        response->success = ok;
        response->message = std::to_string(targets[0]) + "," + std::to_string(targets[1]) + ","
                            + std::to_string(targets[2]) + "," + std::to_string(targets[3]);

    } else if (params == "get_cypher_angle") {
        response->success = true;
        response->message = std::to_string(titan_->GetCypherAngle(motor));

    } else if (params == "get_limit_switch") {
        bool state = titan_->GetLimitSwitch(motor, static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = state ? "triggered" : "open";

    } else if (params == "get_controller_temp") {
        response->success = true;
        response->message = std::to_string(titan_->GetControllerTemp());

    } else if (params == "get_firmware_version") {
        response->success = true;
        response->message = titan_->GetFirmwareVersion();

    } else if (params == "get_hardware_version") {
        response->success = true;
        response->message = titan_->GetHardwareVersion();

    } else if (params == "get_serial_number") {
        response->success = true;
        response->message = titan_->GetSerialNumber();

    } else if (params == "get_id") {
        response->success = true;
        response->message = std::to_string(titan_->GetID());

    } else {
        response->success = false;
        response->message = "unknown command '" + params + "'";
    }
}


void Titan::publish_encoders() {
    if (!enabled_) return;

    for (int i = 0; i < 4; i++) {
        if (motor_configs_[i].encoder_mode == EncoderMode::Quadrature) {
            std_msgs::msg::Float64 enc_msg;
            enc_msg.data = titan_->GetEncoderDistance(i);
            encoder_pubs_[i]->publish(enc_msg);

            std_msgs::msg::Float64 rpm_msg;
            rpm_msg.data = static_cast<double>(titan_->GetRPM(i));
            rpm_pubs_[i]->publish(rpm_msg);
        } else {
            std_msgs::msg::Float64 angle_msg;
            angle_msg.data = titan_->GetCypherAngle(i);
            angle_pubs_[i]->publish(angle_msg);
        }
    }
}

void Titan::resend_speeds() {
    if (!enabled_) return;
    for (int i = 0; i < 4; i++) {
        if (speeds_[i] != 0.0f) {
            titan_->SetSpeed(i, speeds_[i]);
        }
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Titan)
