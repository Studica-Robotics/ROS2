/*
 * titan_component.cpp
 *
 * ros2 component for the titan motor controller.
 * the titan is a can bus motor controller that supports up to 4 motors.
 * it publishes encoder counts for all 4 motor channels and accepts
 * speed, position, and pid commands through a service.
 *
 * topic:   <topic> (std_msgs/Float32MultiArray)
 *            array of 4 floats — encoder count for motors 0-3, published at 20hz
 *
 * service: <name>/titan_cmd (studica_control/SetData)
 *   see titan_component.h for the full command reference.
 *   key initparams fields used across commands:
 *     n_encoder  — motor/channel index (0–3)
 *     speed      — float value: duty (-1.0..1.0), rpm, angle in degrees, or amps
 *     int_value  — integer value: mode codes, pid type, cpr, sensitivity, direction
 *     hold       — bool: used by set_position_hold
 *     dist_per_tick — distance per tick for configure_encoder
 */

#include "studica_control/titan_component.h"

namespace studica_control {


// reads titan parameters from params.yaml and creates one node per entry
// in the sensors list. each node connects to a separate titan controller.
std::vector<std::shared_ptr<rclcpp::Node>> Titan::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> titan_nodes;

    control->declare_parameter<std::vector<std::string>>("titan.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("titan.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string can_id_param     = "titan." + sensor + ".can_id";
        std::string motor_freq_param = "titan." + sensor + ".motor_freq";
        std::string topic_param      = "titan." + sensor + ".topic";

        control->declare_parameter<int>(can_id_param, -1);
        control->declare_parameter<int>(motor_freq_param, -1);
        control->declare_parameter<std::string>(topic_param, "");

        uint8_t can_id      = control->get_parameter(can_id_param).as_int();
        uint16_t motor_freq = control->get_parameter(motor_freq_param).as_int();
        std::string topic   = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> can_id: %d, motor_freq: %d, topic: %s",
                    sensor.c_str(), can_id, motor_freq, topic.c_str());

        auto titan = std::make_shared<Titan>(vmx, sensor, can_id, motor_freq, topic);
        titan_nodes.push_back(titan);
    }

    return titan_nodes;
}


// composable node constructor — used when loading as a ros2 plugin
Titan::Titan(const rclcpp::NodeOptions &options) : Node("titan_", options) {}


// main constructor — connects to the titan controller and sets up
// the publisher, service, and periodic timer
Titan::Titan(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID,
             const uint16_t &motor_freq, const std::string &topic)
    : Node(name), vmx_(vmx), canID_(canID), motor_freq_(motor_freq) {

    titan_ = std::make_shared<studica_driver::Titan>(canID_, motor_freq_, 1, vmx_);

    // service for sending commands to the titan
    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/titan_cmd",
        std::bind(&Titan::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes encoder counts for all 4 motor channels at 20hz
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic, 10);
    encoder_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Titan::publish_encoders, this));

    // resends current speed commands at 100hz to satisfy the titan CAN watchdog
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Titan::resend_speeds, this));

    // configure all 4 encoders with a distance-per-tick of 1 and clear counts
    for (int i = 0; i < 4; i++) {
        titan_->ConfigureEncoder(i, 1);
        titan_->ResetEncoder(i);
    }

    titan_->SetPIDType(0);  // type 0 = duty-cycle mode (SetSpeed uses raw PWM duty, not velocity PID)
    titan_->Enable(true);
    enabled_ = true;

    RCLCPP_INFO(this->get_logger(), "titan ready. can_id: %d, freq: %d hz", canID_, motor_freq_);
}

Titan::~Titan() {}


void Titan::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    cmd(request->params, request, response);
}


// dispatches incoming service commands to the appropriate titan driver function.
// see the header for a full description of each command and its required initparams.
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
        // int_value: 0 = coast, 1 = brake (see firmware docs)
        titan_->SetMotorStopMode(static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = "stop mode set to " + std::to_string(request->initparams.int_value);

    // --- closed loop velocity / position control (titan2 firmware) ---

    } else if (params == "set_target_velocity") {
        // speed field carries the target in rpm — cast to int16
        int16_t rpm = static_cast<int16_t>(request->initparams.speed);
        titan_->SetTargetVelocity(motor, rpm);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " target velocity set to " + std::to_string(rpm) + " rpm";

    } else if (params == "set_target_distance") {
        // int_value carries the target in encoder counts
        titan_->SetTargetDistance(motor, static_cast<int32_t>(request->initparams.int_value));
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " target distance set to "
                            + std::to_string(request->initparams.int_value) + " counts";

    } else if (params == "set_target_angle") {
        // speed field carries the target angle in degrees
        titan_->SetTargetAngle(motor, static_cast<double>(request->initparams.speed));
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " target angle set to "
                            + std::to_string(request->initparams.speed) + " degrees";

    } else if (params == "set_position_hold") {
        // hold field: true = lock position, false = release
        titan_->SetPositionHold(motor, request->initparams.hold);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " position hold "
                            + std::string(request->initparams.hold ? "enabled" : "disabled");

    // --- pid tuning (titan2 firmware) ---

    } else if (params == "set_pid_type") {
        // int_value: pid algorithm type — see firmware documentation for valid values
        titan_->SetPIDType(static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = "pid type set to " + std::to_string(request->initparams.int_value);

    } else if (params == "set_sensitivity") {
        // int_value: 0–255, controls how aggressively the pid responds
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
        // int_value: counts per revolution (cpr) for this encoder
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
        // speed field carries the limit in amps
        titan_->SetCurrentLimit(motor, request->initparams.speed);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " current limit set to "
                            + std::to_string(request->initparams.speed) + " amps";

    } else if (params == "set_current_limit_mode") {
        // int_value: mode code — see firmware documentation for valid values
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
        // int_value: 0 = forward direction, 1 = reverse direction
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
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(4);
    for (int i = 0; i < 4; i++) {
        msg.data[i] = static_cast<float>(titan_->GetEncoderCount(i));
    }
    publisher_->publish(msg);
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
