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
 *     subscribe /name/m_N/cmd      (Float64, duty -1.0 to 1.0, pid type 0)
 *     subscribe /name/m_N/rpm_cmd  (Float64, target rpm, pid type 1 or 2)
 *
 * service: /name/titan_cmd (studica_control/SetData)
 *   see titan_component.h for the full command reference.
 */

#include "studica_control/titan_component.hpp"
#include <cmath>
#include <thread>

namespace studica_control {


std::vector<std::shared_ptr<rclcpp::Node>> Titan::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> titan_nodes;

    control->declare_parameter<std::vector<std::string>>("titan.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("titan.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string can_id_param     = "titan." + sensor + ".can_id";
        std::string motor_freq_param = "titan." + sensor + ".motor_freq";

        std::string encoder_rate_param      = "titan." + sensor + ".encoder_rate_hz";
        std::string motor_update_rate_param = "titan." + sensor + ".motor_update_rate_hz";
        std::string limit_switches_param      = "titan." + sensor + ".limit_switches";
        std::string enable_freshness_param   = "titan." + sensor + ".enable_freshness";
        std::string default_pid_type_param   = "titan." + sensor + ".default_pid_type";
        std::string scurve_profile_param      = "titan." + sensor + ".scurve_profile";

        control->declare_parameter<int> (can_id_param, -1);
        control->declare_parameter<int> (motor_freq_param, -1);
        control->declare_parameter<int> (encoder_rate_param, 20);
        control->declare_parameter<int> (motor_update_rate_param, 50);
        control->declare_parameter<bool>(limit_switches_param, false);
        control->declare_parameter<bool>(enable_freshness_param, false);
        control->declare_parameter<int> (default_pid_type_param, 0);
        control->declare_parameter<int> (scurve_profile_param, 0);   // 0=nav (symmetric S-curve), 1=teleop

        uint8_t  can_id               = static_cast<uint8_t>(control->get_parameter(can_id_param).as_int());
        uint16_t motor_freq           = static_cast<uint16_t>(control->get_parameter(motor_freq_param).as_int());
        int      encoder_rate_hz      = control->get_parameter(encoder_rate_param).as_int();
        int      motor_update_rate_hz = control->get_parameter(motor_update_rate_param).as_int();
        bool     limit_switches       = control->get_parameter(limit_switches_param).as_bool();
        bool     enable_freshness     = control->get_parameter(enable_freshness_param).as_bool();
        uint8_t  default_pid_type     = static_cast<uint8_t>(control->get_parameter(default_pid_type_param).as_int());
        uint8_t  scurve_profile       = static_cast<uint8_t>(control->get_parameter(scurve_profile_param).as_int());

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

        RCLCPP_INFO(control->get_logger(), "%s -> can_id: %d, motor_freq: %d hz, encoder: %dHz, resend: %dHz",
                    sensor.c_str(), can_id, motor_freq, encoder_rate_hz, motor_update_rate_hz);

        auto titan = std::make_shared<Titan>(vmx, sensor, can_id, motor_freq, motor_configs,
                                             encoder_rate_hz, motor_update_rate_hz,
                                             limit_switches, enable_freshness, default_pid_type,
                                             scurve_profile);
        titan_nodes.push_back(titan);
    }

    return titan_nodes;
}


Titan::Titan(const rclcpp::NodeOptions &options) : Node("titan_", options) {}


Titan::Titan(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID,
             const uint16_t &motor_freq, const std::array<MotorConfig, 4> &motor_configs,
             int encoder_rate_hz, int motor_update_rate_hz, bool limit_switches, bool enable_freshness,
             uint8_t default_pid_type, uint8_t scurve_profile)
    : Node(name), vmx_(vmx), canID_(canID), motor_freq_(motor_freq), motor_configs_(motor_configs),
      limit_switches_enabled_(limit_switches), freshness_enabled_(enable_freshness) {

    for (int i = 0; i < 4; i++)
        pid_type_[i] = default_pid_type;

    titan_ = std::make_shared<studica_driver::Titan>(canID_, motor_freq_, 1, vmx_);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/titan_cmd",
        std::bind(&Titan::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    for (int i = 0; i < 4; i++) {
        std::string prefix = name + "/m_" + std::to_string(i);

        // command subscriber — always created
        cmd_subs_[i] = this->create_subscription<std_msgs::msg::Float64>(
            prefix + "/cmd", 1,
            [this, i](std_msgs::msg::Float64::SharedPtr msg) {
                if (!enabled_) return;
                if (pid_type_[i] != 0) {
                    const double duty = msg->data;
                    const bool significant = std::abs(duty) > 1e-4;
                    if (!cmd_rejected_warned_[i]) {
                        cmd_rejected_warned_[i] = true;
                        RCLCPP_WARN(this->get_logger(),
                                             "m_%d: ignoring /cmd duty=%.3f (pid type %u); "
                                             "use /rpm_cmd or set pid type 0",
                                             i, duty, pid_type_[i]);
                    } else if (significant) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "m_%d: ignoring /cmd duty=%.3f (pid type %u); "
                                             "use /rpm_cmd or set pid type 0",
                                             i, duty, pid_type_[i]);
                    }
                    return;
                }
                speeds_[i] = static_cast<float>(msg->data);
                position_active_[i] = false;
            });

        rpm_cmd_subs_[i] = this->create_subscription<std_msgs::msg::Float64>(
            prefix + "/rpm_cmd", 1,
            [this, i](std_msgs::msg::Float64::SharedPtr msg) {
                if (!enabled_) return;
                if (pid_type_[i] == 0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "m_%d: rpm_cmd ignored; set pid type 1 or 2 first", i);
                    return;
                }
                target_rpm_[i] = static_cast<float>(msg->data);
                position_active_[i] = false;
            });

        // feedback publishers — depend on encoder mode
        // rpm is always published regardless of mode (shaft RPM — Titan does not know wheel size)
        rpm_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(prefix + "/rpm", 10);
        if (motor_configs_[i].encoder_mode == EncoderMode::Quadrature) {
            encoder_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(prefix + "/encoder", 10);
            RCLCPP_INFO(this->get_logger(), "  m_%d: quadrature -> /%s/encoder, /%s/rpm",
                        i, prefix.c_str(), prefix.c_str());
        } else {
            angle_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(prefix + "/angle", 10);
            RCLCPP_INFO(this->get_logger(), "  m_%d: absolute   -> /%s/angle, /%s/rpm (shaft rpm)",
                        i, prefix.c_str(), prefix.c_str());
        }

        // limit switch publishers — only created when limit_switches: true
        if (limit_switches_enabled_) {
            limit_fwd_pubs_[i] = this->create_publisher<std_msgs::msg::Bool>(prefix + "/limit_fwd", 10);
            limit_rev_pubs_[i] = this->create_publisher<std_msgs::msg::Bool>(prefix + "/limit_rev", 10);
            RCLCPP_INFO(this->get_logger(), "  m_%d: limit switches -> /%s/limit_fwd, /%s/limit_rev",
                        i, prefix.c_str(), prefix.c_str());
        }
    }

    encoder_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / encoder_rate_hz),
        std::bind(&Titan::publish_encoders, this));

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / motor_update_rate_hz),
        std::bind(&Titan::resend_speeds, this));

    for (int i = 0; i < 4; i++) {
        titan_->ConfigureEncoder(i, motor_configs_[i].dist_per_tick);
        titan_->ResetEncoder(i);
        if (motor_configs_[i].invert_motor)   titan_->InvertMotorDirection(i);
        if (motor_configs_[i].invert_encoder) titan_->InvertEncoderDirection(i);
        if (motor_configs_[i].invert_rpm)     titan_->InvertMotorRPM(i);
    }

    titan_->SetPIDType(default_pid_type);

    // Velocity ramp profile (MCV2): 0=nav (symmetric S-curve), 1=teleop (smooth accel, instant decel).
    titan_->SetRampProfile(scurve_profile);

    // The Titan requires ~1s after initial CONFIG_MOTOR frames before it will
    // accept the enable command. See drivers/examples/titan_example/titan_example.cpp.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    titan_->Enable(true);
    enabled_ = true;

    RCLCPP_INFO(this->get_logger(), "titan ready. can_id: %d, freq: %d hz, pid_type: %u, scurve_profile: %u (%s)",
                canID_, motor_freq_, static_cast<unsigned>(default_pid_type),
                static_cast<unsigned>(scurve_profile), scurve_profile ? "teleop" : "nav");
}

Titan::~Titan() {}

void Titan::resume_motion_resend(int motor, bool all_motors)
{
    if (all_motors) {
        for (int i = 0; i < 4; i++)
            position_active_[i] = false;
    } else {
        position_active_[motor] = false;
    }
}


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
        resume_motion_resend(0, true);
        for (int i = 0; i < 4; i++) {
            speeds_[i] = 0.0f;
            target_rpm_[i] = 0.0f;
        }
        enabled_ = false;
        titan_->Enable(false);
        response->success = true;
        response->message = "titan disabled";

    } else if (params == "set_speed") {
        if (pid_type_[motor] != 0) {
            response->success = false;
            response->message = "motor " + std::to_string(motor) + " is in pid type "
                                + std::to_string(pid_type_[motor])
                                + "; switch to pid type 0 before open-loop duty commands";
        } else {
            float speed = request->initparams.speed;
            resume_motion_resend(motor, false);
            speeds_[motor] = speed;
            titan_->SetSpeed(motor, speed);
            response->success = true;
            response->message = "motor " + std::to_string(motor) + " speed set to " + std::to_string(speed);
        }

    } else if (params == "set_speed_all") {
        for (int i = 0; i < 4; i++) {
            if (pid_type_[i] != 0) {
                response->success = false;
                response->message = "motor " + std::to_string(i) + " is in pid type "
                                    + std::to_string(pid_type_[i])
                                    + "; switch to pid type 0 before open-loop duty commands";
                return;
            }
        }
        float speed = request->initparams.speed;
        resume_motion_resend(0, true);
        for (int i = 0; i < 4; i++)
            speeds_[i] = speed;
        titan_->SetSpeedAll(static_cast<double>(speed));
        response->success = true;
        response->message = "all motors set to " + std::to_string(speed);

    } else if (params == "stop") {
        resume_motion_resend(motor, false);
        if (pid_type_[motor] == 0) {
            speeds_[motor] = 0.0f;
            titan_->SetSpeed(motor, 0.0);
        } else {
            target_rpm_[motor] = 0.0f;
            titan_->SetTargetVelocity(motor, 0.0f);
        }
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " stopped";

    } else if (params == "disable_motor") {
        speeds_[motor] = 0.0f;
        target_rpm_[motor] = 0.0f;
        resume_motion_resend(motor, false);
        titan_->DisableMotor(motor);
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " disabled";

    } else if (params == "set_motor_stop_mode") {
        titan_->SetMotorStopMode(static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = "stop mode set to " + std::to_string(request->initparams.int_value);

    // --- closed loop velocity / position control (titan2 firmware) ---

    } else if (params == "set_target_velocity") {
        // closed-loop only: firmware ignores SetTargetVelocity when pid type is OFF
        if (pid_type_[motor] == 0) {
            response->success = false;
            response->message = "motor " + std::to_string(motor)
                                + " is in pid type 0 (OFF); set pid type 1 or 2 before set_target_velocity";
        } else {
            float rpm = request->initparams.speed;
            resume_motion_resend(motor, false);
            target_rpm_[motor] = rpm;
            titan_->SetTargetVelocity(motor, rpm);
            response->success = true;
            response->message = "motor " + std::to_string(motor) + " target velocity set to " + std::to_string(rpm) + " rpm";
        }

    } else if (params == "set_target_distance") {
        if (pid_type_[motor] == 0) {
            response->success = false;
            response->message = "motor " + std::to_string(motor)
                                + " is in pid type 0 (OFF); set pid type 1 or 2 before set_target_distance";
        } else {
            position_active_[motor] = true;
            titan_->SetTargetDistance(motor, static_cast<int32_t>(request->initparams.int_value));
            response->success = true;
            response->message = "motor " + std::to_string(motor) + " target distance set to "
                                + std::to_string(request->initparams.int_value) + " counts";
        }

    } else if (params == "set_target_angle") {
        if (pid_type_[motor] == 0) {
            response->success = false;
            response->message = "motor " + std::to_string(motor)
                                + " is in pid type 0 (OFF); set pid type 1 or 2 before set_target_angle";
        } else {
            position_active_[motor] = true;
            titan_->SetTargetAngle(motor, static_cast<double>(request->initparams.speed));
            response->success = true;
            response->message = "motor " + std::to_string(motor) + " target angle set to "
                                + std::to_string(request->initparams.speed) + " degrees";
        }

    } else if (params == "set_position_hold") {
        if (pid_type_[motor] == 0) {
            response->success = false;
            response->message = "motor " + std::to_string(motor)
                                + " is in pid type 0 (OFF); set pid type 1 or 2 before set_position_hold";
        } else {
            position_active_[motor] = request->initparams.hold;
            if (!request->initparams.hold)
                resume_motion_resend(motor, false);
            titan_->SetPositionHold(motor, request->initparams.hold);
            response->success = true;
            response->message = "motor " + std::to_string(motor) + " position hold "
                                + std::string(request->initparams.hold ? "enabled" : "disabled");
        }

    // --- pid tuning (titan2 firmware) ---
    } else if (params == "set_pid_type") {
        uint8_t type = static_cast<uint8_t>(request->initparams.int_value);
        titan_->SetPIDType(type);
        for (int i = 0; i < 4; i++) {
            pid_type_[i] = type;
            position_active_[i] = false;
            if (type == 0)
                target_rpm_[i] = 0.0f;
        }
        response->success = true;
        response->message = "pid type set to " + std::to_string(type);

    } else if (params == "set_sensitivity") {
        titan_->SetSensitivity(motor, static_cast<uint8_t>(request->initparams.int_value));
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " sensitivity set to "
                            + std::to_string(request->initparams.int_value);

    } else if (params == "set_scurve_profile") {
        uint8_t profile = static_cast<uint8_t>(request->initparams.int_value);  // 0=nav, 1=teleop (global)
        titan_->SetRampProfile(profile);
        response->success = true;
        response->message = std::string("scurve profile set to ") + (profile ? "teleop" : "nav");

    } else if (params == "autotune") {
        titan_->AutotuneAll();
        response->success = true;
        response->message = "autotune started on all motors";

    } else if (params == "autotune_symmetric") {
        char signs[4];
        std::string signs_str;
        for (int i = 0; i < 4; i++) {
            signs[i] = motor_configs_[i].invert_motor ? '-' : '+';
            signs_str += signs[i];
        }
        titan_->AutotuneAll(signs);
        response->success = true;
        response->message = "symmetric autotune started (" + signs_str + " from invert_motor)";

    } else if (params == "autotune_motor") {
        titan_->AutotuneMotor(motor);
        response->success = true;
        response->message = "autotune started on motor " + std::to_string(motor);

    } else if (params == "set_motor_pid_type") {
        uint8_t type = static_cast<uint8_t>(request->initparams.int_value);
        titan_->SetMotorPIDType(motor, type);
        pid_type_[motor] = type;
        position_active_[motor] = false;
        if (type == 0)
            target_rpm_[motor] = 0.0f;
        response->success = true;
        response->message = "motor " + std::to_string(motor) + " pid type set to "
                            + std::to_string(type);

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
        float targets[4] = {0.f, 0.f, 0.f, 0.f};
        bool ok = titan_->TryGetTargetRPMFromAll(targets);
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

    // Quadrature encoder
    for (int i = 0; i < 4; i++) {
        if (motor_configs_[i].encoder_mode != EncoderMode::Quadrature) continue;
        if (freshness_enabled_) {
            double enc_dist; bool enc_fresh;
            titan_->GetEncoderDistanceFresh(i, enc_dist, enc_fresh, nullptr);
            if (enc_fresh) {
                stale_count_[i] = 0;
                std_msgs::msg::Float64 msg; msg.data = enc_dist;
                encoder_pubs_[i]->publish(msg);
            } else {
                if (++stale_count_[i] == 5)
                    RCLCPP_WARN(get_logger(), "m_%d: encoder stale for %d consecutive cycles",
                                i, stale_count_[i]);
            }
        } else {
            std_msgs::msg::Float64 msg; msg.data = titan_->GetEncoderDistance(i);
            encoder_pubs_[i]->publish(msg);
        }
    }

    // Cypher Max
    {
        if (freshness_enabled_) {
            double angles[4] = {};
            bool cypher_fresh;
            titan_->GetCypherAnglesFresh(angles, cypher_fresh, nullptr);
            if (!cypher_fresh) {
                if (++cypher_stale_count_ == 5)
                    RCLCPP_WARN(get_logger(), "cypher: stale for %d consecutive cycles", cypher_stale_count_);
            } else {
                cypher_stale_count_ = 0;
                for (int i = 0; i < 4; i++) {
                    if (motor_configs_[i].encoder_mode != EncoderMode::Absolute) continue;
                    std_msgs::msg::Float64 msg; msg.data = angles[i];
                    angle_pubs_[i]->publish(msg);
                }
            }
        } else {
            for (int i = 0; i < 4; i++) {
                if (motor_configs_[i].encoder_mode != EncoderMode::Absolute) continue;
                std_msgs::msg::Float64 msg; msg.data = titan_->GetCypherAngle(i);
                angle_pubs_[i]->publish(msg);
            }
        }
    }

    // RPM
    for (int i = 0; i < 4; i++) {
        if (freshness_enabled_) {
            float rpm_val; bool rpm_fresh;
            titan_->GetRPMFresh(i, rpm_val, rpm_fresh, nullptr);
            if (rpm_fresh) {
                rpm_stale_count_[i] = 0;
                std_msgs::msg::Float64 msg; msg.data = static_cast<double>(rpm_val);
                rpm_pubs_[i]->publish(msg);
            } else {
                if (++rpm_stale_count_[i] == 5)
                    RCLCPP_WARN(get_logger(), "m_%d: rpm stale for %d consecutive cycles",
                                i, rpm_stale_count_[i]);
            }
        } else {
            std_msgs::msg::Float64 msg; msg.data = static_cast<double>(titan_->GetRPM(i));
            rpm_pubs_[i]->publish(msg);
        }
    }

    // Limit switches
    if (limit_switches_enabled_) {
        bool fwd[4] = {}, rev[4] = {};
        bool do_publish = true;

        if (freshness_enabled_) {
            bool fresh;
            titan_->GetLimitSwitchesFresh(fwd, rev, fresh, nullptr);
            if (!fresh) {
                if (++ls_stale_count_ == 5)
                    RCLCPP_WARN(get_logger(), "limit switches: stale for %d consecutive cycles",
                                ls_stale_count_);
                do_publish = false;
            } else {
                ls_stale_count_ = 0;
            }
        } else {
            for (int i = 0; i < 4; i++) {
                fwd[i] = titan_->GetLimitSwitch(i, 0);
                rev[i] = titan_->GetLimitSwitch(i, 1);
            }
        }

        if (do_publish) {
            for (int i = 0; i < 4; i++) {
                std_msgs::msg::Bool fwd_msg, rev_msg;
                fwd_msg.data = !fwd[i];  // active-low: invert so true = triggered
                rev_msg.data = !rev[i];
                limit_fwd_pubs_[i]->publish(fwd_msg);
                limit_rev_pubs_[i]->publish(rev_msg);
            }
        }
    }
}

void Titan::resend_speeds() {
    if (!enabled_) return;
    for (int i = 0; i < 4; i++) {
        if (position_active_[i])
            continue;
        if (pid_type_[i] == 0)
            titan_->SetSpeed(i, speeds_[i]);
        else
            titan_->SetTargetVelocity(i, target_rpm_[i]);
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Titan)
