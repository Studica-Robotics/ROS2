/*
 * titan_component.h
 *
 * ros2 component for the studica titan motor controller.
 * the titan controls up to 4 brushed dc motors over can bus. each titan
 * has a unique can id so you can run multiple on the same robot.
 *
 * topic (publishes): <topic> (std_msgs/Float32MultiArray)
 *   encoder counts for all 4 motors, published at 20hz.
 *   data[0..3] = raw encoder tick counts (float, but whole numbers)
 *
 * service: <name>/titan_cmd (studica_control/SetData)
 *   send commands via the params field. initparams carries extra values:
 *     n_encoder  — which motor to target (0–3)
 *     speed      — floating point value (duty -1.0..1.0, rpm, angle, amps)
 *     int_value  — integer value (mode codes, pid type, resolution, sensitivity)
 *     hold       — boolean (used by set_position_hold)
 *     dist_per_tick — distance per encoder tick for configure_encoder
 *
 *   basic control:
 *     enable               — power on the controller (required before any motor command)
 *     disable              — power off the controller (all motors stop immediately)
 *     set_speed            — set one motor to a duty cycle (-1.0 to 1.0)
 *                            requires: n_encoder, speed
 *     set_speed_all        — set all 4 motors to the same duty cycle
 *                            requires: speed
 *     stop                 — set one motor speed to 0
 *                            requires: n_encoder
 *     disable_motor        — cut power to one motor only
 *                            requires: n_encoder
 *     set_motor_stop_mode  — set how motors stop (0=coast, 1=brake)
 *                            requires: int_value
 *
 *   closed loop velocity / position control (titan2 firmware):
 *     set_target_velocity  — run a motor at a target speed in rpm
 *                            requires: n_encoder, speed (rpm as float, cast to int16)
 *     set_target_distance  — drive a motor to a target position in encoder counts
 *                            requires: n_encoder, int_value (count)
 *     set_target_angle     — drive a motor to a target angle in degrees
 *                            requires: n_encoder, speed (degrees)
 *     set_position_hold    — lock a motor at its current position
 *                            requires: n_encoder, hold (true=hold on, false=hold off)
 *
 *   pid tuning (titan2 firmware):
 *     set_pid_type         — select the pid algorithm (see firmware docs for type codes)
 *                            requires: int_value
 *     set_sensitivity      — tune how aggressively the pid responds (0–255)
 *                            requires: n_encoder, int_value
 *     autotune             — automatically tune pid for all motors
 *
 *   encoder configuration:
 *     setup_encoder        — prepare an encoder channel for use
 *                            requires: n_encoder
 *     configure_encoder    — set distance per tick for odometry
 *                            requires: n_encoder, dist_per_tick
 *     set_encoder_resolution — set encoder counts per revolution
 *                            requires: n_encoder, int_value (cpr)
 *     reset_encoder        — reset one encoder count to zero
 *                            requires: n_encoder
 *
 *   current limiting:
 *     set_current_limit    — limit how many amps a motor can draw
 *                            requires: n_encoder, speed (amps)
 *     set_current_limit_mode — set current limiter behavior (see firmware docs)
 *                            requires: n_encoder, int_value
 *
 *   motor direction / inversion:
 *     invert_motor         — flip the positive direction of a motor
 *                            requires: n_encoder
 *     invert_motor_direction — same as invert_motor (hardware direction flip)
 *                            requires: n_encoder
 *     invert_motor_rpm     — flip the sign of reported rpm only
 *                            requires: n_encoder
 *     invert_encoder_direction — flip the sign of encoder count only
 *                            requires: n_encoder
 *
 *   read back:
 *     get_rpm              — current rpm for one motor
 *                            requires: n_encoder
 *     get_encoder_count    — raw tick count for one motor
 *                            requires: n_encoder
 *     get_encoder_distance — distance traveled for one motor
 *                            requires: n_encoder
 *     get_target_rpm       — target rpm for all 4 motors (comma separated)
 *     get_cypher_angle     — angle from a cypher encoder on a given port
 *                            requires: n_encoder (port index)
 *     get_limit_switch     — limit switch state for one motor and direction
 *                            requires: n_encoder, int_value (0=forward, 1=reverse)
 *     get_controller_temp  — mcu temperature in celsius
 *     get_firmware_version — titan firmware version string
 *     get_hardware_version — titan hardware version string
 *     get_serial_number    — titan serial number string
 *     get_id               — titan device id
 */

#ifndef TITAN_COMPONENT_H
#define TITAN_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "studica_control/srv/set_data.hpp"
#include "titan.h"
#include "VMXPi.h"

namespace studica_control {

// titan — motor controller node. one instance per physical titan board.
class Titan : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per titan entry in the list
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Titan(const rclcpp::NodeOptions &options);

    // main constructor — connects to the titan and sets up topics/services
    Titan(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID,
          const uint16_t &motor_freq, const std::string &topic);

    ~Titan();

private:
    std::shared_ptr<studica_driver::Titan> titan_;
    std::shared_ptr<VMXPi> vmx_;
    uint8_t canID_;
    uint16_t motor_freq_;
    float dist_per_tick_;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // receives the raw service request and forwards it to cmd()
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // parses the command string and calls the appropriate titan driver function
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request,
             std::shared_ptr<studica_control::srv::SetData::Response> response);

    // reads encoder counts from all 4 motors and publishes them
    void publish_encoders();
};

}  // namespace studica_control

#endif  // TITAN_COMPONENT_H
