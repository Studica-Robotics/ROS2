/*
 * ultrasonic_component.h
 *
 * ros2 component for an hc-sr04-style ultrasonic range sensor.
 * the sensor sends out a sound pulse and measures how long it takes to bounce
 * back from an obstacle, giving a distance reading. effective range is roughly
 * 2cm to 4m. readings outside this range are reported as infinity.
 * supports multiple sensors by adding entries to the sensors list in params.yaml.
 *
 * topic (publishes): <topic> (sensor_msgs/Range)
 *   distance in metres, published at 20hz.
 *   radiation_type: ultrasound
 *   valid range: 0.02m – 4.0m
 *
 * service: ultrasonic_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_distance             — return distance in millimetres as a string
 *     get_distance_inches      — return distance in inches as a string
 *     get_distance_millimeters — return distance in millimetres as a string
 */

#ifndef ULTRASONIC_COMPONENT_H
#define ULTRASONIC_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "studica_control/srv/set_data.hpp"
#include "ultrasonic.h"
#include "VMXPi.h"

namespace studica_control {

// ultrasonic — sonar range sensor node. one instance per sensor.
class Ultrasonic : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per ultrasonic entry in the list
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Ultrasonic(const rclcpp::NodeOptions &options);

    // main constructor — connects to the sensor and sets up topics/services
    Ultrasonic(std::shared_ptr<VMXPi> vmx, const std::string &name,
               VMXChannelIndex ping, VMXChannelIndex echo, const std::string &topic);

    ~Ultrasonic();

private:
    std::shared_ptr<studica_driver::Ultrasonic> ultrasonic_;
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex ping_;  // trigger pin — sends the sound pulse
    VMXChannelIndex echo_;  // echo pin — receives the reflected pulse

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // receives the raw service request and forwards it to cmd()
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // parses the command string and returns the requested distance value
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

    // triggers a ping, reads the result, and publishes a range message
    void publish_range();
};

} // namespace studica_control

#endif // ULTRASONIC_COMPONENT_H
