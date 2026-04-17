/*
 * sharp_component.h
 *
 * ros2 component for a sharp gp2y infrared range sensor.
 * the sensor emits an infrared beam and measures how long it takes to reflect
 * back, giving a distance reading. effective range is roughly 10cm to 80cm.
 * readings outside this range are reported as infinity. supports multiple sensors.
 *
 * topic (publishes): <topic> (sensor_msgs/Range)
 *   distance in metres, published at 20hz.
 *   radiation_type: infrared
 *   valid range: 0.1m – 0.8m
 *
 * service: sharp_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_distance — return the current distance in centimetres as a string
 */

#ifndef SHARP_COMPONENT_H
#define SHARP_COMPONENT_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "sharp.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

// sharp — infrared range sensor node. one instance per sensor.
class Sharp : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per sharp entry in the list
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Sharp(const rclcpp::NodeOptions &options);

    // main constructor — connects to the sensor and sets up topics/services
    Sharp(std::shared_ptr<VMXPi> vmx, const std::string &name,
          VMXChannelIndex port, const std::string &topic);

    ~Sharp();

private:
    std::shared_ptr<studica_driver::Sharp> sharp_;
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex port_;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // receives the raw service request and forwards it to cmd()
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    // parses the command string and returns the requested reading
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

    // reads distance from the sensor and publishes a range message
    void publish_range();
};

} // namespace studica_control

#endif // SHARP_COMPONENT_H
