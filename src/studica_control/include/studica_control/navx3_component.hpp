/*
 * navx3_component.hpp
 *
 * ros2 component for the studica navx3 imu, over CAN or USB. separate from the imu
 * component (the navx built into the vmx-pi); either or both can be enabled.
 *
 * topic (publishes): <topic> (sensor_msgs/Imu), default /<name>/imu, at update_rate_hz
 *   orientation      — quaternion in the navx3's own axes (yaw counter-clockwise positive);
 *                      9-axis (with magnetometer) when yaw_axes is 9, else 6-axis
 *   angular_velocity — radians per second
 *   linear_accel     — metres per second squared, as the navx3 reports it
 *   describe how the navx3 is mounted with a tf from base_link to frame_id.
 *   a field the navx3 hasn't sent recently gets covariance[0] = -1 (not provided);
 *   otherwise each covariance diagonal comes from params (all zero = unknown).
 *
 * service: /<name>/navx3_cmd (studica_control/SetData)
 *   reset_yaw     — zero the yaw at the current heading
 *   get_data      — current yaw, pitch, roll and temperature as a string
 *   get_info      — firmware version and sensor unique ID
 *   self_test     — run the imu self test (~4 s, keep the robot still; no data meanwhile)
 *
 * params (per sensor in params.yaml):
 *   transport (can|usb), can_id, serial_port, topic, frame_id,
 *   update_rate_hz (1-200: sensor output rate and publish rate), yaw_axes (6|9),
 *   orientation_variance, angular_velocity_variance, linear_acceleration_variance
 */

#ifndef NAVX3_COMPONENT_H
#define NAVX3_COMPONENT_H

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "navx3.hpp"
#include "navx3_usb.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

struct Navx3Settings {
    bool usb = false;
    uint8_t can_id = 0;
    std::string serial_port = "/dev/ttyACM0";
    std::string topic;
    std::string frame_id = "navx3_link";
    int update_rate_hz = 50;
    int yaw_axes = 6;
    double orientation_variance = 0.0;
    double angular_velocity_variance = 0.0;
    double linear_acceleration_variance = 0.0;
};

class Navx3 : public rclcpp::Node {
public:
    // reads params.yaml and creates one node per entry in navx3.sensors
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(
        rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    // composable node constructor — used when loading as a ros2 plugin
    explicit Navx3(const rclcpp::NodeOptions &options);

    Navx3(std::shared_ptr<VMXPi> vmx, const std::string &name, const Navx3Settings &settings);

    ~Navx3();

private:
    std::shared_ptr<studica_driver::Navx3> navx_can_;
    std::shared_ptr<studica_driver::Navx3Usb> navx_usb_;
    Navx3Settings settings_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    void publish_data();

    // Latest sample on whichever transport is in use; false if nothing recent.
    bool read(studica_driver::Navx3Data &data);
    // NAVX3_OK or the transport's error code, as text.
    static std::string status_string(int status);
};

} // namespace studica_control

#endif // NAVX3_COMPONENT_H
