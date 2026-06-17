/*
 * colore_component.hpp
 *
 * ROS2 component for the Studica Colore color sensor (CAN or USB).
 *
 * topics (publish):
 *   /<name>/color        (std_msgs/ColorRGBA)            - always on, normalized 0..1
 *   /<name>/color_info   (studica_control/ColoreColorMsg) - optional, debug/echo
 *   /<name>/xyz          (geometry_msgs/Vector3Stamped)   - optional, CIE XYZ
 *   /<name>/raw_spectrum (studica_control/ColoreRawMsg)   - optional, 18ch (USB only)
 *
 * service: /<name>/colore_cmd (studica_control/SetData)
 *   get_config, set_brightness, set_format, set_sample_time,
 *   set_measmode_auto, set_measmode_off, get_color
 *
 * params (per sensor):
 *   transport (can|usb), can_id, serial_port, frame_id, publish_rate_hz,
 *   publish_outputs, color_format, brightness, sample_time_ms
 */
#ifndef COLORE_COMPONENT_H
#define COLORE_COMPONENT_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "colore.hpp"
#include "colore_usb.hpp"
#include "studica_control/msg/colore_color_msg.hpp"
#include "studica_control/msg/colore_raw_msg.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

enum class ColoreTransport { Can, Usb };

struct ColorePublishOutputs {
    bool info{false};
    bool xyz{false};
    bool raw{false};
};

class Colore : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(
        rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    explicit Colore(const rclcpp::NodeOptions &options);

    Colore(std::shared_ptr<VMXPi> vmx, const std::string &name,
           ColoreTransport transport, uint8_t can_id, const std::string &serial_port,
           const std::string &frame_id, int publish_rate_hz,
           const std::vector<std::string> &publish_outputs,
           const std::string &color_format, int brightness, int sample_time_ms);

    ~Colore();

private:
    ColoreTransport transport_{ColoreTransport::Can};
    std::shared_ptr<studica_driver::Colore> colore_can_;
    std::shared_ptr<studica_driver::ColoreUsb> colore_usb_;
    std::shared_ptr<VMXPi> vmx_;
    std::string frame_id_;
    std::string color_format_;
    int publish_rate_hz_;
    ColorePublishOutputs outputs_;

    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr color_publisher_;
    rclcpp::Publisher<studica_control::msg::ColoreColorMsg>::SharedPtr info_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr xyz_publisher_;
    rclcpp::Publisher<studica_control::msg::ColoreRawMsg>::SharedPtr raw_publisher_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(const std::string &params,
             const studica_control::srv::SetData::Request &request,
             std::shared_ptr<studica_control::srv::SetData::Response> response);

    void publish_color();

    static ColoreTransport parse_transport(const std::string &value);
    static ColorePublishOutputs parse_publish_outputs(const std::vector<std::string> &names);
    static studica_driver::Colore::ColorFormat parse_color_format(const std::string &value);
    // Standard CIE XYZ (D65) -> sRGB 0..1, gamma-encoded.
    static void xyz_to_srgb(float X, float Y, float Z, float &r, float &g, float &b);
};

} // namespace studica_control

#endif // COLORE_COMPONENT_H
