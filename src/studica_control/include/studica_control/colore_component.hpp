/*
 * colore_component.hpp
 *
 * ROS2 component for the Studica Colore color sensor (CAN or USB).
 *
 * topics (publish):
 *   /<name>/color        (std_msgs/ColorRGBA)             - always on, normalized 0..1
 *   /<name>/color_info   (studica_control/ColoreColorMsg) - optional, debug/echo (incl. CIE XYZ)
 *   /<name>/color_match  (studica_control/ColoreMatch)    - optional, nearest reference color in xy
 *
 * service: /<name>/colore_cmd (studica_control/SetData)
 *   get_config, set_brightness,
 *   learn_color,<name> (snapshot current xy as a reference for matching)
 *
 * params (per sensor):
 *   transport (can|usb), can_id, serial_port, publish_rate_hz,
 *   publish_outputs, brightness,
 *   measmode (off|auto|fixed), measmode_z_mm,
 *   match_references, match_<name>_xy, match_threshold
 */
#ifndef COLORE_COMPONENT_H
#define COLORE_COMPONENT_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "colore.hpp"
#include "colore_usb.hpp"
#include "studica_control/msg/colore_color_msg.hpp"
#include "studica_control/msg/colore_match.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

enum class ColoreTransport { Can, Usb };

struct ColorePublishOutputs {
    bool info{false};
    bool match{false};
};

struct ColoreReference { std::string label; float x,y; };

class Colore : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(
        rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    explicit Colore(const rclcpp::NodeOptions &options);

    Colore(std::shared_ptr<VMXPi> vmx, const std::string &name,
           ColoreTransport transport, uint8_t can_id, const std::string &serial_port,
           int publish_rate_hz,
           const std::vector<std::string> &publish_outputs,
           int brightness,
           const std::string &measmode, float measmode_z_mm,
           const std::vector<ColoreReference> &references = {}, float match_threshold = 0.05f);

    ~Colore();

private:
    ColoreTransport transport_{ColoreTransport::Can};
    std::shared_ptr<studica_driver::Colore> colore_can_;
    std::shared_ptr<studica_driver::ColoreUsb> colore_usb_;
    std::shared_ptr<VMXPi> vmx_;
    int publish_rate_hz_;
    ColorePublishOutputs outputs_;
    std::vector<ColoreReference> references_;
    float match_threshold_{0.05f};
    // Latest chromaticity, cached so the learn_color command can snapshot it.
    float last_x_{0.0f}, last_y_{0.0f};
    bool have_last_xy_{false};

    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr color_publisher_;
    rclcpp::Publisher<studica_control::msg::ColoreColorMsg>::SharedPtr info_publisher_;
    rclcpp::Publisher<studica_control::msg::ColoreMatch>::SharedPtr match_publisher_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(const std::string &params,
             const studica_control::srv::SetData::Request &request,
             std::shared_ptr<studica_control::srv::SetData::Response> response);

    void publish_color();

    // Apply the measurement mode at startup. mode is off|auto|fixed (case-insensitive);
    // z_mm is the fixed user Z (used only for "fixed").
    void apply_measmode(const std::string &mode, float z_mm);

    static ColoreTransport parse_transport(const std::string &value);
    static ColorePublishOutputs parse_publish_outputs(const std::vector<std::string> &names);
    // Standard CIE XYZ (D65) -> sRGB 0..1, gamma-encoded.
    static void xyz_to_srgb(float X, float Y, float Z, float &r, float &g, float &b);
    // CIE XYZ -> xy chromaticity. Returns false if X+Y+Z ~ 0 (no light).
    static bool xyz_to_xy(float X, float Y, float Z, float &x, float &y);
    // Classify cached chromaticity against references_ and publish ColoreMatch.
    void publish_match(float x, float y);
};

} // namespace studica_control

#endif // COLORE_COMPONENT_H
