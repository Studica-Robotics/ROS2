/*
 * parsec_component.hpp
 *
 * ROS2 component for the Studica Parsec multi-zone ToF sensor (CAN or USB).
 *
 * topics (publish, optional via publish_outputs):
 *   /<name>/zones      (studica_control/ParsecZoneMsg) - int16[] fdist in mm (readable in topic echo)
 *   /<name>/grid       (sensor_msgs/Image, 16UC1) - 4x4 or 8x8 grid; mm per pixel as raw bytes (use RViz/cv_bridge)
 *   /<name>/pointcloud (sensor_msgs/PointCloud2) - one point per zone in frame_id
 *   /<name>/min_range  (sensor_msgs/Range) - nearest valid zone distance in metres
 *
 * service: /<name>/parsec_cmd (studica_control/SetData)
 *   get_config          - firmware version and CAN ID (CAN) or GETCONFIG line (USB)
 *   get_zone_distance   - distance for zone index in initparams.n_encoder (mm)
 *   get_min_distance    - nearest valid zone distance (mm)
 *
 * params (per sensor in params.yaml):
 *   transport     - can (default) or usb
 *   can_id        - CAN only; device CAN ID 0..63
 *   serial_port   - USB only; e.g. /dev/ttyACM0
 *   publish_outputs - zones, grid, pointcloud (any combination)
 */

#ifndef PARSEC_COMPONENT_H
#define PARSEC_COMPONENT_H

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "parsec.hpp"
#include "parsec_usb.hpp"
#include "studica_control/msg/parsec_zone_msg.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

enum class ParsecTransport {
    Can,
    Usb,
};

struct ParsecPublishOutputs {
    bool zones{false};
    bool grid{false};
    bool pointcloud{false};
};

class Parsec : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(
        rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);

    explicit Parsec(const rclcpp::NodeOptions &options);

    Parsec(std::shared_ptr<VMXPi> vmx, const std::string &name,
           ParsecTransport transport, uint8_t can_id, const std::string &serial_port,
           const std::string &frame_id, int publish_rate_hz,
           const std::vector<std::string> &publish_outputs);

    ~Parsec();

private:
    ParsecTransport transport_{ParsecTransport::Can};
    std::shared_ptr<studica_driver::Parsec> parsec_can_;
    std::shared_ptr<studica_driver::ParsecUsb> parsec_usb_;
    std::shared_ptr<VMXPi> vmx_;
    std::string frame_id_;
    std::string usb_config_line_;
    int publish_rate_hz_;
    ParsecPublishOutputs outputs_;

    std::array<int16_t, 64> last_fdist_{};
    uint8_t last_zones_{0};

    rclcpp::Publisher<studica_control::msg::ParsecZoneMsg>::SharedPtr zones_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr grid_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr min_range_publisher_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    void cmd(const std::string &params,
             const studica_control::srv::SetData::Request &request,
             std::shared_ptr<studica_control::srv::SetData::Response> response);

    void publish_zones();
    bool read_fdist(uint8_t *seq, uint8_t *zones, int16_t *fdist, int max_zones, int *filled_out);

    static ParsecTransport parse_transport(const std::string &value);
    static ParsecPublishOutputs parse_publish_outputs(const std::vector<std::string> &names);
    static bool is_valid_distance_mm(int16_t d);
    static int16_t find_min_valid_distance(const int16_t *fdist, int count);
    static int resolution_zone_count(uint8_t zones_reported);
    static void grid_size_from_zones(int resolution_zones, uint32_t &width, uint32_t &height);
    static sensor_msgs::msg::Image build_grid_image(
        const rclcpp::Time &stamp, const std::string &frame_id,
        const int16_t *fdist, int resolution_zones);
    static sensor_msgs::msg::PointCloud2 build_point_cloud(
        const rclcpp::Time &stamp, const std::string &frame_id,
        const int16_t *fdist, int resolution_zones);
};

} // namespace studica_control

#endif // PARSEC_COMPONENT_H
