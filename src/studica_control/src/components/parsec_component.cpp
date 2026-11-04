/*
 * parsec_component.cpp
 *
 * ROS2 component for the Studica Parsec multi-zone ToF sensor.
 * CAN: studica_driver::Parsec::ReadDataStreamCAN2()
 * USB: studica_driver::ParsecUsb serial binary frames on /dev/ttyACM0
 * Current firmware only.
 */

#include "studica_control/parsec_component.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace studica_control {

namespace {

std::string to_lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

} // namespace


std::vector<std::shared_ptr<rclcpp::Node>> Parsec::initialize(
    rclcpp::Node *control, std::shared_ptr<VMXPi> vmx)
{
    std::vector<std::shared_ptr<rclcpp::Node>> parsec_nodes;

    control->declare_parameter<std::vector<std::string>>("parsec.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("parsec.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        const std::string prefix = "parsec." + sensor;
        const std::string transport_param = prefix + ".transport";
        const std::string can_id_param = prefix + ".can_id";
        const std::string serial_port_param = prefix + ".serial_port";
        const std::string frame_id_param = prefix + ".frame_id";
        const std::string publish_rate_param = prefix + ".publish_rate_hz";
        const std::string publish_outputs_param = prefix + ".publish_outputs";
        const std::string resolution_param = prefix + ".resolution";
        const std::string zone_mask_param = prefix + ".zone_mask";

        control->declare_parameter<std::string>(transport_param, "can");
        control->declare_parameter<int>(can_id_param, 0);
        control->declare_parameter<std::string>(serial_port_param, "/dev/ttyACM0");
        control->declare_parameter<std::string>(frame_id_param, sensor);
        control->declare_parameter<int>(publish_rate_param, 15);
        control->declare_parameter<std::vector<std::string>>(
            publish_outputs_param, std::vector<std::string>{"grid"});
        control->declare_parameter<int>(resolution_param, 0);
        control->declare_parameter<std::string>(zone_mask_param, "");

        const ParsecTransport transport =
            parse_transport(control->get_parameter(transport_param).as_string());
        const uint8_t can_id = static_cast<uint8_t>(control->get_parameter(can_id_param).as_int());
        const std::string serial_port = control->get_parameter(serial_port_param).as_string();
        const std::string frame_id = control->get_parameter(frame_id_param).as_string();
        const int publish_rate_hz = control->get_parameter(publish_rate_param).as_int();
        const std::vector<std::string> publish_outputs =
            control->get_parameter(publish_outputs_param).as_string_array();
        const int resolution = control->get_parameter(resolution_param).as_int();
        const std::string zone_mask = control->get_parameter(zone_mask_param).as_string();

        RCLCPP_INFO(control->get_logger(),
                    "%s -> transport: %s, frame_id: %s, publish_rate_hz: %d, publish_outputs: [%s]",
                    sensor.c_str(),
                    transport == ParsecTransport::Usb ? "usb" : "can",
                    frame_id.c_str(), publish_rate_hz,
                    [&publish_outputs]() {
                        std::ostringstream oss;
                        for (size_t i = 0; i < publish_outputs.size(); ++i) {
                            if (i > 0) {
                                oss << ", ";
                            }
                            oss << publish_outputs[i];
                        }
                        return oss.str();
                    }().c_str());
        if (transport == ParsecTransport::Can) {
            RCLCPP_INFO(control->get_logger(), "%s -> can_id: %u", sensor.c_str(), can_id);
        } else {
            RCLCPP_INFO(control->get_logger(), "%s -> serial_port: %s", sensor.c_str(), serial_port.c_str());
        }

        parsec_nodes.push_back(std::make_shared<Parsec>(
            vmx, sensor, transport, can_id, serial_port, frame_id, publish_rate_hz, publish_outputs,
            resolution, zone_mask));
    }

    return parsec_nodes;
}


Parsec::Parsec(const rclcpp::NodeOptions &options) : Node("parsec", options) {}


Parsec::Parsec(std::shared_ptr<VMXPi> vmx, const std::string &name,
               ParsecTransport transport, uint8_t can_id, const std::string &serial_port,
               const std::string &frame_id, int publish_rate_hz,
               const std::vector<std::string> &publish_outputs,
               int resolution, const std::string &zone_mask)
    : Node(name),
      transport_(transport),
      vmx_(vmx),
      frame_id_(frame_id),
      publish_rate_hz_(publish_rate_hz >= 1 && publish_rate_hz <= 60 ? publish_rate_hz : 15),
      outputs_(parse_publish_outputs(publish_outputs))
{
    if (publish_rate_hz_ != publish_rate_hz) {
        RCLCPP_WARN(this->get_logger(), "publish_rate_hz %d out of range 1-60, using 15", publish_rate_hz);
    }

    if (transport_ == ParsecTransport::Usb) {
        parsec_usb_ = std::make_shared<studica_driver::ParsecUsb>(serial_port);
        if (!parsec_usb_->IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Parsec USB failed to open %s", serial_port.c_str());
            return;
        }
        if (!parsec_usb_->ConfigureStreaming()) {
            RCLCPP_WARN(this->get_logger(), "Parsec USB streaming config failed on %s", serial_port.c_str());
        }
    } else {
        parsec_can_ = std::make_shared<studica_driver::Parsec>(can_id, vmx_);
        if (!parsec_can_->IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Parsec driver failed to init for CAN ID %u", can_id);
            return;
        }
    }

    // Resolution first: changing it restarts ranging at 15 Hz, which resets the rate.
    if (resolution != 0) {
        const std::string error = set_resolution(resolution);
        if (!error.empty()) {
            RCLCPP_WARN(this->get_logger(), "resolution %d not applied: %s", resolution, error.c_str());
        }
    }
    const std::string rate_error = set_output_rate(publish_rate_hz_);
    if (!rate_error.empty()) {
        RCLCPP_WARN(this->get_logger(), "output rate %d Hz not applied: %s", publish_rate_hz_, rate_error.c_str());
    }
    if (!zone_mask.empty()) {
        uint64_t mask = 0;
        const std::string error = parse_zone_mask(zone_mask, mask) ? set_zone_mask(mask) : "not a hex mask";
        if (!error.empty()) {
            RCLCPP_WARN(this->get_logger(), "zone_mask %s not applied: %s", zone_mask.c_str(), error.c_str());
        }
    }

    if (outputs_.zones) {
        zones_publisher_ =
            this->create_publisher<studica_control::msg::ParsecZoneMsg>(name + "/zones", 10);
    }
    if (outputs_.grid) {
        grid_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(name + "/grid", 10);
    }
    if (outputs_.pointcloud) {
        pointcloud_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(name + "/pointcloud", 10);
    }
    min_range_publisher_ = this->create_publisher<sensor_msgs::msg::Range>(name + "/min_range", 10);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/parsec_cmd",
        std::bind(&Parsec::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    const int period_ms = std::max(1, 1000 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&Parsec::publish_zones, this));

    if (transport_ == ParsecTransport::Can) {
        studica_driver::Parsec::Config config;
        if (parsec_can_->GetConfig(config)) {
            RCLCPP_INFO(this->get_logger(), "Parsec ready on CAN ID %u, firmware %u.%u.%u",
                        can_id, config.firmware[0], config.firmware[1], config.firmware[2]);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Parsec started on CAN ID %u but GETCONFIG response not received yet", can_id);
        }
    } else if (parsec_usb_->RequestConfig(&usb_config_line_)) {
        RCLCPP_INFO(this->get_logger(), "Parsec USB ready on %s: %s",
                    serial_port.c_str(), usb_config_line_.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Parsec USB opened %s but GETCONFIG not received yet",
                    serial_port.c_str());
    }

    std::ostringstream topics;
    if (outputs_.zones) {
        topics << "/" << name << "/zones ";
    }
    if (outputs_.grid) {
        topics << "/" << name << "/grid ";
    }
    if (outputs_.pointcloud) {
        topics << "/" << name << "/pointcloud ";
    }
    topics << "/" << name << "/min_range";

    RCLCPP_INFO(this->get_logger(), "Parsec topics:%s at %d Hz", topics.str().c_str(), publish_rate_hz_);
}


Parsec::~Parsec() {}


ParsecTransport Parsec::parse_transport(const std::string &value)
{
    const std::string key = to_lower(value);
    if (key == "usb" || key == "serial") {
        return ParsecTransport::Usb;
    }
    return ParsecTransport::Can;
}


ParsecPublishOutputs Parsec::parse_publish_outputs(const std::vector<std::string> &names)
{
    ParsecPublishOutputs outputs;
    for (const auto &name : names) {
        const std::string key = to_lower(name);
        if (key == "zones") {
            outputs.zones = true;
        } else if (key == "grid") {
            outputs.grid = true;
        } else if (key == "pointcloud" || key == "points" || key == "cloud") {
            outputs.pointcloud = true;
        }
    }

    if (!outputs.zones && !outputs.grid && !outputs.pointcloud) {
        outputs.grid = true;
    }
    return outputs;
}


void Parsec::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                          std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    cmd(request->params, *request, response);
}


void Parsec::cmd(const std::string &params,
                 const studica_control::srv::SetData::Request &request,
                 std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    if (params == "get_config") {
        if (transport_ == ParsecTransport::Usb) {
            if (!parsec_usb_) {
                response->success = false;
                response->message = "USB driver not initialized";
                return;
            }
            std::string line;
            if (!parsec_usb_->RequestConfig(&line)) {
                response->success = false;
                response->message = "no GETCONFIG response";
                return;
            }
            usb_config_line_ = line;
            response->success = true;
            response->message = line;
            return;
        }

        if (!parsec_can_) {
            response->success = false;
            response->message = "CAN driver not initialized";
            return;
        }
        studica_driver::Parsec::Config config;
        if (!parsec_can_->GetConfig(config)) {
            response->success = false;
            response->message = "no GETCONFIG response";
            return;
        }
        response->success = true;
        response->message = "firmware=" + std::to_string(config.firmware[0]) + "." +
                            std::to_string(config.firmware[1]) + "." +
                            std::to_string(config.firmware[2]) +
                            ", version=" + std::to_string(config.version) +
                            ", fdist_en=" + std::to_string(config.distance_enabled) +
                            ", can_id=" + std::to_string(config.can_id);
        return;
    }

    // Settings: "" back means it worked.
    auto reply = [&](const std::string &error, const std::string &done) {
        response->success = error.empty();
        response->message = error.empty() ? done : error;
    };
    const int value = request.initparams.int_value;
    if (params == "set_resolution") {
        reply(set_resolution(value), "resolution set to " + std::to_string(value) + " zones");
        return;
    }
    if (params == "set_output_rate") {
        reply(set_output_rate(value), "output rate set to " + std::to_string(value) + " Hz");
        return;
    }
    if (params == "set_frequency") {
        reply(set_frequency(value), "ranging frequency set to " + std::to_string(value) + " Hz");
        return;
    }
    if (params.rfind("set_zone_mask", 0) == 0) {
        const auto comma = params.find(',');
        uint64_t mask = 0;
        if (comma == std::string::npos || !parse_zone_mask(params.substr(comma + 1), mask)) {
            reply("usage: set_zone_mask,<hex mask>, e.g. set_zone_mask,FFFF", "");
            return;
        }
        reply(set_zone_mask(mask), "zone mask set to " + params.substr(comma + 1));
        return;
    }

    if (params == "get_zone_distance") {
        const int zone = request.initparams.n_encoder;
        if (zone < 0 || zone >= static_cast<int>(last_zones_)) {
            response->success = false;
            response->message = "invalid zone " + std::to_string(zone);
            return;
        }
        response->success = true;
        response->message = std::to_string(last_fdist_[static_cast<size_t>(zone)]);
        return;
    }

    if (params == "get_min_distance") {
        const int16_t min_mm = find_min_valid_distance(last_fdist_.data(), last_zones_);
        if (min_mm < 0) {
            response->success = false;
            response->message = "no valid zones";
            return;
        }
        response->success = true;
        response->message = std::to_string(min_mm);
        return;
    }

    response->success = false;
    response->message = "unknown command '" + params +
                        "' — use get_config, get_zone_distance, get_min_distance, set_resolution, "
                        "set_output_rate, set_frequency or set_zone_mask,<hex>";
}


std::string Parsec::can_result(int ack_status)
{
    switch (ack_status) {
        case PARSEC_ACK_OK: return "";
        case PARSEC_ACK_BAD_ARGUMENT: return "rejected by sensor";
        case PARSEC_ACK_TIMEOUT: return "no reply";
        default: return "sensor error " + std::to_string(ack_status);
    }
}


std::string Parsec::usb_result(studica_driver::ParsecUsb::Result result)
{
    if (result == studica_driver::ParsecUsb::Result::Ok) {
        return "";
    }
    return studica_driver::ParsecUsb::ResultString(result);
}


std::string Parsec::set_resolution(int zones)
{
    if (zones != 16 && zones != 64) {
        return "resolution must be 16 or 64";
    }
    if (parsec_can_) {
        return can_result(parsec_can_->SetResolution(static_cast<uint8_t>(zones)));
    }
    if (parsec_usb_) {
        return usb_result(parsec_usb_->SetResolution(zones));
    }
    return "driver not initialized";
}


std::string Parsec::set_output_rate(int hz)
{
    if (hz < 1 || hz > 60) {
        return "output rate must be 1-60 Hz";
    }
    if (parsec_can_) {
        return can_result(parsec_can_->SetOutputRate(static_cast<uint8_t>(hz)));
    }
    if (parsec_usb_) {
        return usb_result(parsec_usb_->SetOutputRate(hz));
    }
    return "driver not initialized";
}


std::string Parsec::set_frequency(int hz)
{
    if (hz < 1 || hz > 60) {
        return "ranging frequency must be 1-60 Hz";
    }
    if (parsec_can_) {
        return can_result(parsec_can_->SetRangingFrequency(static_cast<uint8_t>(hz)));
    }
    if (parsec_usb_) {
        return usb_result(parsec_usb_->SetRangingFrequency(hz));
    }
    return "driver not initialized";
}


std::string Parsec::set_zone_mask(uint64_t mask)
{
    if (parsec_can_) {
        return can_result(parsec_can_->SetZoneMask(mask));
    }
    if (parsec_usb_) {
        return usb_result(parsec_usb_->SetZoneMask(mask));
    }
    return "driver not initialized";
}


bool Parsec::parse_zone_mask(const std::string &text, uint64_t &mask)
{
    std::string hex = text;
    if (hex.rfind("0x", 0) == 0 || hex.rfind("0X", 0) == 0) {
        hex = hex.substr(2);
    }
    if (hex.empty() || hex.size() > 16 ||
        hex.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) {
        return false;
    }
    mask = std::stoull(hex, nullptr, 16);
    return mask != 0;
}


bool Parsec::is_valid_distance_mm(int16_t d)
{
    return d >= 1 && d <= 4000;
}


int16_t Parsec::find_min_valid_distance(const int16_t *fdist, int count)
{
    int16_t min_d = std::numeric_limits<int16_t>::max();
    bool found = false;
    for (int i = 0; i < count; ++i) {
        if (is_valid_distance_mm(fdist[i]) && fdist[i] < min_d) {
            min_d = fdist[i];
            found = true;
        }
    }
    return found ? min_d : static_cast<int16_t>(-1);
}


int Parsec::resolution_zone_count(uint8_t zones_reported)
{
    return zones_reported > 16 ? 64 : 16;
}


void Parsec::grid_size_from_zones(int resolution_zones, uint32_t &width, uint32_t &height)
{
    if (resolution_zones == 64) {
        width = 8;
        height = 8;
    } else {
        width = 4;
        height = 4;
    }
}


sensor_msgs::msg::Image Parsec::build_grid_image(
    const rclcpp::Time &stamp, const std::string &frame_id,
    const int16_t *fdist, int resolution_zones)
{
    uint32_t width = 0;
    uint32_t height = 0;
    grid_size_from_zones(resolution_zones, width, height);

    const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<uint8_t> data(pixel_count * 2, 0);

    for (size_t i = 0; i < pixel_count; ++i) {
        uint16_t px = 0;
        if (i < static_cast<size_t>(resolution_zones) && fdist[i] >= 0) {
            px = static_cast<uint16_t>(fdist[i]);
        }
        data[2 * i] = static_cast<uint8_t>(px & 0xFF);
        data[2 * i + 1] = static_cast<uint8_t>((px >> 8) & 0xFF);
    }

    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id;
    image.height = height;
    image.width = width;
    image.encoding = "16UC1";
    image.is_bigendian = false;
    image.step = width * 2;
    image.data = std::move(data);
    return image;
}


sensor_msgs::msg::PointCloud2 Parsec::build_point_cloud(
    const rclcpp::Time &stamp, const std::string &frame_id,
    const int16_t *fdist, int resolution_zones)
{
    uint32_t width = 0;
    uint32_t height = 0;
    grid_size_from_zones(resolution_zones, width, height);

    int valid_points = 0;
    for (int i = 0; i < resolution_zones; ++i) {
        if (fdist[i] >= 0) {
            ++valid_points;
        }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(static_cast<size_t>(valid_points));

    constexpr double field_of_view = 0.785; // ~45 deg
    const double tan_half_fov = std::tan(field_of_view * 0.5);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (int i = 0; i < resolution_zones; ++i) {
        if (fdist[i] < 0) {
            continue;
        }

        const uint32_t row = static_cast<uint32_t>(i) / width;
        const uint32_t col = static_cast<uint32_t>(i) % width;
        const double u = (static_cast<double>(col) + 0.5) / static_cast<double>(width) - 0.5;
        const double v = (static_cast<double>(row) + 0.5) / static_cast<double>(height) - 0.5;
        const double depth_m = static_cast<double>(fdist[i]) / 1000.0;

        *iter_x = static_cast<float>(depth_m * u * 2.0 * tan_half_fov);
        *iter_y = static_cast<float>(depth_m * v * 2.0 * tan_half_fov);
        *iter_z = static_cast<float>(depth_m);
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    return cloud;
}


bool Parsec::read_fdist(uint8_t *seq, uint8_t *zones, int16_t *fdist, int max_zones, int *filled_out)
{
    if (filled_out == nullptr) {
        return false;
    }
    *filled_out = 0;

    if (transport_ == ParsecTransport::Usb) {
        if (!parsec_usb_) {
            return false;
        }
        *filled_out = parsec_usb_->ReadLatestFdist(seq, zones, fdist, max_zones);
        return *filled_out > 0;
    }

    if (!parsec_can_) {
        return false;
    }
    // Non-blocking: 0 when no new frame finished since the last tick.
    *filled_out = parsec_can_->ReadDataStreamCAN2(seq, zones, fdist, max_zones);
    return *filled_out > 0;
}


void Parsec::publish_zones()
{
    uint8_t seq = 0;
    uint8_t zones = 0;
    int16_t fdist[64] = {0};

    int n = 0;
    if (!read_fdist(&seq, &zones, fdist, 64, &n) || n <= 0) {
        return;
    }

    last_zones_ = static_cast<uint8_t>(n);
    for (int i = 0; i < n; ++i) {
        last_fdist_[static_cast<size_t>(i)] = fdist[i];
    }

    const int resolution_zones = resolution_zone_count(zones);
    const auto stamp = this->get_clock()->now();

    if (outputs_.zones && zones_publisher_) {
        studica_control::msg::ParsecZoneMsg zones_msg;
        zones_msg.header.stamp = stamp;
        zones_msg.header.frame_id = frame_id_;
        zones_msg.seq = seq;
        zones_msg.zones = static_cast<uint8_t>(resolution_zones);
        zones_msg.fdist.assign(fdist, fdist + resolution_zones);
        zones_publisher_->publish(zones_msg);
    }
    if (outputs_.grid && grid_publisher_) {
        grid_publisher_->publish(build_grid_image(stamp, frame_id_, fdist, resolution_zones));
    }
    if (outputs_.pointcloud && pointcloud_publisher_) {
        pointcloud_publisher_->publish(build_point_cloud(stamp, frame_id_, fdist, resolution_zones));
    }

    constexpr double min_range_m = 0.001;  // 1 mm
    constexpr double max_range_m = 4.0;
    constexpr double field_of_view = 0.785; // ~45 deg

    const int16_t min_mm = find_min_valid_distance(fdist, n);

    sensor_msgs::msg::Range range_msg;
    range_msg.header.stamp = stamp;
    range_msg.header.frame_id = frame_id_;
    range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range_msg.field_of_view = field_of_view;
    range_msg.min_range = min_range_m;
    range_msg.max_range = max_range_m;
    range_msg.range = (min_mm >= 0) ? static_cast<double>(min_mm) / 1000.0 : std::numeric_limits<float>::infinity();
    min_range_publisher_->publish(range_msg);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Parsec)
