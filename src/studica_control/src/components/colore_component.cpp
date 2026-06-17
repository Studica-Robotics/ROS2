/*
 * colore_component.cpp
 * ROS2 component for the Studica Colore color sensor.
 *   CAN: studica_driver::Colore  (XYZ telemetry @ COLORE_CAN_TELEM_XYZ, scaled x10000)
 *   USB: studica_driver::ColoreUsb (text color lines on /dev/ttyACM0)
 */
#include "studica_control/colore_component.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace studica_control {

namespace {
std::string to_lower(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v;
}
std::string hex_from_rgb(uint8_t r, uint8_t g, uint8_t b) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "#%02X%02X%02X", r, g, b);
    return std::string(buf);
}
}  // namespace


std::vector<std::shared_ptr<rclcpp::Node>> Colore::initialize(
    rclcpp::Node *control, std::shared_ptr<VMXPi> vmx)
{
    std::vector<std::shared_ptr<rclcpp::Node>> nodes;

    control->declare_parameter<std::vector<std::string>>("colore.sensors", std::vector<std::string>{});
    std::vector<std::string> ids = control->get_parameter("colore.sensors").as_string_array();

    for (const auto &sensor : ids) {
        const std::string p = "colore." + sensor;
        control->declare_parameter<std::string>(p + ".transport", "can");
        control->declare_parameter<int>(p + ".can_id", 0);
        control->declare_parameter<std::string>(p + ".serial_port", "/dev/ttyACM0");
        control->declare_parameter<std::string>(p + ".frame_id", sensor);
        control->declare_parameter<int>(p + ".publish_rate_hz", 10);
        control->declare_parameter<std::vector<std::string>>(p + ".publish_outputs",
                                                             std::vector<std::string>{"info"});
        control->declare_parameter<std::string>(p + ".color_format", "srgb");
        control->declare_parameter<int>(p + ".brightness", 50);
        control->declare_parameter<int>(p + ".sample_time_ms", 200);

        const ColoreTransport transport = parse_transport(control->get_parameter(p + ".transport").as_string());
        const uint8_t can_id = static_cast<uint8_t>(control->get_parameter(p + ".can_id").as_int());
        const std::string serial_port = control->get_parameter(p + ".serial_port").as_string();
        const std::string frame_id = control->get_parameter(p + ".frame_id").as_string();
        const int rate = control->get_parameter(p + ".publish_rate_hz").as_int();
        const auto outputs = control->get_parameter(p + ".publish_outputs").as_string_array();
        const std::string fmt = control->get_parameter(p + ".color_format").as_string();
        const int brightness = control->get_parameter(p + ".brightness").as_int();
        const int sample_ms = control->get_parameter(p + ".sample_time_ms").as_int();

        RCLCPP_INFO(control->get_logger(), "%s -> transport: %s, frame_id: %s, format: %s, rate: %d Hz",
                    sensor.c_str(), transport == ColoreTransport::Usb ? "usb" : "can",
                    frame_id.c_str(), fmt.c_str(), rate);

        nodes.push_back(std::make_shared<Colore>(vmx, sensor, transport, can_id, serial_port,
                                                 frame_id, rate, outputs, fmt, brightness, sample_ms));
    }
    return nodes;
}


Colore::Colore(const rclcpp::NodeOptions &options) : Node("colore", options) {}


Colore::Colore(std::shared_ptr<VMXPi> vmx, const std::string &name,
               ColoreTransport transport, uint8_t can_id, const std::string &serial_port,
               const std::string &frame_id, int publish_rate_hz,
               const std::vector<std::string> &publish_outputs,
               const std::string &color_format, int brightness, int sample_time_ms)
    : Node(name),
      transport_(transport),
      vmx_(vmx),
      frame_id_(frame_id),
      color_format_(to_lower(color_format)),
      publish_rate_hz_(publish_rate_hz > 0 ? publish_rate_hz : 10),
      outputs_(parse_publish_outputs(publish_outputs))
{
    if (transport_ == ColoreTransport::Usb) {
        colore_usb_ = std::make_shared<studica_driver::ColoreUsb>(serial_port);
        if (!colore_usb_->IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Colore USB failed to open %s", serial_port.c_str());
            return;
        }
        colore_usb_->ConfigureStreaming(color_format_, sample_time_ms);
        if (brightness >= 0) colore_usb_->SendCommand("BRIGHTNESS," + std::to_string(brightness));
        std::string cfg;
        if (colore_usb_->RequestConfig(&cfg))
            RCLCPP_INFO(this->get_logger(), "Colore USB ready on %s", serial_port.c_str());
    } else {
        colore_can_ = std::make_shared<studica_driver::Colore>(can_id, vmx_);
        if (colore_can_->GetCanID() != can_id) {
            RCLCPP_ERROR(this->get_logger(), "Colore CAN driver failed to init for ID %u", can_id);
            return;
        }
        colore_can_->SetColorFormat(parse_color_format(color_format_));
        if (sample_time_ms > 0) colore_can_->SetSampleTimeMs(static_cast<uint16_t>(sample_time_ms));
        if (brightness >= 0)    colore_can_->SetBrightness(static_cast<uint8_t>(brightness));
        if (outputs_.raw)
            RCLCPP_WARN(this->get_logger(), "%s: raw_spectrum not available over CAN (USB only)", name.c_str());
    }

    color_publisher_ = this->create_publisher<std_msgs::msg::ColorRGBA>(name + "/color", 10);
    if (outputs_.info)
        info_publisher_ = this->create_publisher<studica_control::msg::ColoreColorMsg>(name + "/color_info", 10);
    if (outputs_.xyz)
        xyz_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(name + "/xyz", 10);
    if (outputs_.raw)
        raw_publisher_ = this->create_publisher<studica_control::msg::ColoreRawMsg>(name + "/raw_spectrum", 10);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/colore_cmd",
        std::bind(&Colore::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    const int period_ms = std::max(1, 1000 / publish_rate_hz_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                     std::bind(&Colore::publish_color, this));

    RCLCPP_INFO(this->get_logger(), "Colore topics: /%s/color (+info=%d xyz=%d raw=%d) at %d Hz",
                name.c_str(), outputs_.info, outputs_.xyz, outputs_.raw, publish_rate_hz_);
}


Colore::~Colore() {}


ColoreTransport Colore::parse_transport(const std::string &v) {
    const std::string k = to_lower(v);
    return (k == "usb" || k == "serial") ? ColoreTransport::Usb : ColoreTransport::Can;
}


ColorePublishOutputs Colore::parse_publish_outputs(const std::vector<std::string> &names) {
    ColorePublishOutputs o;
    for (const auto &n : names) {
        const std::string k = to_lower(n);
        if (k == "info" || k == "color_info") o.info = true;
        else if (k == "xyz") o.xyz = true;
        else if (k == "raw" || k == "raw_spectrum") o.raw = true;
    }
    return o;
}


studica_driver::Colore::ColorFormat Colore::parse_color_format(const std::string &v) {
    using CF = studica_driver::Colore::ColorFormat;
    const std::string k = to_lower(v);
    if (k == "rgb") return CF::RGB;
    if (k == "raw") return CF::RAW;
    if (k == "hex") return CF::HEX;
    if (k == "xyz") return CF::XYZ;
    return CF::SRGB;
}


void Colore::xyz_to_srgb(float X, float Y, float Z, float &r, float &g, float &b) {
    float rl =  3.2406f * X - 1.5372f * Y - 0.4986f * Z;
    float gl = -0.9689f * X + 1.8758f * Y + 0.0415f * Z;
    float bl =  0.0557f * X - 0.2040f * Y + 1.0570f * Z;
    auto enc = [](float c) {
        c = std::min(1.0f, std::max(0.0f, c));
        return (c <= 0.0031308f) ? 12.92f * c : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
    };
    r = enc(rl); g = enc(gl); b = enc(bl);
}


void Colore::publish_color() {
    const auto stamp = this->get_clock()->now();

    std_msgs::msg::ColorRGBA color;
    color.a = 1.0f;
    studica_control::msg::ColoreColorMsg info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.format = color_format_;
    info.dist_mm = -1.f;
    bool have_color = false, have_xyz = false;

    if (transport_ == ColoreTransport::Usb) {
        if (!colore_usb_) return;
        studica_driver::ColoreUsb::Sample s;
        if (!colore_usb_->ReadLatest(&s)) return;
        info.seq = static_cast<uint16_t>(s.seq);

        if (s.has_srgb) {
            color.r = s.r / 255.0f; color.g = s.g / 255.0f; color.b = s.b / 255.0f;
            info.r = s.r; info.g = s.g; info.b = s.b; info.hex = hex_from_rgb(s.r, s.g, s.b);
            have_color = true;
        }
        if (s.has_hex) info.hex = s.hex;
        if (s.has_xyz) {
            info.x = s.x; info.y = s.y; info.z = s.z; have_xyz = true;
            if (!have_color) { xyz_to_srgb(s.x, s.y, s.z, color.r, color.g, color.b); have_color = true; }
        }
        if (s.has_tristim) { info.fz = s.fz; info.fy = s.fy; info.fxl = s.fxl; }
        if (s.has_dist) info.dist_mm = s.dist_mm;

        if (outputs_.raw && raw_publisher_ && s.has_raw) {
            studica_control::msg::ColoreRawMsg raw;
            raw.header.stamp = stamp; raw.header.frame_id = frame_id_;
            raw.seq = static_cast<uint16_t>(s.seq);
            raw.channels.assign(s.raw.begin(), s.raw.begin() + s.raw_n);
            raw_publisher_->publish(raw);
        }
    } else {
        if (!colore_can_) return;
        uint8_t buf[8] = {0};
        int n = colore_can_->Read(COLORE_CAN_TELEM_XYZ, buf, sizeof(buf));
        if (n < 8) return;
        const uint16_t seq = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
        auto i16 = [](uint8_t lo, uint8_t hi) {
            return static_cast<int16_t>(static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
        };
        const float scale = 10000.0f;
        info.seq = seq;
        info.x = i16(buf[2], buf[3]) / scale;
        info.y = i16(buf[4], buf[5]) / scale;
        info.z = i16(buf[6], buf[7]) / scale;
        have_xyz = true;
        xyz_to_srgb(info.x, info.y, info.z, color.r, color.g, color.b);
        info.r = static_cast<uint8_t>(color.r * 255.0f);
        info.g = static_cast<uint8_t>(color.g * 255.0f);
        info.b = static_cast<uint8_t>(color.b * 255.0f);
        info.hex = hex_from_rgb(info.r, info.g, info.b);
        have_color = true;
    }

    if (have_color) color_publisher_->publish(color);
    if (outputs_.info && info_publisher_) info_publisher_->publish(info);
    if (outputs_.xyz && xyz_publisher_ && have_xyz) {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.stamp = stamp; v.header.frame_id = frame_id_;
        v.vector.x = info.x; v.vector.y = info.y; v.vector.z = info.z;
        xyz_publisher_->publish(v);
    }
}


void Colore::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                          std::shared_ptr<studica_control::srv::SetData::Response> response) {
    cmd(request->params, *request, response);
}


void Colore::cmd(const std::string &params,
                 const studica_control::srv::SetData::Request &request,
                 std::shared_ptr<studica_control::srv::SetData::Response> response) {
    auto ok = [&](const std::string &m) { response->success = true;  response->message = m; };
    auto err = [&](const std::string &m) { response->success = false; response->message = m; };

    if (params == "get_config") {
        if (transport_ == ColoreTransport::Usb && colore_usb_) {
            std::string line;
            if (colore_usb_->RequestConfig(&line)) return ok(line);
            return err("no GETCONFIG response");
        }
        if (colore_can_) {
            uint32_t v = 0;
            if (colore_can_->GetConfig(/*item=*/1, v)) return ok("colorformat=" + std::to_string(v));
            return err("GETCONFIG (CAN) failed");
        }
        return err("driver not initialized");
    }
    if (params == "set_brightness") {
        const int b = request.initparams.n_encoder;   // reuse an int field for the value
        if (transport_ == ColoreTransport::Usb && colore_usb_)
            return colore_usb_->SendCommand("BRIGHTNESS," + std::to_string(b)) ? ok("brightness set") : err("send failed");
        if (colore_can_)
            return colore_can_->SetBrightness(static_cast<uint8_t>(b)) ? ok("brightness set") : err("ack failed");
        return err("driver not initialized");
    }
    if (params == "set_measmode_auto") {
        if (colore_can_) return colore_can_->SetMeasureModeAuto(false) ? ok("measmode auto") : err("ack failed");
        if (colore_usb_) return colore_usb_->SendCommand("MEASMODE,-1,0") ? ok("measmode auto") : err("send failed");
        return err("driver not initialized");
    }
    if (params == "set_measmode_off") {
        if (colore_can_) return colore_can_->SetMeasureModeOff() ? ok("measmode off") : err("ack failed");
        if (colore_usb_) return colore_usb_->SendCommand("MEASMODE,OFF") ? ok("measmode off") : err("send failed");
        return err("driver not initialized");
    }
    err("unknown command '" + params + "' — use get_config, set_brightness, set_measmode_auto, set_measmode_off");
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Colore)
