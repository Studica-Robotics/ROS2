/*
 * colore_component.cpp
 * ROS2 component for the Studica Colore color sensor.
 *   CAN: studica_driver::Colore  (XYZ telemetry @ COLORE_CAN_TELEM_XYZ, scaled x10000)
 *   USB: studica_driver::ColoreUsb (text color lines on /dev/ttyACM0)
 */
#include "studica_control/colore_component.hpp"

#include <algorithm>
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
        control->declare_parameter<int>(p + ".publish_rate_hz", 10);
        control->declare_parameter<std::vector<std::string>>(p + ".publish_outputs",
                                                             std::vector<std::string>{"info"});
        control->declare_parameter<int>(p + ".brightness", 50);
        control->declare_parameter<std::string>(p + ".measmode", "off");
        control->declare_parameter<double>(p + ".measmode_z_mm", -1.0);
        control->declare_parameter<std::vector<std::string>>(p + ".match_references", std::vector<std::string>{});
        control->declare_parameter<double>(p + ".match_threshold", 0.05);

        const ColoreTransport transport = parse_transport(control->get_parameter(p + ".transport").as_string());
        const uint8_t can_id = static_cast<uint8_t>(control->get_parameter(p + ".can_id").as_int());
        const std::string serial_port = control->get_parameter(p + ".serial_port").as_string();
        const int rate = control->get_parameter(p + ".publish_rate_hz").as_int();
        const auto outputs = control->get_parameter(p + ".publish_outputs").as_string_array();
        const int brightness = control->get_parameter(p + ".brightness").as_int();
        const std::string measmode = control->get_parameter(p + ".measmode").as_string();
        const double measmode_z_mm = control->get_parameter(p + ".measmode_z_mm").as_double();
        const double match_threshold = control->get_parameter(p + ".match_threshold").as_double();

        // Reference colors for matching: each name in match_references needs a
        // matching match_<name>_xy: [x, y] entry (declared lazily here).
        // NB: copy the array into a named local first — iterating directly over
        // get_parameter(...).as_string_array() dangles, because as_string_array()
        // returns a reference into the temporary Parameter, which is destroyed
        // before the loop body runs (range-for only extends the final temporary).
        const std::vector<std::string> ref_names =
            control->get_parameter(p + ".match_references").as_string_array();
        std::vector<ColoreReference> references;
        for (const auto &rname : ref_names) {
            const std::string key = p + ".match_" + rname + "_xy";
            control->declare_parameter<std::vector<double>>(key, std::vector<double>{});
            const auto xy = control->get_parameter(key).as_double_array();
            if (xy.size() == 2)
                references.push_back({rname, static_cast<float>(xy[0]), static_cast<float>(xy[1])});
            else
                RCLCPP_WARN(control->get_logger(), "colore %s: %s must be [x, y]; skipping",
                            sensor.c_str(), key.c_str());
        }

        RCLCPP_INFO(control->get_logger(), "%s -> transport: %s, rate: %d Hz",
                    sensor.c_str(), transport == ColoreTransport::Usb ? "usb" : "can", rate);

        nodes.push_back(std::make_shared<Colore>(vmx, sensor, transport, can_id, serial_port,
                                                 rate, outputs, brightness,
                                                 measmode, static_cast<float>(measmode_z_mm),
                                                 references, static_cast<float>(match_threshold)));
    }
    return nodes;
}


Colore::Colore(const rclcpp::NodeOptions &options) : Node("colore", options) {}


Colore::Colore(std::shared_ptr<VMXPi> vmx, const std::string &name,
               ColoreTransport transport, uint8_t can_id, const std::string &serial_port,
               int publish_rate_hz,
               const std::vector<std::string> &publish_outputs,
               int brightness,
               const std::string &measmode, float measmode_z_mm,
               const std::vector<ColoreReference> &references, float match_threshold)
    : Node(name),
      transport_(transport),
      vmx_(vmx),
      publish_rate_hz_(publish_rate_hz > 0 ? publish_rate_hz : 10),
      outputs_(parse_publish_outputs(publish_outputs)),
      references_(references),
      match_threshold_(match_threshold > 0.0f ? match_threshold : 0.05f)
{
    // One knob: the publish rate also sets how often the sensor samples, so CAN/USB
    // traffic matches the publish rate. Clamp to the firmware's 20-2000 ms window.
    const int sample_ms = std::max(20, std::min(2000, 1000 / publish_rate_hz_));

    if (transport_ == ColoreTransport::Usb) {
        colore_usb_ = std::make_shared<studica_driver::ColoreUsb>(serial_port);
        if (!colore_usb_->IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Colore USB failed to open %s", serial_port.c_str());
            return;
        }
        // ROS always streams XYZ; the host derives sRGB/hex and colour-matching from it.
        // (Other USB clients can still request SRGB/HEX/RGB/RAW directly from the firmware.)
        colore_usb_->ConfigureStreaming("xyz", sample_ms);
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
        // Force the onboard status LED to render calibrated colour (CAN telemetry
        // is XYZ regardless; this only affects the sensor's own indicator LED).
        colore_can_->SetColorFormat(studica_driver::Colore::ColorFormat::XYZ);
        colore_can_->SetSampleTimeMs(static_cast<uint16_t>(sample_ms));
        if (brightness >= 0) colore_can_->SetBrightness(static_cast<uint8_t>(brightness));
    }

    // Measurement mode is a startup parameter (off|auto|fixed), not a runtime
    // command: the firmware-driven multi-flash mode is a streaming setting, so
    // it is applied once here like brightness and the sample rate.
    apply_measmode(measmode, measmode_z_mm);

    color_publisher_ = this->create_publisher<std_msgs::msg::ColorRGBA>(name + "/color", 10);
    if (outputs_.info)
        info_publisher_ = this->create_publisher<studica_control::msg::ColoreColorMsg>(name + "/color_info", 10);
    if (outputs_.match)
        match_publisher_ = this->create_publisher<studica_control::msg::ColoreMatch>(name + "/color_match", 10);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/colore_cmd",
        std::bind(&Colore::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    const int period_ms = std::max(1, 1000 / publish_rate_hz_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                     std::bind(&Colore::publish_color, this));

    RCLCPP_INFO(this->get_logger(), "Colore topics: /%s/color (+info=%d match=%d/%zu refs) at %d Hz",
                name.c_str(), outputs_.info, outputs_.match, references_.size(), publish_rate_hz_);
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
        else if (k == "match" || k == "color_match") o.match = true;
    }
    return o;
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


bool Colore::xyz_to_xy(float X, float Y, float Z, float &x, float &y) {
    const float sum = X + Y + Z;
    if (sum <= 1e-6f) return false;   // no light -> chromaticity undefined
    x = X / sum;
    y = Y / sum;
    return true;
}


void Colore::publish_match(float x, float y) {
    studica_control::msg::ColoreMatch m;
    m.x = x;
    m.y = y;

    float best = 1e9f;
    const ColoreReference *hit = nullptr;
    for (const auto &ref : references_) {
        const float d = std::hypot(x - ref.x, y - ref.y);
        if (d < best) { best = d; hit = &ref; }
    }
    if (hit && best <= match_threshold_) {
        m.label = hit->label;
        m.confidence = std::max(0.0f, 1.0f - best / match_threshold_);
    } else {
        m.label = "unknown";
        m.confidence = 0.0f;
    }
    match_publisher_->publish(m);
}


void Colore::publish_color() {
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0f;
    studica_control::msg::ColoreColorMsg info;
    bool have_color = false, have_xyz = false;

    // Both transports carry CIE XYZ as the source of truth; the host derives
    // /color (normalized sRGB) and the debug r/g/b/hex from it identically.
    auto fill_from_xyz = [&](float X, float Y, float Z) {
        info.x = X; info.y = Y; info.z = Z; have_xyz = true;
        xyz_to_srgb(X, Y, Z, color.r, color.g, color.b);
        info.r = static_cast<uint8_t>(color.r * 255.0f);
        info.g = static_cast<uint8_t>(color.g * 255.0f);
        info.b = static_cast<uint8_t>(color.b * 255.0f);
        info.hex = hex_from_rgb(info.r, info.g, info.b);
        have_color = true;
    };

    if (transport_ == ColoreTransport::Usb) {
        if (!colore_usb_) return;
        studica_driver::ColoreUsb::Sample s;
        if (!colore_usb_->ReadLatest(&s)) return;
        info.seq = static_cast<uint16_t>(s.seq);
        if (!s.has_xyz) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "colore: samples arriving (seq=%u) but no XYZ line — not publishing", s.seq);
            return;   // XYZ is always streamed; nothing to publish without it yet
        }
        fill_from_xyz(s.x, s.y, s.z);
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
        fill_from_xyz(i16(buf[2], buf[3]) / scale,
                      i16(buf[4], buf[5]) / scale,
                      i16(buf[6], buf[7]) / scale);
    }

    if (have_color) color_publisher_->publish(color);
    if (outputs_.info && info_publisher_) info_publisher_->publish(info);
    // Chromaticity matching needs XYZ. Cache the latest xy so learn_color can snapshot it.
    if (have_xyz && xyz_to_xy(info.x, info.y, info.z, last_x_, last_y_)) {
        have_last_xy_ = true;
        if (outputs_.match && match_publisher_) publish_match(last_x_, last_y_);
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
            if (!colore_usb_->RequestConfig(&line)) return err("no GETCONFIG response");
            // Flatten multi-line GETCONFIG so ros2 CLI doesn't show \n
            for (std::size_t i = 0; i < line.size();) {
                if (line[i] == '\n') {
                    line.replace(i, 1, ", ");
                    i += 2;
                } else {
                    ++i;
                }
            }
            return ok(line);
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
    if (params.rfind("learn_color", 0) == 0) {
        // learn_color,<name> — snapshot the current chromaticity as reference <name>.
        const auto comma = params.find(',');
        if (comma == std::string::npos || comma + 1 >= params.size())
            return err("usage: learn_color,<name>");
        if (!have_last_xy_)
            return err("no chromaticity yet — no color data received from the sensor");
        const std::string label = params.substr(comma + 1);
        const std::string xy = "xy=(" + std::to_string(last_x_) + ", " + std::to_string(last_y_) + ")";
        for (auto &ref : references_) {
            if (ref.label == label) {
                ref.x = last_x_; ref.y = last_y_;
                return ok("updated " + label + " " + xy);
            }
        }
        references_.push_back({label, last_x_, last_y_});
        return ok("learned " + label + " " + xy);
    }
    err("unknown command '" + params + "' — use get_config, set_brightness, learn_color,<name> "
        "(measmode is a startup parameter, not a command)");
}


void Colore::apply_measmode(const std::string &mode, float z_mm) {
    const std::string m = to_lower(mode);

    if (m != "off" && m != "auto" && m != "fixed") {
        RCLCPP_WARN(this->get_logger(), "colore: unknown measmode '%s' — using off "
                    "(valid: off, auto, fixed)", mode.c_str());
    }

    if (transport_ == ColoreTransport::Usb) {
        if (!colore_usb_) return;
        if (m == "auto")
            colore_usb_->SendCommand("MEASMODE,-1,0");
        else if (m == "fixed")
            colore_usb_->SendCommand("MEASMODE," + std::to_string(z_mm) + ",0");
        else
            colore_usb_->SendCommand("MEASMODE,OFF");
    } else {
        if (!colore_can_) return;
        if (m == "auto")        colore_can_->SetMeasureModeAuto();
        else if (m == "fixed")  colore_can_->SetMeasureModeFixed(z_mm);
        else                    colore_can_->SetMeasureModeOff();
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Colore)
