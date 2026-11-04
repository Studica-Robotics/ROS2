/*
 * navx3_component.cpp
 *
 * ros2 component for the studica navx3 imu.
 *   CAN: studica_driver::Navx3    (classic CAN, one frame per message group)
 *   USB: studica_driver::Navx3Usb ("DATA:" lines on /dev/ttyACM0)
 * see navx3_component.hpp for topics, service commands and params.
 */

#include "studica_control/navx3_component.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <thread>

namespace studica_control {

namespace {

std::string to_lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

} // namespace


std::vector<std::shared_ptr<rclcpp::Node>> Navx3::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx)
{
    std::vector<std::shared_ptr<rclcpp::Node>> nodes;

    control->declare_parameter<std::vector<std::string>>("navx3.sensors", std::vector<std::string>{});
    const std::vector<std::string> sensor_ids = control->get_parameter("navx3.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        const std::string p = "navx3." + sensor;
        Navx3Settings s;
        control->declare_parameter<std::string>(p + ".transport", "can");
        control->declare_parameter<int>(p + ".can_id", 0);
        control->declare_parameter<std::string>(p + ".serial_port", s.serial_port);
        control->declare_parameter<std::string>(p + ".topic", "");
        control->declare_parameter<std::string>(p + ".frame_id", s.frame_id);
        control->declare_parameter<int>(p + ".update_rate_hz", s.update_rate_hz);
        control->declare_parameter<int>(p + ".yaw_axes", s.yaw_axes);
        control->declare_parameter<double>(p + ".orientation_variance", 0.0);
        control->declare_parameter<double>(p + ".angular_velocity_variance", 0.0);
        control->declare_parameter<double>(p + ".linear_acceleration_variance", 0.0);

        const std::string transport = to_lower(control->get_parameter(p + ".transport").as_string());
        s.usb = transport == "usb" || transport == "serial";
        const int can_id = control->get_parameter(p + ".can_id").as_int();
        s.serial_port = control->get_parameter(p + ".serial_port").as_string();
        s.topic = control->get_parameter(p + ".topic").as_string();
        if (s.topic.empty()) s.topic = sensor + "/imu";
        s.frame_id = control->get_parameter(p + ".frame_id").as_string();
        s.update_rate_hz = control->get_parameter(p + ".update_rate_hz").as_int();
        s.yaw_axes = control->get_parameter(p + ".yaw_axes").as_int();
        s.orientation_variance = control->get_parameter(p + ".orientation_variance").as_double();
        s.angular_velocity_variance = control->get_parameter(p + ".angular_velocity_variance").as_double();
        s.linear_acceleration_variance = control->get_parameter(p + ".linear_acceleration_variance").as_double();

        // checked before the casts below, so a bad value can't alias a valid one
        if (!s.usb && (can_id < 0 || can_id > 63)) {
            RCLCPP_ERROR(control->get_logger(), "navx3.%s: can_id must be 0-63, got %d. navx3 not started",
                         sensor.c_str(), can_id);
            continue;
        }
        if (s.update_rate_hz < 1 || s.update_rate_hz > 200) {
            RCLCPP_ERROR(control->get_logger(), "navx3.%s: update_rate_hz must be 1-200, got %d. navx3 not started",
                         sensor.c_str(), s.update_rate_hz);
            continue;
        }
        if (s.yaw_axes != 6 && s.yaw_axes != 9) {
            RCLCPP_WARN(control->get_logger(), "navx3.%s: yaw_axes must be 6 or 9, got %d. using 6",
                        sensor.c_str(), s.yaw_axes);
            s.yaw_axes = 6;
        }
        s.can_id = static_cast<uint8_t>(can_id);

        nodes.push_back(std::make_shared<Navx3>(vmx, sensor, s));
    }
    return nodes;
}


Navx3::Navx3(const rclcpp::NodeOptions &options) : Node("navx3", options) {}


Navx3::Navx3(std::shared_ptr<VMXPi> vmx, const std::string &name, const Navx3Settings &settings)
    : Node(name), settings_(settings)
{
    if (settings_.usb) {
        navx_usb_ = std::make_shared<studica_driver::Navx3Usb>(settings_.serial_port);
        if (!navx_usb_->IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "NavX3 USB failed to open %s", settings_.serial_port.c_str());
            return;
        }
    } else {
        navx_can_ = std::make_shared<studica_driver::Navx3>(settings_.can_id, vmx);
        if (!navx_can_->IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "NavX3 driver failed to init for CAN ID %u", settings_.can_id);
            return;
        }
    }

    const std::string where = settings_.usb ? settings_.serial_port : "CAN ID " + std::to_string(settings_.can_id);

    // Over USB the NavX3 is silent until the driver wakes it; give it a moment to
    // start streaming so the settings below aren't sent before it listens.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    studica_driver::Navx3Data first;
    while (!read(first) && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (first.valid_mask == 0)
        RCLCPP_WARN(this->get_logger(), "no data from NavX3 on %s yet", where.c_str());

    // Settings only change the NavX3's RAM, so they're sent on every start.
    const uint16_t rate = static_cast<uint16_t>(settings_.update_rate_hz);
    int status = navx_usb_ ? navx_usb_->SetOutputRate(rate) : navx_can_->SetOutputRate(rate);
    if (status != NAVX3_OK)
        RCLCPP_WARN(this->get_logger(), "output rate %d Hz not applied: %s", settings_.update_rate_hz,
                    status_string(status).c_str());
    status = navx_usb_ ? navx_usb_->SetYawAxes(settings_.yaw_axes) : navx_can_->SetYawAxes(settings_.yaw_axes);
    if (status != NAVX3_OK)
        RCLCPP_WARN(this->get_logger(), "yaw axes %d not applied: %s", settings_.yaw_axes,
                    status_string(status).c_str());

    int major = 0, minor = 0, patch = 0;
    status = navx_usb_ ? navx_usb_->GetFirmwareVersion(major, minor, patch)
                       : navx_can_->GetFirmwareVersion(major, minor, patch);
    if (status == NAVX3_OK) {
        RCLCPP_INFO(this->get_logger(), "NavX3 ready on %s, firmware %d.%d.%d",
                    where.c_str(), major, minor, patch);
    } else {
        RCLCPP_WARN(this->get_logger(), "NavX3 started but did not report its firmware: %s",
                    status_string(status).c_str());
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(settings_.topic, 10);
    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/navx3_cmd",
        std::bind(&Navx3::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(1000000 / settings_.update_rate_hz),
        std::bind(&Navx3::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "NavX3 publishing %s at %d Hz, frame %s, %d-axis yaw",
                settings_.topic.c_str(), settings_.update_rate_hz, settings_.frame_id.c_str(),
                settings_.yaw_axes);
}


Navx3::~Navx3() {}


std::string Navx3::status_string(int status)
{
    switch (status) {
        case NAVX3_OK: return "ok";
        case NAVX3_TIMEOUT: return "no reply";
        case NAVX3_USB_NOT_OPEN: return "port not open";
        case NAVX3_USB_UNKNOWN_COMMAND: return "not supported by this firmware";
        default: return "rejected by sensor (status " + std::to_string(status) + ")";
    }
}


bool Navx3::read(studica_driver::Navx3Data &data)
{
    // Stale after three missed samples, and never under 100 ms.
    const auto max_age = std::chrono::milliseconds(std::max(100, 3000 / settings_.update_rate_hz));
    if (navx_usb_) return navx_usb_->Read(data, max_age);
    if (navx_can_) return navx_can_->Read(data, max_age);
    return false;
}


void Navx3::publish_data()
{
    studica_driver::Navx3Data d;
    if (!read(d)) return;

    const uint32_t quat_bit = settings_.yaw_axes == 9 ? NAVX3_MSG_QUAT9D : NAVX3_MSG_QUAT6D;
    if (!(d.valid_mask & quat_bit)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "NavX3 is sending data but no %d-axis quaternion", settings_.yaw_axes);
        return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = settings_.frame_id;

    // driver order is w, x, y, z
    const double *q = settings_.yaw_axes == 9 ? d.quat9 : d.quat6;
    msg.orientation.w = q[0];
    msg.orientation.x = q[1];
    msg.orientation.y = q[2];
    msg.orientation.z = q[3];

    msg.angular_velocity.x = d.angular_vel_rad_per_sec[0];
    msg.angular_velocity.y = d.angular_vel_rad_per_sec[1];
    msg.angular_velocity.z = d.angular_vel_rad_per_sec[2];

    msg.linear_acceleration.x = d.linear_accel_mps2[0];
    msg.linear_acceleration.y = d.linear_accel_mps2[1];
    msg.linear_acceleration.z = d.linear_accel_mps2[2];

    // -1 in element 0 means "not provided"; all zeros means "covariance unknown"
    for (int i = 0; i < 3; i++) {
        msg.orientation_covariance[i * 4] = settings_.orientation_variance;
        msg.angular_velocity_covariance[i * 4] = settings_.angular_velocity_variance;
        msg.linear_acceleration_covariance[i * 4] = settings_.linear_acceleration_variance;
    }
    if (!(d.valid_mask & NAVX3_MSG_ANGULAR_VELOCITY)) {
        msg.angular_velocity = geometry_msgs::msg::Vector3();
        msg.angular_velocity_covariance.fill(0.0);
        msg.angular_velocity_covariance[0] = -1.0;
    }
    if (!(d.valid_mask & NAVX3_MSG_LINEAR_ACCEL)) {
        msg.linear_acceleration = geometry_msgs::msg::Vector3();
        msg.linear_acceleration_covariance.fill(0.0);
        msg.linear_acceleration_covariance[0] = -1.0;
    }

    publisher_->publish(msg);
}


void Navx3::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    const std::string &params = request->params;
    if (!navx_can_ && !navx_usb_) {
        response->success = false;
        response->message = "driver not initialized";
        return;
    }

    if (params == "reset_yaw") {
        const int status = navx_usb_ ? navx_usb_->ResetYaw() : navx_can_->ResetYaw();
        response->success = status == NAVX3_OK;
        response->message = response->success ? "yaw reset" : status_string(status);

    } else if (params == "get_data") {
        studica_driver::Navx3Data d;
        if (!read(d)) {
            response->success = false;
            response->message = "no recent data";
            return;
        }
        char text[128];
        std::snprintf(text, sizeof(text), "yaw: %.2f, pitch: %.2f, roll: %.2f, temperature: %.1f C",
                      d.yaw_deg, d.pitch_deg, d.roll_deg, d.temperature_c);
        response->success = true;
        response->message = text;

    } else if (params == "get_info") {
        int major = 0, minor = 0, patch = 0;
        uint32_t uuid[3] = {0, 0, 0};
        int status = navx_usb_ ? navx_usb_->GetFirmwareVersion(major, minor, patch)
                               : navx_can_->GetFirmwareVersion(major, minor, patch);
        if (status == NAVX3_OK)
            status = navx_usb_ ? navx_usb_->GetSensorUuid(uuid) : navx_can_->GetSensorUuid(uuid);
        if (status != NAVX3_OK) {
            response->success = false;
            response->message = status_string(status);
            return;
        }
        char text[96];
        std::snprintf(text, sizeof(text), "firmware: %d.%d.%d, uuid: %08X-%08X-%08X",
                      major, minor, patch, uuid[0], uuid[1], uuid[2]);
        response->success = true;
        response->message = text;

    } else if (params == "self_test") {
        int32_t results[8] = {0};
        const int status = navx_usb_ ? navx_usb_->SelfTest(results) : navx_can_->SelfTest(results);
        if (status != NAVX3_OK) {
            response->success = false;
            response->message = status_string(status);
            return;
        }
        // gyro overall, x, y, z, then accelerometer overall, x, y, z
        std::string text = "gyro:";
        for (int i = 0; i < 4; i++) text += " " + std::to_string(results[i]);
        text += ", accel:";
        for (int i = 4; i < 8; i++) text += " " + std::to_string(results[i]);
        response->success = true;
        response->message = text;

    } else {
        response->success = false;
        response->message = "unknown command '" + params + "' — use reset_yaw, get_data, get_info or self_test";
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Navx3)
