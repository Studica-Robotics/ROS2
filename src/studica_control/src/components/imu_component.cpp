/*
 * imu_component.cpp
 *
 * ros2 component for the onboard imu (inertial measurement unit).
 * reads orientation, rotation rate, and linear acceleration from the
 * navx sensor built into the vmx-pi board.
 *
 * topic:   <topic> (sensor_msgs/Imu) — published at 20hz
 *            orientation as a quaternion (x, y, z, w)
 *            angular velocity in radians per second (x, y, z)
 *            linear acceleration in meters per second squared (x, y, z)
 *
 * service: get_imu_data (studica_control/SetData)
 *   returns the current pitch, yaw, and roll as a string
 *   (no specific params field needed — any call returns the values)
 */

#include "studica_control/imu_component.h"

namespace studica_control {


// reads imu parameters from params.yaml and creates the imu node
std::shared_ptr<rclcpp::Node> Imu::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("imu.name");
    control->declare_parameter<std::string>("imu.topic");
    control->declare_parameter<std::string>("imu.frame_id", "imu_link");

    std::string name     = control->get_parameter("imu.name").as_string();
    std::string topic    = control->get_parameter("imu.topic").as_string();
    std::string frame_id = control->get_parameter("imu.frame_id").as_string();

    return std::make_shared<Imu>(vmx, name, topic, frame_id);
}


// composable node constructor — used when loading as a plugin
Imu::Imu(const rclcpp::NodeOptions &options) : Node("imu", options) {}


// main constructor — connects to the imu and sets up the publisher,
// service, and periodic timer. zeros the yaw on startup.
Imu::Imu(std::shared_ptr<VMXPi> vmx, const std::string &name, const std::string &topic, const std::string &frame_id)
    : rclcpp::Node(name), vmx_(vmx), frame_id_(frame_id) {

    imu_ = std::make_shared<studica_driver::Imu>(vmx_);
    imu_->ZeroYaw();

    // service for reading pitch, yaw, and roll on demand
    service_ = this->create_service<studica_control::srv::SetData>(
        "get_imu_data",
        std::bind(&Imu::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes full imu data at 20hz
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Imu::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "imu component ready.");
}

Imu::~Imu() {}


// returns pitch, yaw, and roll as a human-readable string
void Imu::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> /* request */,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        float pitch = imu_->GetPitch();
        float yaw   = imu_->GetYaw();
        float roll  = imu_->GetRoll();

        response->success = true;
        response->message = "pitch: " + std::to_string(pitch)
                          + ", yaw: "  + std::to_string(yaw)
                          + ", roll: " + std::to_string(roll);

        RCLCPP_INFO(this->get_logger(), "pitch: %f, yaw: %f, roll: %f", pitch, yaw, roll);

    } catch (const std::exception &e) {
        response->success = false;
        response->message = "failed to get imu data: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "failed to get imu data: %s", e.what());
    }
}


// reads orientation, angular velocity, and linear acceleration from the
// imu and publishes them as a standard ros2 imu message.
// units: orientation as quaternion, angular velocity in rad/s, acceleration in m/s²
// covariances are set to -1 (unknown) — set your own values if needed
void Imu::publish_data() {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = frame_id_;

    msg.orientation.x = imu_->GetQuaternionX();
    msg.orientation.y = imu_->GetQuaternionY();
    msg.orientation.z = imu_->GetQuaternionZ();
    msg.orientation.w = imu_->GetQuaternionW();

    // raw gyro values come in degrees per second — convert to radians per second
    msg.angular_velocity.x = imu_->GetRawGyroX() * (M_PI / 180.0);
    msg.angular_velocity.y = imu_->GetRawGyroY() * (M_PI / 180.0);
    msg.angular_velocity.z = imu_->GetRawGyroZ() * (M_PI / 180.0);

    // acceleration values come in g's — convert to meters per second squared
    msg.linear_acceleration.x = imu_->GetWorldLinearAccelX() * 9.80665;
    msg.linear_acceleration.y = imu_->GetWorldLinearAccelY() * 9.80665;
    msg.linear_acceleration.z = imu_->GetWorldLinearAccelZ() * 9.80665;

    // -1 in the first covariance element means "unknown" per the ros2 imu spec
    msg.orientation_covariance[0]         = -1.0;
    msg.angular_velocity_covariance[0]    = -1.0;
    msg.linear_acceleration_covariance[0] = -1.0;

    publisher_->publish(msg);
}


void Imu::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("vmx error %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Imu)
