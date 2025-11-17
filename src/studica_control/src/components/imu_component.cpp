#include "studica_control/imu_component.h"

namespace studica_control {

std::shared_ptr<rclcpp::Node> Imu::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("imu.name");
    control->declare_parameter<std::string>("imu.topic");
    std::string name = control->get_parameter("imu.name").as_string();
    std::string topic = control->get_parameter("imu.topic").as_string();
    
    auto imu = std::make_shared<Imu>(vmx, name, topic);
    return imu;
}

Imu::Imu(const rclcpp::NodeOptions &options) : Node("imu", options) {}

Imu::Imu(std::shared_ptr<VMXPi> vmx, const std::string &name, const std::string &topic) : rclcpp::Node(name), vmx_(vmx) {
    imu_ = std::make_shared<studica_driver::Imu>(vmx_);
    imu_->ZeroYaw();
    service_ = this->create_service<studica_control::srv::SetData>("get_imu_data",
        std::bind(&Imu::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Imu::publish_data, this));
    RCLCPP_INFO(this->get_logger(), "IMU component is ready.");
}

Imu::~Imu() {}

void Imu::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> /* request */,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (imu_) RCLCPP_INFO(this->get_logger(), "IMU is available. Type: %s", typeid(*imu_).name());
    else RCLCPP_WARN(this->get_logger(), "IMU is not available.");

    try {
        float pitch = imu_->GetPitch();
        float yaw = imu_->GetYaw();
        float roll = imu_->GetRoll();

        response->success = true;
        response->message = "Pitch: " + std::to_string(pitch) + ", "
                            + "Yaw: " + std::to_string(yaw) + ", "
                            + "Roll: " + std::to_string(roll) + ".";
        RCLCPP_INFO(this->get_logger(), "Pitch: %f, Yaw: %f, Roll: %f.", pitch, yaw, roll);
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "Failed to get IMU data: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to get IMU data: %s", e.what());
    }
}

void Imu::publish_data() {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "imu_link";

    msg.orientation.x = imu_->GetQuaternionX();
    msg.orientation.y = imu_->GetQuaternionY();
    msg.orientation.z = imu_->GetQuaternionZ();
    msg.orientation.w = imu_->GetQuaternionW();

    msg.angular_velocity.x = imu_->GetRawGyroX() * (M_PI / 180.0); // need to convert from DPS to RPS
    msg.angular_velocity.y = imu_->GetRawGyroY() * (M_PI / 180.0);
    msg.angular_velocity.z = imu_->GetRawGyroZ() * (M_PI / 180.0);

    msg.linear_acceleration.x = imu_->GetWorldLinearAccelX() * 9.80665; // need to convert g's to m/s^2
    msg.linear_acceleration.y = imu_->GetWorldLinearAccelY() * 9.80665;
    msg.linear_acceleration.z = imu_->GetWorldLinearAccelZ() * 9.80665;

    // Populate covariances (3x3 matrices flattened row-major) for EKF weighting (2D use)
    // orientation_covariance: roll, pitch large; yaw modest (~2 deg)^2
    msg.orientation_covariance = {
        1e6, 0.0, 0.0,
        0.0, 1e6, 0.0,
        0.0, 0.0, 0.0012
    };

    // angular_velocity_covariance: roll/pitch large; yaw rate modest (~1.8 deg/s)^2
    msg.angular_velocity_covariance = {
        1e6, 0.0, 0.0,
        0.0, 1e6, 0.0,
        0.0, 0.0, 0.001
    };

    // linear_acceleration_covariance: not used by our EKF; set reasonable diagonal
    msg.linear_acceleration_covariance = {
        0.5, 0.0, 0.0,
        0.0, 0.5, 0.0,
        0.0, 0.0, 0.5
    };

    publisher_->publish(msg);
}

void Imu::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *err_str = GetVMXErrorString(vmxerr);
    printf("VMX Error %d: %s\n", vmxerr, err_str);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Imu)
