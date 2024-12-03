#include "studica_control/imu_component.h"

namespace studica_control
{

Imu::Imu(const rclcpp::NodeOptions &options) : Node("imu_", options) {
    // imu_ = std::make_shared<studica_driver::Imu>(std::make_shared<VMXPi>(true, 50));
    // Create a service to handle commands
    service_ = this->create_service<studica_control::srv::SetData>(
        "get_imu_data",
        std::bind(&Imu::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "IMU component is ready. hehe");
}

Imu::Imu(std::shared_ptr<VMXPi> vmx) : rclcpp::Node("imu_component"), vmx_(vmx)
{

    // Create a service to handle commands
    service_ = this->create_service<studica_control::srv::SetData>(
        "get_imu_data",
        std::bind(&Imu::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the imu object
    imu_ = std::make_shared<studica_driver::Imu>(vmx_);

    RCLCPP_INFO(this->get_logger(), "IMU component is ready.");
}
Imu::~Imu() {}

void Imu::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                       std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    if (imu_) {
        RCLCPP_INFO(this->get_logger(), "IMU is available. Type: %s", typeid(*imu_).name());
    } else {
        RCLCPP_WARN(this->get_logger(), "IMU is not available.");
    }

    try
    {
        // Retrieve pitch, yaw, and roll from the IMU
        float pitch = imu_->GetPitch();
        float yaw = imu_->GetYaw();
        float roll = imu_->GetRoll();

        response->success = true;
        response->message = "Pitch: " + std::to_string(pitch) + ", "
                            + "Yaw: " + std::to_string(yaw) + ", "
                            + "Roll: " + std::to_string(roll) + ".";
        RCLCPP_INFO(this->get_logger(), "Pitch: %f, Yaw: %f, Roll: %f.", pitch, yaw, roll);
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->message = "Failed to set IMU angle: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to set IMU angle: %s", e.what());
    }
}

void Imu::DisplayVMXError(VMXErrorCode vmxerr)
{
    const char *err_str = GetVMXErrorString(vmxerr);
    printf("VMX Error %d: %s\n", vmxerr, err_str);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Imu)