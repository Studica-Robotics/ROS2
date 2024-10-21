#include "studica_control/imu_driver_node.h"

ImuDriver::ImuDriver(std::shared_ptr<VMXPi> vmx)
    : Device("imu_driver_node_"), vmx_(vmx) {
        
    is_publishing_ = false;
    // Create a publisher for IMU data
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_publisher", 10);

    // Declare and initialize parameters for selecting which fields to publish
    this->declare_parameter<bool>("publish_yaw", true);
    this->declare_parameter<bool>("publish_pitch", true);
    this->declare_parameter<bool>("publish_roll", true);

    bool realtime = true;
    uint8_t update_rate_hz = 50;
    vmx_ = vmx;

    // Check if VMXPi is open
    if (vmx_->IsOpen())
    {
        RCLCPP_INFO(this->get_logger(), "IMU Connected");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ImuDriver::Spin, this)); // Will check the state before publishing
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open VMX Client.");
    }
}

// Publish a simple message
void ImuDriver::publish_message()
{
    auto message = std_msgs::msg::String();
    message.data = std::string("Message from ") + this->get_name() + std::string(": ") + std::to_string(count_++);
    // std::cout << this->get_name() << ": " << count_ << std::endl;
    publisher_->publish(message);
}

// Publish IMU data
void ImuDriver::publish_imu_data()
{
    vmx::AHRS &ahrs_ref = vmx_->ahrs;
    auto imu_msg = sensor_msgs::msg::Imu();

    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    // Get the parameter values to determine which fields to publish
    bool publish_yaw = this->get_parameter("publish_yaw").as_bool();
    bool publish_pitch = this->get_parameter("publish_pitch").as_bool();
    bool publish_roll = this->get_parameter("publish_roll").as_bool();

    if (publish_yaw)
    {
        imu_msg.orientation.x = ahrs_ref.GetYaw();
    }
    if (publish_pitch)
    {
        imu_msg.orientation.y = ahrs_ref.GetPitch();
    }
    if (publish_roll)
    {
        imu_msg.orientation.z = ahrs_ref.GetRoll();
    }

    // Publish the IMU data
    imu_publisher_->publish(imu_msg);
    // std::cout << "Published IMU Data: " << imu_msg.orientation.x << ", " << imu_msg.orientation.y << ", " << imu_msg.orientation.z << std::endl;
}

// Main loop where publishing happens if enabled
void ImuDriver::Spin()
{
    if (is_publishing_)
    { // Check if publishing is enabled
        publish_imu_data();
        publish_message();
    }
}

// Start publishing
void ImuDriver::start_publishing()
{
    if (!is_publishing_)
    {
        is_publishing_ = true;
        RCLCPP_INFO(this->get_logger(), "Publishing started");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Already publishing");
    }
}

// Stop publishing
void ImuDriver::stop_publishing()
{
    if (is_publishing_)
    {
        is_publishing_ = false;
        RCLCPP_INFO(this->get_logger(), "Publishing stopped");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Already stopped");
    }
}

// Override the cmd function to handle start/stop commands
void ImuDriver::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    if (params == "start")
    {
        start_publishing();
        response->success = true;
        response->message = "Publishing started";
    }
    else if (params == "stop")
    {
        stop_publishing();
        response->success = true;
        response->message = "Publishing stopped";
    }
    else
    {
        response->success = false;
        response->message = "Unknown command";
    }
}
