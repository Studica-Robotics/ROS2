#include "studica_control/imu_driver_node.h"

// public class Device : public rclcpp::Node {
//     public:
//         Device(std::string name) : Node(name) {
//             publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//             timer_ = this->create_wall_timer(
//                 std::chrono::seconds(1), 
//                 std::bind(&Device::publish_message, this));
//             count_ = 0;
//         }

//         void publish_message() {
//             auto message = std_msgs::msg::String();
//             message.data = "Hello, world! " + std::to_string(count_++);
//             publisher_->publish(message);
//         }

//         void cmd();

//     private:
//         rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//         rclcpp::TimerBase::SharedPtr timer_;
//         int count_;
// }

ImuDriver::ImuDriver() : Node("imu_driver_node_") {
    // Create a publisher for IMU data
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_publisher" , 10);

    // Declare and initialize parameters for selecting which fields to publish
    this->declare_parameter<bool>("publish_yaw", true);
    this->declare_parameter<bool>("publish_pitch", true);
    this->declare_parameter<bool>("publish_roll", true);

    bool realtime = true;
    uint8_t update_rate_hz = 50;
    vmx = std::make_shared<VMXPi>(realtime, update_rate_hz);

    // Check if VMXPi is open
    if (vmx->IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "IMU Connected");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&ImuDriver::Spin, this));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open VMX Client.");
    }
}
void ImuDriver::publish_message()
{
    auto message = std_msgs::msg::String();
    message.data = std::string("Message from ") + this->get_name() + std::string(": ") + std::to_string(count_++);
    std::cout << this->get_name() << ": " << count_ << std::endl;
    // message.data = ("Message from %s: %s", this->get_name(), std::to_string(count_++));
    publisher_->publish(message);
}
    
void ImuDriver::publish_imu_data() {
    vmx::AHRS& ahrs_ref = vmx->ahrs;
    auto imu_msg = sensor_msgs::msg::Imu();

    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    // Get the parameter values to determine which fields to publish
    bool publish_yaw = this->get_parameter("publish_yaw").as_bool();
    bool publish_pitch = this->get_parameter("publish_pitch").as_bool();
    bool publish_roll = this->get_parameter("publish_roll").as_bool();

  
        imu_msg.orientation.x = ahrs_ref.GetYaw();
   
        imu_msg.orientation.y = ahrs_ref.GetPitch();
    
        imu_msg.orientation.z = ahrs_ref.GetRoll();

    // Publish the IMU data
    imu_publisher_->publish(imu_msg);
    std::cout << "Published IMU Data: " << imu_msg.orientation.x << ", " << imu_msg.orientation.y << ", " << imu_msg.orientation.z << std::endl;
}

int ImuDriver::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    return 0;
}

void ImuDriver::Spin() { 
    publish_imu_data(); 
    publish_message();
}

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto imu_node = std::make_shared<ImuDriver>();
//     rclcpp::spin(imu_node);
//     rclcpp::shutdown();
//     return 0;
// }
