#ifndef ULTRASONIC_DRIVER_HPP_
#define ULTRASONIC_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "VMXPi.h"
#include <studica_control/srv/set_data.hpp>
#include "studica_control/device.h"
#include <std_msgs/msg/float32.hpp>

class UltrasonicDriver : public Device {
public:
    UltrasonicDriver(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex ping, VMXChannelIndex echo);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

private:
    void read_distance();
    void Spin();
    void start_reading();
    void stop_reading();
    void DisplayVMXError(VMXErrorCode vmxerr);

    std::shared_ptr<VMXPi> vmx_;
    bool is_reading_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_publisher_; // Publisher for distance dataVMXIO* vmx_;

    VMXChannelIndex ping_channel_index;
    VMXChannelIndex echo_channel_index;
    VMXResourceHandle ping_output_res_handle;
    VMXResourceHandle echo_inputcap_res_handle;
};

#endif // ULTRASONIC_DRIVER_HPP_
