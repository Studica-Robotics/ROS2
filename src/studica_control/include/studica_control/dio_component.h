#ifndef DIO_COMPONENT_H
#define DIO_COMPONENT_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "dio.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class DIO : public rclcpp::Node {
public:
    explicit DIO(const rclcpp::NodeOptions &options);
    DIO(std::shared_ptr<VMXPi> vmx, VMXChannelIndex pin, studica_driver::PinMode pin_mode, std::string type);
    ~DIO();

private:
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    void publish_dio_state();

    // void start_publishing();

    void DisplayVMXError(VMXErrorCode vmxerr);

    std::shared_ptr<VMXPi> vmx_;                                  // VMXPi instance
    std::shared_ptr<studica_driver::DIO> dio_;                    // DIO instance
    VMXChannelIndex pin_;                                         // Pin index for the DIO component
    studica_driver::PinMode pin_mode_;                            // Pin mode for the DIO component
    // int button_pin_;                                              // The pin designated as the button (-1 if not set)
    
    bool is_publishing;
    int btn_pin;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;  // Service for managing DIO state
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;          // Publisher for DIO state
    rclcpp::TimerBase::SharedPtr timer_;                                // Timer for periodic state publishing
};

} // namespace studica_control

#endif // DIO_COMPONENT_H
