#include <memory>

#include "studica_control/encoder_component.h"
#include "studica_control/servo_component.h"
#include "rclcpp/rclcpp.hpp"
#include "VMXPi.h"
#include <studica_control/msg/initialize_params.hpp>

class ControlServer : public rclcpp::Node {
private:
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    // map of names to nodes
    std::map<std::string, std::shared_ptr<rclcpp::Node>> component_map;

    void service_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                          std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string name = request->name.c_str();
        std::string component = request->component.c_str();
        if (component == "servo") {
            uint8_t pin = request->initparams.pin;
            string servo_type_str = request->initparams.servo_type;
            rclcpp::NodeOptions options;
            auto servo_node = std::make_shared<studica_control::Servo>(options);
            if (servo_type_str == "standard") {
                servo_node->initialize(pin, studica_driver::ServoType::Standard, -150, 150);
            } else if (servo_type_str == "continuous") {
                servo_node->initialize(pin, studica_driver::ServoType::Continuous, -100, 100);
            } else if (servo_type_str == "linear") {
                servo_node->initialize(pin, studica_driver::ServoType::Linear, 0, 100);
            } else {
                RCLCPP_INFO(this->get_logger(), "Invalid servo type. Allowed values are 'standard' or 'continuous'.");
                return;
            }
            executor_->add_node(std::dynamic_pointer_cast<rclcpp::Node>(servo_node));
            component_map[name] = servo_node;
        }
        if (component == "get_node") {
            std::dynamic_pointer_cast<studica_control::Servo>(component_map[name])->cmd(request->params.c_str(), response);
        }
    }

public:
    ControlServer() : Node("control_server") {
        service_ = this->create_service<studica_control::srv::SetData>(
            "control_server", 
            std::bind(&ControlServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    void set_executor(const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>& exec) {
        executor_ = exec;
    }


};

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto control_server = std::make_shared<ControlServer>();
    control_server->set_executor(executor);
    executor->add_node(control_server);

    // auto servo = std::make_shared<studica_control::Servo>(13, studica_driver::ServoType::Standard);
    // auto servo2 = std::make_shared<studica_control::Servo>(14, studica_driver::ServoType::Continuous);
    
    executor->spin();

    rclcpp::shutdown();

    return 0;
}