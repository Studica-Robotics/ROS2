#include <memory>

#include "studica_control/encoder_component.h"
#include "studica_control/servo_component.h"
#include "studica_control/imu_component.h"
#include "studica_control/titan_component.h"
#include "studica_control/dio_component.h"
#include "studica_control/mecanum_drive_component.h"
#include "rclcpp/rclcpp.hpp"
#include "VMXPi.h"
#include <studica_control/msg/initialize_params.hpp>

class ControlServer : public rclcpp::Node {
private:
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    // map of names to nodes
    std::map<std::string, std::shared_ptr<rclcpp::Node>> component_map;

    void service_callback(
        const std::shared_ptr<studica_control::srv::SetData::Request> request,
        std::shared_ptr<studica_control::srv::SetData::Response> response) 
    {
        std::string name = request->name;
        std::string component = request->component;

        if (component == "servo") {
            // Extract initialization parameters
            uint8_t pin = request->initparams.pin;
            std::string servo_type_str = request->initparams.servo_type;
            std::string name = request->name;

            // Determine the servo type
            studica_driver::ServoType servo_type;
            int min_angle = 0, max_angle = 0;
            
            if (servo_type_str == "standard") {
                servo_type = studica_driver::ServoType::Standard;
                min_angle = -150;
                max_angle = 150;
            } else if (servo_type_str == "continuous") {
                servo_type = studica_driver::ServoType::Continuous;
                min_angle = -100;
                max_angle = 100;
            } else if (servo_type_str == "linear") {
                servo_type = studica_driver::ServoType::Linear;
                min_angle = 0;
                max_angle = 100;
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid servo type. Allowed values are 'standard', 'continuous', or 'linear'.");
                response->success = false;
                response->message = "Invalid servo type.";
                return;
            }

            try {
                auto servo_node = std::make_shared<studica_control::Servo>(vmx_, name, pin, servo_type, min_angle, max_angle);
            
                // auto node_options = rclcpp::NodeOptions();
                // auto servo_node = std::make_shared<studica_control::Servo>(node_options);

                // Add to executor and component map
                executor_->add_node(servo_node);
                component_map[name] = servo_node;

                response->success = true;
                response->message = "Servo component '" + name + "' created successfully.";
                RCLCPP_INFO(this->get_logger(), "Created servo component '%s' of type '%s' on pin %d.",
                            name.c_str(), servo_type_str.c_str(), pin);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create servo component: %s", e.what());
                response->success = false;
                response->message = "Failed to create servo component: " + std::string(e.what());
            }
        } else if (component == "imu") {
            try {
                auto imu_node = std::make_shared<studica_control::Imu>(vmx_);
                executor_->add_node(imu_node);
                component_map[name] = imu_node;

                response->success = true;
                response->message = "IMU component '" + name + "' created successfully.";
                RCLCPP_INFO(this->get_logger(), "Created IMU component '%s'.", name.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create IMU component: %s", e.what());
                response->success = false;
                response->message = "Failed to create IMU component: " + std::string(e.what());
            }
        } else if (component == "titan") {
            try {
                auto ip = request->initparams;
                auto titan_node = std::make_shared<studica_control::Titan>(
                    vmx_, "Titan", 
                    ip.can_id, 
                    ip.motor_freq, 
                    ip.ticks_per_rotation, 
                    ip.wheel_radius, 
                    ip.wheel_separation);
                executor_->add_node(titan_node);
                component_map[name] = titan_node;

                response->success = true;
                response->message = "Titan '" + name + "' created successfully.";
                RCLCPP_INFO(this->get_logger(), "Created Titan component '%s'.", name.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create servo component: %s", e.what());
                response->success = false;
                response->message = "Failed to create servo component: " + std::string(e.what());
            }
        } else if (component == "dio") {
            try {
                auto ip = request->initparams;
                auto dio_node = std::make_shared<studica_control::DIO>(vmx_, ip.pin, studica_driver::PinMode::OUTPUT, std::string(request->params));
                executor_->add_node(dio_node);
                component_map[name] = dio_node;

                response->success = true;
                response->message = "DIO component '" + name + "' created successfully.";
                RCLCPP_INFO(this->get_logger(), "Created DIO component '%s'.", name.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create DIO component: %s", e.what());
                response->success = false;
                response->message = "Failed to create DIO component: " + std::string(e.what());
            }
        } else if (component == "mecanum") {
            try {
                auto ip = request->initparams;
                auto titan_node = std::make_shared<studica_control::MecanumDrive>(
                    vmx_, "MecanumDrive", 
                    ip.can_id, 
                    ip.motor_freq, 
                    ip.ticks_per_rotation, 
                    ip.wheel_radius, 
                    ip.wheelbase, 
                    ip.width);
                executor_->add_node(titan_node);
                component_map[name] = titan_node;

                response->success = true;
                response->message = "Mecanum Drive Controller '" + name + "' created successfully.";
                RCLCPP_INFO(this->get_logger(), "Created MecanumDrive component '%s'.", name.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create mecanum drive component: %s", e.what());
                response->success = false;
                response->message = "Failed to create mecanum drive component: " + std::string(e.what());
        }
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid component type '%s'.", component.c_str());
            response->success = false;
            response->message = "Invalid component type. Allowed type is 'servo'.";
        }
    }


public:
    std::shared_ptr<VMXPi> vmx_;
    ControlServer() : Node("control_server") {
        vmx_ = std::make_shared<VMXPi>(true, 50);

        service_ = this->create_service<studica_control::srv::SetData>(
            "create_component", 
            std::bind(&ControlServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    void set_executor(const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>& exec) {
        executor_ = exec;
    }
};


int main(int argc, char *argv[]) {

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