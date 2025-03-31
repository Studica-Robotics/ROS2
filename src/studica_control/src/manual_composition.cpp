#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "studica_control/cobra_component.h"
#include "studica_control/dc_encoder_component.h"
#include "studica_control/diff_drive_component.h"
#include "studica_control/dio_component.h"
#include "studica_control/encoder_component.h"
#include "studica_control/imu_component.h"
#include "studica_control/mecanum_drive_component.h"
#include "studica_control/servo_component.h"
#include "studica_control/sharp_component.h"
#include "studica_control/titan_component.h"
#include "studica_control/ultrasonic_component.h"
#include "studica_control/msg/initialize_params.hpp"
#include "VMXPi.h"

class ControlServer : public rclcpp::Node {
private:
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::map<std::string, std::shared_ptr<rclcpp::Node>> component_map;

    std::string check_unique_name(const std::string &requested_name) {
        std::string unique_name = requested_name;
        int suffix = 1;

        while (component_map.find(unique_name) != component_map.end()) {
            unique_name = requested_name + "_" + std::to_string(suffix++);
        }

        if (unique_name != requested_name) {
            RCLCPP_INFO(this->get_logger(), "Renamed component from '%s' to '%s' to ensure unique naming.",
                requested_name.c_str(), unique_name.c_str());
        }

        return unique_name;
    }

    template<typename T, typename... Args>
    bool create_component(const std::string &requested_name,
                          const std::string &component_type,
                          std::shared_ptr<studica_control::srv::SetData::Response> response,
                          Args&&... args) {
        try {
            std::string unique_name = check_unique_name(requested_name);
            auto component = std::make_shared<T>(std::forward<Args>(args)...);
            executor_->add_node(component);
            component_map[unique_name] = component;

            response->success = true;
            response->message = component_type + " component '" + unique_name + "' created successfully!";

            RCLCPP_INFO(this->get_logger(), "Created %s component %s", component_type.c_str(), unique_name.c_str());

            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create %s component %s", component_type.c_str(), e.what());
            response->success = false;
            response->message = "Failed to create " + component_type + " component: " + std::string(e.what());

            return false;
        }
    }

    void service_callback(
        const std::shared_ptr<studica_control::srv::SetData::Request> request,
        std::shared_ptr<studica_control::srv::SetData::Response> response) {
        
        std::string name = request->name;
        std::string component = request->component;
        auto ip = request->initparams;
        
        if (component == "cobra") {
            create_component<studica_control::Cobra>(name, "Cobra", response, vmx_, "Cobra", ip.vref);
        } else if (component == "dutycycle") {
                create_component<studica_control::DutyCycleEncoder>(name, "duty_cycle_encoder", response, vmx_, "duty_cycle_encoder", ip.port);
        } else if (component == "diffdrive") {
            std::string unique_name = check_unique_name("diff_odom");
            auto component = std::make_shared<studica_control::DiffOdometry>();
            executor_->add_node(component);
            component_map[unique_name] = component;
            create_component<studica_control::DiffDrive>(
                name, "Differential Drive", response, vmx_, component, "DiffDrive", ip.can_id, ip.motor_freq, 
                ip.ticks_per_rotation, ip.wheel_radius, ip.wheel_separation, ip.left, ip.right, 
                ip.invert_left, ip.invert_right);
        } else if (component == "dio") {
            std::string dio_type_str = ip.dio_type;
            RCLCPP_INFO(this->get_logger(), "%s", std::string(request->params).c_str());
            if (dio_type_str == "input") {
                create_component<studica_control::DIO>(
                    name, "DIO", response, vmx_, ip.pin, studica_driver::PinMode::INPUT, std::string(request->params));
            } else if (dio_type_str == "output") {
                create_component<studica_control::DIO>(
                    name, "DIO", response, vmx_, ip.pin, studica_driver::PinMode::OUTPUT, std::string(request->params));
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid dio type. Allowed values are 'input' or 'output'.");
                response->success = false;
                response->message = "Invalid dio type.";
                return;
            }
        } else if (component == "encoder") {
            create_component<studica_control::Encoder>(name, "Encoder", response, vmx_, "Encoder", ip.port_a, ip.port_b);
        } else if (component == "imu") {
            create_component<studica_control::Imu>(name, "IMU", response, vmx_);
        } else if (component == "mecanum") {
            std::string unique_name = check_unique_name("mecanum_odom");
            auto component = std::make_shared<studica_control::MecanumOdometry>();
            executor_->add_node(component);
            component_map[unique_name] = component;
            create_component<studica_control::MecanumDrive>(
                name, "Mecanum Drive", response, vmx_, component, "MecanumDrive", ip.can_id, ip.motor_freq,
                ip.ticks_per_rotation, ip.wheel_radius, ip.wheelbase, ip.width, ip.front_left,
                ip.front_right, ip.rear_left, ip.rear_right, ip.invert_front_left, ip.invert_front_right,
                ip.invert_rear_left, ip.invert_rear_right);
        } else if (component == "servo") {
            uint8_t pin = ip.pin;
            std::string servo_type_str = ip.servo_type;
            
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
            create_component<studica_control::Servo>(name, "Servo", response, vmx_, name, pin, servo_type, min_angle, max_angle);
        } else if (component == "sharp") {
            create_component<studica_control::Sharp>(name, "Sharp", response, "sharp", ip.port, vmx_);
        } else if (component == "titan") {
            create_component<studica_control::Titan>(
                name, "Titan", response, vmx_, "Titan", ip.can_id, ip.motor_freq,
                ip.ticks_per_rotation, ip.wheel_radius);
        } else if (component == "ultrasonic") {
            create_component<studica_control::Ultrasonic>(
                name, "Ultrasonic", response, vmx_, "ultrasonic", ip.ping, ip.echo);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid component type '%s'.", component.c_str());
            response->success = false;
            response->message = "Invalid component type.";
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

    void set_executor(const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> &exec) {
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

    executor->spin();

    rclcpp::shutdown();

    return 0;
}
