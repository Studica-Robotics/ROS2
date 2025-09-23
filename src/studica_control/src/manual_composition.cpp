#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "studica_control/cobra_component.h"
#include "studica_control/dc_encoder_component.h"
#include "studica_control/diff_drive_component.h"
#include "studica_control/dio_component.h"
#include "studica_control/encoder_component.h"
// #include "studica_control/gamepad_component.h"
#include "studica_control/imu_component.h"
#include "studica_control/mecanum_drive_component.h"
#include "studica_control/servo_component.h"
#include "studica_control/sharp_component.h"
#include "studica_control/titan_component.h"
#include "studica_control/ultrasonic_component.h"
#include "VMXPi.h"

class ControlServer : public rclcpp::Node {
public:
    ControlServer() : Node("control_server") {
        vmx_ = std::make_shared<VMXPi>(true, 50);
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    }

    bool initialize() {
        this->declare_parameter<bool>("cobra.enabled", false);
        this->declare_parameter<bool>("duty_cycle.enabled", false);
        this->declare_parameter<bool>("diff_drive_component.enabled", false);
        this->declare_parameter<bool>("dio.enabled", false);
        this->declare_parameter<bool>("encoder.enabled", false);
        // this->declare_parameter<bool>("gamepad.enabled", false);
        this->declare_parameter<bool>("imu.enabled", false);
        this->declare_parameter<bool>("mecanum_drive_component.enabled", false);
        this->declare_parameter<bool>("servo.enabled", false);
        this->declare_parameter<bool>("sharp.enabled", false);
        this->declare_parameter<bool>("titan.enabled", false);
        this->declare_parameter<bool>("ultrasonic.enabled", false);

        bool cobra_enabled = this->get_parameter("cobra.enabled").as_bool();
        bool duty_cycle_enabled = this->get_parameter("duty_cycle.enabled").as_bool();
        bool diff_drive_enabled = this->get_parameter("diff_drive_component.enabled").as_bool();
        bool dio_enabled = this->get_parameter("dio.enabled").as_bool();
        bool encoder_enabled = this->get_parameter("encoder.enabled").as_bool();
        // bool gamepad_enabled = this->get_parameter("gamepad.enabled").as_bool();
        bool imu_enabled = this->get_parameter("imu.enabled").as_bool();
        bool mecanum_drive_enabled = this->get_parameter("mecanum_drive_component.enabled").as_bool();
        bool servo_enabled = this->get_parameter("servo.enabled").as_bool();
        bool sharp_enabled = this->get_parameter("sharp.enabled").as_bool();
        bool titan_enabled = this->get_parameter("titan.enabled").as_bool();
        bool ultrasonic_enabled = this->get_parameter("ultrasonic.enabled").as_bool();

        if (diff_drive_enabled && mecanum_drive_enabled) {
            RCLCPP_ERROR(this->get_logger(), "Cannot initialize both differential and mecanum drive controllers. Make sure only one of them is enabled.");
            return false;
        }

        if (cobra_enabled) {
            auto cobra_nodes = studica_control::Cobra::initialize(this, vmx_);
            component_nodes.insert(component_nodes.end(), cobra_nodes.begin(), cobra_nodes.end());
        }

        if (duty_cycle_enabled) {
            auto duty_cycle_nodes = studica_control::DutyCycleEncoder::initialize(this, vmx_);
            component_nodes.insert(component_nodes.end(), duty_cycle_nodes.begin(), duty_cycle_nodes.end());
        }

        if (diff_drive_enabled) {
            auto odom = studica_control::DiffOdometry::initialize(this);
            auto diff_drive_node = studica_control::DiffDrive::initialize(this, odom, vmx_);
            component_nodes.push_back(odom);
            component_nodes.push_back(diff_drive_node);
        }

        if (dio_enabled) {
            auto dio_nodes = studica_control::DIO::initialize(this, vmx_);
            component_nodes.insert(component_nodes.end(), dio_nodes.begin(), dio_nodes.end());
        }

        if (encoder_enabled) {
            auto encoder_nodes = studica_control::Encoder::initialize(this, vmx_);
            component_nodes.insert(component_nodes.end(), encoder_nodes.begin(), encoder_nodes.end());
        }

        // if (gamepad_enabled) {
        //     auto gamepad_node = studica_control::GamepadController::initialize(this);
        //     component_nodes.push_back(gamepad_node);
        // }

        if (imu_enabled) {
            auto imu_node = studica_control::Imu::initialize(this, vmx_);
            component_nodes.push_back(imu_node);
        }

        if (mecanum_drive_enabled) {
            auto odom = studica_control::MecanumOdometry::initialize(this);
            auto mecanum_drive_node = studica_control::MecanumDrive::initialize(this, odom, vmx_);
            component_nodes.push_back(odom);
            component_nodes.push_back(mecanum_drive_node);
        }

        if (servo_enabled) {
            auto servo_nodes = studica_control::Servo::initialize(this, vmx_);
            component_nodes.insert(component_nodes.end(), servo_nodes.begin(), servo_nodes.end());
        }

        if (sharp_enabled) {
            auto sharp_nodes = studica_control::Sharp::initialize(this, vmx_);
            component_nodes.insert(component_nodes.end(), sharp_nodes.begin(), sharp_nodes.end());
        }

        if (titan_enabled) {
            auto titan_nodes = studica_control::Titan::initialize(this, vmx_);
            component_nodes.insert(component_nodes.end(), titan_nodes.begin(), titan_nodes.end());
        }

        if (ultrasonic_enabled) {
            auto ultrasonic_nodes = studica_control::Ultrasonic::initialize(this, vmx_);
            component_nodes.insert(component_nodes.end(), ultrasonic_nodes.begin(), ultrasonic_nodes.end());
        }

        for (const auto &node : component_nodes) {
            executor_->add_node(node);
        }

        return true;
    }

    void run() {
        executor_->add_node(this->shared_from_this());
        executor_->spin();
    }

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::vector<std::shared_ptr<rclcpp::Node>> component_nodes;
};

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto control_server = std::make_shared<ControlServer>();
    if (control_server->initialize()) {
        control_server->run();
    }

    rclcpp::shutdown();
    return 0;
}
