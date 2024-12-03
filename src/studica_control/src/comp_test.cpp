#include <memory>
#include "studica_control/encoder_component.h"
#include "studica_control/servo_component.h"
#include "studica_control/sharp_component.h"
#include "studica_control/cobra_component.h"
#include "studica_control/titan_component.h"
#include "rclcpp/rclcpp.hpp"
#include "VMXPi.h"
#include <thread>
#include <chrono>
#include <iostream>

// Function to execute commands for a Titan instance
void runTitanCommands(std::shared_ptr<studica_control::Titan> titan, const std::string& titan_name) {
    // Create a shared request object
    auto request = std::make_shared<studica_control::srv::SetData::Request>();
    auto response = std::make_shared<studica_control::srv::SetData::Response>();

    request->initparams.n_encoder = 2;
    // Start titan
    titan->cmd("start", request, response);
    std::cout << titan_name << " response->message: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Setup encoder
    titan->cmd("setup_enc", request, response);
    std::cout << titan_name << " response->message: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    
    // Set speed
    request->initparams.speed = 0.8; // set desired speed
    titan->cmd("set_speed", request, response);
    std::cout << titan_name << " response->message: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Get encoder distance
    titan->cmd("get_enc_dist", request, response);
    std::cout << titan_name << " response->message: get_enc_dist: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Get encoder count
    titan->cmd("get_enc_cnt", request, response);
    std::cout << titan_name << " response->message: get_enc_cnt: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Get RPM
    titan->cmd("get_rpm", request, response);
    std::cout << titan_name << " response->message: get_rpm: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Invert encoder direction
    titan->cmd("inv_motor_dir", request, response);
    std::cout << titan_name << " response->message: inv_motor_dir: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // slower speed
    request->initparams.speed = 0.2; // Set the desired speed here
    titan->cmd("set_speed", request, response);
    std::cout << titan_name << " response->message: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Get encoder distance again
    titan->cmd("get_enc_dist", request, response);
    std::cout << titan_name << " response->message: get_enc_dist: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Get encoder count again
    titan->cmd("get_enc_cnt", request, response);
    std::cout << titan_name << " response->message: get_enc_cnt: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Get RPM again
    titan->cmd("get_rpm", request, response);
    std::cout << titan_name << " response->message: get_rpm: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Reset titan
    titan->cmd("reset", request, response);
    std::cout << titan_name << " response->message: reset: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    // Stop titan
    titan->cmd("stop", request, response);
    std::cout << titan_name << " response->message: stop: " << response->message << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

        // // sharp
    // auto sharp = std::make_shared<studica_control::Sharp>("shp", 22, vmx);
    // exec.add_node(sharp);
    // sharp->cmd("get_distance", response);
    // std::cout << "response->message: " << response->message << std::endl;

    // // cobra
    // auto cobra = std::make_shared<studica_control::Cobra>("cbr", 5.0, 1, vmx);
    // exec.add_node(cobra);
    // cobra->cmd("get_volt", response);
    // std::cout << "response->message: " << response->message << std::endl;

    std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);
    auto titan = std::make_shared<studica_control::Titan>(vmx, "ttn", 45, 15600, 0.0006830601, 0.8);
    exec.add_node(titan);

    // Start threads for each Titan instance
    std::thread titanThread1(runTitanCommands, titan, "Titan 0");

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
