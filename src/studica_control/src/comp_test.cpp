#include <memory>
#include "studica_control/encoder_component.h"
#include "studica_control/servo_component.h"
#include "studica_control/sharp_component.h"
#include "studica_control/cobra_component.h"
#include "rclcpp/rclcpp.hpp"
#include "VMXPi.h"

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto response = std::make_shared<studica_control::srv::SetData::Response>();

    // // sharp
    // auto sharp = std::make_shared<studica_control::Sharp>("shp", 22, vmx);
    // exec.add_node(sharp);
    // sharp->cmd("get_distance", response);
    // std::cout << "response->message: " << response->message << std::endl;

    // cobra
    auto cobra = std::make_shared<studica_control::Cobra>("cbr", 5.0, 1);
    exec.add_node(cobra);
    cobra->cmd("get_volt", response);
    std::cout << "response->message: " << response->message << std::endl;

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
