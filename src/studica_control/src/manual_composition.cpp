#include <memory>

#include "studica_control/encoder_component.h"
#include "studica_control/servo_component.h"
#include "rclcpp/rclcpp.hpp"
#include "VMXPi.h"

int main(int argc, char *argv[])
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // This will also parse command line arguments one day (as of Beta 1 they are not used).
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With this version, all callbacks will be called from within this thread (the main one).
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    // Add some nodes to the executor which provide work for the executor during its "spin" function.
    // An example of available work is executing a subscription callback, or a timer callback.
    // auto encoder = std::make_shared<studica_control::Encoder>(options);
    // exec.add_node(encoder);

    auto servo = std::make_shared<studica_control::Servo>(13, studica_driver::ServoType::Standard);
    // wait 2 seconds
    exec.add_node(servo);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    servo->cmd("0", std::make_shared<studica_control::srv::SetData::Response>());

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    exec.spin();

    rclcpp::shutdown();

    return 0;
}