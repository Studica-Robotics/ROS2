/*
 * dio_component.cpp
 *
 * ros2 component for a digital input/output pin on the vmx-pi board.
 * mode is set in params.yaml and cannot change at runtime.
 *
 * topic (publishes):  <name>/state (std_msgs/Bool) — pin state at 10 hz,
 *                     plus immediately on every interrupt edge (if configured)
 * topic (subscribes): <name>/cmd   (std_msgs/Bool) — set pin directly (output only)
 * service: <name>/dio_cmd (studica_control/SetData)
 *   toggle — flip output state (output mode only)
 */

#include "studica_control/dio_component.hpp"

namespace studica_control {


std::vector<std::shared_ptr<rclcpp::Node>>
DIO::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx)
{
    std::vector<std::shared_ptr<rclcpp::Node>> dio_nodes;

    control->declare_parameter<std::vector<std::string>>("dio.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("dio.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string pin_param          = "dio." + sensor + ".pin";
        std::string type_param         = "dio." + sensor + ".type";
        std::string int_edge_param     = "dio." + sensor + ".interrupt_edge";
        std::string debounce_param     = "dio." + sensor + ".debounce_ms";

        control->declare_parameter<int>(pin_param, -1);
        control->declare_parameter<std::string>(type_param, "");
        control->declare_parameter<std::string>(int_edge_param, "none");
        control->declare_parameter<int>(debounce_param, 0);

        int pin                  = control->get_parameter(pin_param).as_int();
        std::string type         = control->get_parameter(type_param).as_string();
        std::string int_edge     = control->get_parameter(int_edge_param).as_string();
        int debounce_ms          = control->get_parameter(debounce_param).as_int();

        RCLCPP_INFO(control->get_logger(), "%s -> pin: %d, type: %s, interrupt_edge: %s, debounce: %d ms",
                    sensor.c_str(), pin, type.c_str(), int_edge.c_str(), debounce_ms);

        if (type == "input") {
            dio_nodes.push_back(std::make_shared<DIO>(
                vmx, sensor, pin, studica_driver::PinMode::INPUT, int_edge, debounce_ms));
        } else if (type == "output") {
            dio_nodes.push_back(std::make_shared<DIO>(
                vmx, sensor, pin, studica_driver::PinMode::OUTPUT, int_edge, debounce_ms));
        } else {
            RCLCPP_ERROR(control->get_logger(),
                         "invalid dio type '%s' for '%s' — use 'input' or 'output'",
                         type.c_str(), sensor.c_str());
        }
    }

    return dio_nodes;
}


DIO::DIO(const rclcpp::NodeOptions &options) : rclcpp::Node("dio", options) {}


DIO::DIO(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex pin,
         studica_driver::PinMode pin_mode, const std::string &interrupt_edge, int debounce_ms)
    : rclcpp::Node(name), vmx_(vmx), pin_(pin), pin_mode_(pin_mode)
{
    dio_ = std::make_shared<studica_driver::DIO>(pin_, pin_mode_, vmx_);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/dio_cmd",
        std::bind(&DIO::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // output pins get a cmd subscriber for direct set
    if (pin_mode_ == studica_driver::PinMode::OUTPUT) {
        cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            name + "/cmd", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                dio_->Set(msg->data);
            });
        RCLCPP_INFO(this->get_logger(), "dio output ready on pin %d. cmd: /%s/cmd", pin_, name.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "dio input ready on pin %d.", pin_);
    }

    publisher_ = this->create_publisher<std_msgs::msg::Bool>(name + "/state", 10);

    // 10 hz polling — always present regardless of interrupt mode
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DIO::publish_dio_state, this));

    // -----------------------------------------------------------------------
    // Hardware interrupt (input mode only)
    // -----------------------------------------------------------------------
    if (pin_mode_ == studica_driver::PinMode::INPUT && interrupt_edge != "none") {

        InterruptConfig::InterruptEdge vmx_edge;
        if (interrupt_edge == "falling") {
            vmx_edge = InterruptConfig::FALLING;
        } else {
            if (interrupt_edge != "rising") {
                RCLCPP_WARN(this->get_logger(),
                            "unknown interrupt_edge '%s' for pin %d — defaulting to 'rising'",
                            interrupt_edge.c_str(), pin_);
            }
            vmx_edge = InterruptConfig::RISING;
        }

        // The interrupt callback fires in a VMX background thread.
        // rclcpp publisher::publish() is thread-safe, so we publish directly.
        // Captures publisher_ by shared_ptr value — safe across threads.
        auto pub = publisher_;
        auto logger = this->get_logger();
        int captured_pin = pin_;

        bool ok = dio_->EnableInterrupt(vmx_edge,
            [pub, logger, captured_pin](bool pin_state, InterruptEdgeType edge) {
                std_msgs::msg::Bool msg;
                msg.data = pin_state;
                pub->publish(msg);

                const char *edge_str =
                    (edge == InterruptEdgeType::RISING_EDGE_INTERRUPT) ? "rising" : "falling";
                RCLCPP_INFO(logger, "interrupt pin %d: %s edge → %s",
                            captured_pin, edge_str, pin_state ? "HIGH" : "LOW");
            }, debounce_ms);

        if (ok) {
            if (debounce_ms > 0) {
                RCLCPP_INFO(this->get_logger(),
                            "hardware interrupt enabled on pin %d (%s edge, debounce %d ms)",
                            pin_, interrupt_edge.c_str(), debounce_ms);
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "hardware interrupt enabled on pin %d (%s edge, no debounce)",
                            pin_, interrupt_edge.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "failed to enable interrupt on pin %d — falling back to polling only",
                        pin_);
        }
    }
}

DIO::~DIO() {}


void DIO::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        if (request->params == "toggle") {
            if (pin_mode_ != studica_driver::PinMode::OUTPUT) {
                response->success = false;
                response->message = "toggle is only valid on output pins";
                return;
            }
            dio_->Set(!dio_->Get());
            response->success = true;
            response->message = "pin toggled to " + std::string(dio_->Get() ? "high" : "low");
        } else {
            response->success = false;
            response->message = "unknown command '" + request->params + "'";
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "failed to set dio pin: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "failed to set dio pin: %s", e.what());
    }
}


void DIO::publish_dio_state() {
    try {
        std_msgs::msg::Bool msg;
        msg.data = dio_->Get();
        publisher_->publish(msg);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "failed to read dio state: %s", e.what());
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DIO)
