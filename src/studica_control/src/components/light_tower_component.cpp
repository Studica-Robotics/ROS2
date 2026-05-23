/*
 * light_tower_component.cpp
 *
 * ROS2 component for a 5-pin LED tower (red / green / yellow / buzzer + continuous).
 *
 * Blink modes:
 *   solid    — continuous=HIGH, color=HIGH
 *   blink_hw — continuous=LOW,  color=HIGH  (hardware drives the flash rate)
 *   blink    — continuous=HIGH, driver toggles color pin at default_blink_hz
 *   <hz>     — continuous=HIGH, driver toggles color pin at the given hz
 *
 * Mutual exclusion is enforced in software: activating any output first clears
 * the currently active one.
 */

#include "studica_control/light_tower_component.hpp"

#include <chrono>
#include <stdexcept>

namespace studica_control {


std::shared_ptr<rclcpp::Node> LightTower::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("light_tower.name",          "light_tower");
    control->declare_parameter<int>        ("light_tower.pin_continuous", -1);
    control->declare_parameter<int>        ("light_tower.pin_red",        -1);
    control->declare_parameter<int>        ("light_tower.pin_green",      -1);
    control->declare_parameter<int>        ("light_tower.pin_yellow",     -1);
    control->declare_parameter<int>        ("light_tower.pin_buzzer",     -1);
    control->declare_parameter<double>     ("light_tower.default_blink_hz", 1.0);

    std::string name         = control->get_parameter("light_tower.name").as_string();
    int pin_continuous       = control->get_parameter("light_tower.pin_continuous").as_int();
    int pin_red              = control->get_parameter("light_tower.pin_red").as_int();
    int pin_green            = control->get_parameter("light_tower.pin_green").as_int();
    int pin_yellow           = control->get_parameter("light_tower.pin_yellow").as_int();
    int pin_buzzer           = control->get_parameter("light_tower.pin_buzzer").as_int();
    double default_blink_hz  = control->get_parameter("light_tower.default_blink_hz").as_double();

    return std::make_shared<LightTower>(vmx, name,
                                        pin_continuous, pin_red, pin_green,
                                        pin_yellow, pin_buzzer, default_blink_hz);
}


LightTower::LightTower(const rclcpp::NodeOptions &options)
    : rclcpp::Node("light_tower", options) {}


LightTower::LightTower(std::shared_ptr<VMXPi> vmx,
                       const std::string &name,
                       int pin_continuous,
                       int pin_red,
                       int pin_green,
                       int pin_yellow,
                       int pin_buzzer,
                       double default_blink_hz)
    : rclcpp::Node(name)
    , default_blink_hz_(default_blink_hz)
{
    tower_ = std::make_shared<studica_driver::LightTower>(
        static_cast<VMXChannelIndex>(pin_continuous),
        static_cast<VMXChannelIndex>(pin_red),
        static_cast<VMXChannelIndex>(pin_green),
        static_cast<VMXChannelIndex>(pin_yellow),
        static_cast<VMXChannelIndex>(pin_buzzer),
        vmx);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/set",
        std::bind(&LightTower::cmd_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    state_pub_ = this->create_publisher<std_msgs::msg::String>(name + "/state", 10);

    // 1 hz heartbeat so late subscribers always see the current state
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LightTower::publish_state, this));

    RCLCPP_INFO(this->get_logger(),
                "light_tower ready. pins: continuous=%d  red=%d  green=%d  yellow=%d  buzzer=%d  "
                "default_blink_hz=%.1f",
                pin_continuous, pin_red, pin_green, pin_yellow, pin_buzzer, default_blink_hz);
}

LightTower::~LightTower()
{
    cancel_blink();
    tower_->AllOff();
}


void LightTower::cmd_callback(
    const std::shared_ptr<studica_control::srv::SetData::Request>  request,
          std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    const std::string &params = request->params;

    // split "color" or "color:mode_or_hz"
    std::string color;
    std::string mode_str;
    auto sep = params.find(':');
    if (sep == std::string::npos) {
        color    = params;
        mode_str = "";
    } else {
        color    = params.substr(0, sep);
        mode_str = params.substr(sep + 1);
    }

    // validate color
    if (color != "off" && color != "red" && color != "green" &&
        color != "yellow" && color != "buzzer") {
        response->success = false;
        response->message = "unknown color '" + color + "' — use: red, green, yellow, buzzer, off";
        return;
    }

    // resolve blink_hz: >0 = sw blink, 0 = solid, <0 = hw blink
    double blink_hz = 0.0;
    if (!mode_str.empty()) {
        if (mode_str == "blink") {
            blink_hz = default_blink_hz_;
        } else if (mode_str == "blink_hw") {
            blink_hz = -1.0;
        } else {
            try {
                blink_hz = std::stod(mode_str);
            } catch (const std::exception &) {
                response->success = false;
                response->message = "invalid mode '" + mode_str
                                  + "' — use: blink, blink_hw, or a numeric hz";
                return;
            }
        }
    }

    set_output(color, blink_hz);

    response->success = true;
    response->message = state_string();
    RCLCPP_INFO(this->get_logger(), "%s", state_string().c_str());
}


void LightTower::set_output(const std::string &color, double blink_hz)
{
    clear_active();
    cancel_blink();

    if (color == "off") {
        active_color_ = "none";
        blink_hz_     = 0.0;
        tower_->AllOff();
        publish_state();
        return;
    }

    active_color_ = color;
    blink_hz_     = blink_hz;

    if (blink_hz > 0.0) {
        // software blink — continuous HIGH, driver toggles the colour pin
        tower_->SetContinuous(true);
        blink_state_ = true;
        set_pin(color, true);
        auto period_ns = static_cast<int64_t>(1e9 / blink_hz);
        blink_timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(period_ns),
            [this, color]() {
                blink_state_ = !blink_state_;
                set_pin(color, blink_state_);
            });
    } else if (blink_hz < 0.0) {
        // hardware blink — continuous LOW, hardware drives the flash rate
        tower_->SetContinuous(false);
        set_pin(color, true);
    } else {
        // solid — continuous HIGH
        tower_->SetContinuous(true);
        set_pin(color, true);
    }

    publish_state();
}


void LightTower::set_pin(const std::string &color, bool value)
{
    if      (color == "red")    tower_->SetRed(value);
    else if (color == "green")  tower_->SetGreen(value);
    else if (color == "yellow") tower_->SetYellow(value);
    else if (color == "buzzer") tower_->SetBuzzer(value);
}


void LightTower::clear_active()
{
    if (active_color_ != "none") {
        set_pin(active_color_, false);
        tower_->SetContinuous(false);
    }
}


void LightTower::cancel_blink()
{
    if (blink_timer_) {
        blink_timer_->cancel();
        blink_timer_.reset();
    }
}


std::string LightTower::state_string() const
{
    if (active_color_ == "none") return "off";
    if (blink_hz_ > 0.0)        return active_color_ + ":blink:" + std::to_string(blink_hz_);
    if (blink_hz_ < 0.0)        return active_color_ + ":blink_hw";
    return active_color_ + ":solid";
}


void LightTower::publish_state()
{
    std_msgs::msg::String msg;
    msg.data = state_string();
    state_pub_->publish(msg);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::LightTower)
