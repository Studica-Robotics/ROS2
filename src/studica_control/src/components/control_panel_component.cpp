/*
 * control_panel_component.cpp
 *
 * ros2 component for a start, stop and reset button panel with an LED for each.
 * polls the buttons, runs the state machine, drives the LEDs and tells the titan
 * components when motors may run.
 *
 * topic (publishes):  /control_panel/state (String, transient local) — waiting, running, stopped
 * topic (publishes):  /control_panel/reset (Empty) — every reset press
 * topic (publishes):  /motors_enabled (Bool, transient local) — true only while running
 * topic (subscribes): /control_panel/press (String) — software press: start, stop or reset
 */

#include "studica_control/control_panel_component.hpp"

#include <cmath>

namespace studica_control {


std::shared_ptr<rclcpp::Node> ControlPanel::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<int>("control_panel.button_start", -1);
    control->declare_parameter<int>("control_panel.button_stop", -1);
    control->declare_parameter<int>("control_panel.button_reset", -1);
    control->declare_parameter<int>("control_panel.led_start", -1);
    control->declare_parameter<int>("control_panel.led_stop", -1);
    control->declare_parameter<int>("control_panel.led_reset", -1);
    control->declare_parameter<int>("control_panel.press_lockout_ms", 300);
    control->declare_parameter<double>("control_panel.blink_hz", 2.0);

    std::array<int, 3> button_pins;
    button_pins[START] = control->get_parameter("control_panel.button_start").as_int();
    button_pins[STOP]  = control->get_parameter("control_panel.button_stop").as_int();
    button_pins[RESET] = control->get_parameter("control_panel.button_reset").as_int();

    std::array<int, 3> led_pins;
    led_pins[START] = control->get_parameter("control_panel.led_start").as_int();
    led_pins[STOP]  = control->get_parameter("control_panel.led_stop").as_int();
    led_pins[RESET] = control->get_parameter("control_panel.led_reset").as_int();

    int press_lockout_ms = control->get_parameter("control_panel.press_lockout_ms").as_int();
    double blink_hz      = control->get_parameter("control_panel.blink_hz").as_double();

    for (int i = 0; i < 3; i++) {
        if (button_pins[i] < 0 || led_pins[i] < 0) {
            RCLCPP_ERROR(control->get_logger(),
                         "control panel needs all six pins set in params.yaml — motors stay off");
            return nullptr;
        }
    }

    RCLCPP_INFO(control->get_logger(), "control_panel -> buttons: %d/%d/%d, leds: %d/%d/%d",
                button_pins[START], button_pins[STOP], button_pins[RESET],
                led_pins[START], led_pins[STOP], led_pins[RESET]);

    auto panel = std::make_shared<ControlPanel>(vmx, button_pins, led_pins, press_lockout_ms, blink_hz);
    if (!panel->pins_ready()) {
        RCLCPP_ERROR(control->get_logger(), "control panel pins failed to open — motors stay off");
        return nullptr;
    }
    return panel;
}


ControlPanel::ControlPanel(const rclcpp::NodeOptions &options) : rclcpp::Node("control_panel", options) {}


ControlPanel::ControlPanel(std::shared_ptr<VMXPi> vmx, const std::array<int, 3> &button_pins,
                           const std::array<int, 3> &led_pins, int press_lockout_ms, double blink_hz)
    : rclcpp::Node("control_panel"), press_lockout_(press_lockout_ms),
      start_time_(std::chrono::steady_clock::now()),
      blink_hz_(blink_hz > 0.0 ? blink_hz : 2.0)
{
    for (int i = 0; i < 3; i++) {
        buttons_[i] = std::make_shared<studica_driver::DIO>(
            button_pins[i], studica_driver::PinMode::INPUT, vmx);
        leds_[i] = std::make_shared<studica_driver::DIO>(
            led_pins[i], studica_driver::PinMode::OUTPUT, vmx);
    }
    if (!pins_ready()) return;

    for (int i = 0; i < 3; i++) {
        leds_[i]->Set(false);  // opening a pin doesn't set a level, so clear what the last run left
        // the first sample is only a baseline, so a button held during boot is not a press
        last_samples_[i] = buttons_[i]->Get();
        button_levels_[i] = last_samples_[i];
    }

    // transient local so anything starting after the panel still gets the current state
    rclcpp::QoS latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/control_panel/state", latched);
    motors_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("/motors_enabled", latched);
    reset_pub_ = this->create_publisher<std_msgs::msg::Empty>("/control_panel/reset", 10);

    press_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/control_panel/press", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "start") {
                on_press(START, "software");
            } else if (msg->data == "stop") {
                on_press(STOP, "software");
            } else if (msg->data == "reset") {
                on_press(RESET, "software");
            } else {
                RCLCPP_WARN(this->get_logger(), "ignoring unknown press '%s'", msg->data.c_str());
            }
        });

    set_state(State::Waiting);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&ControlPanel::update, this));

    RCLCPP_INFO(this->get_logger(), "control panel ready. press start to enable motors");
}

ControlPanel::~ControlPanel() {
    for (auto &led : leds_) {
        if (led && led->IsInitialized()) led->Set(false);
    }
}


bool ControlPanel::pins_ready() const {
    for (int i = 0; i < 3; i++) {
        if (!buttons_[i] || !buttons_[i]->IsInitialized()) return false;
        if (!leds_[i] || !leds_[i]->IsInitialized()) return false;
    }
    return true;
}


void ControlPanel::update() {
    for (int i = 0; i < 3; i++) {
        bool sample = buttons_[i]->Get();
        if (sample == last_samples_[i] && sample != button_levels_[i]) {
            button_levels_[i] = sample;
            if (!sample) on_press(i, "button");  // high to low: pulled to ground
        }
        last_samples_[i] = sample;
    }
    update_leds();
}


void ControlPanel::update_leds() {
    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_time_;
    bool blink_on = std::fmod(elapsed.count() * blink_hz_, 1.0) < 0.5;

    for (int i = 0; i < 3; i++) {
        bool level = led_modes_[i] == Led::Solid || (led_modes_[i] == Led::Blink && blink_on);
        if (level != led_levels_[i]) {
            led_levels_[i] = level;
            leds_[i]->Set(level);
        }
    }
}


void ControlPanel::on_press(int button, const std::string &source) {
    // stop skips the lockout: pressing it twice is harmless, losing one is not
    auto now = std::chrono::steady_clock::now();
    if (button != STOP && now - last_presses_[button] < press_lockout_) return;
    last_presses_[button] = now;

    if (button == STOP) {
        RCLCPP_INFO(this->get_logger(), "stop pressed (%s)", source.c_str());
        if (state_ != State::Stopped) set_state(State::Stopped);

    } else if (button == START) {
        RCLCPP_INFO(this->get_logger(), "start pressed (%s)", source.c_str());
        if (!button_levels_[STOP]) {
            RCLCPP_WARN(this->get_logger(), "start ignored: stop button is held");
        } else if (state_ == State::Waiting) {
            set_state(State::Running);
        }

    } else {
        RCLCPP_INFO(this->get_logger(), "reset pressed (%s)", source.c_str());
        reset_pub_->publish(std_msgs::msg::Empty());
        if (state_ == State::Stopped) set_state(State::Waiting);
    }
}


void ControlPanel::set_state(State state) {
    state_ = state;

    std_msgs::msg::String state_msg;
    if (state_ == State::Waiting) {
        state_msg.data = "waiting";
        set_led_modes(Led::Blink, Led::Off, Led::Off);
    } else if (state_ == State::Running) {
        state_msg.data = "running";
        set_led_modes(Led::Solid, Led::Off, Led::Off);
    } else {
        state_msg.data = "stopped";
        set_led_modes(Led::Off, Led::Solid, Led::Blink);
    }

    // motors first, so nothing watching the state can act before a stop has been sent
    std_msgs::msg::Bool motors_msg;
    motors_msg.data = (state_ == State::Running);
    motors_enabled_pub_->publish(motors_msg);
    state_pub_->publish(state_msg);

    RCLCPP_INFO(this->get_logger(), "state: %s", state_msg.data.c_str());
}


void ControlPanel::set_led_modes(Led start, Led stop, Led reset) {
    led_modes_[START] = start;
    led_modes_[STOP]  = stop;
    led_modes_[RESET] = reset;
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::ControlPanel)
