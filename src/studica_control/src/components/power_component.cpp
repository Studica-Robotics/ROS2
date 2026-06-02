/*
 * power_component.cpp
 *
 * ros2 component for the vmx-pi onboard power monitor.
 * reads the system (battery) voltage via vmx.power.GetSystemVoltage()
 * and publishes it as a sensor_msgs/BatteryState message at 2 hz.
 *
 * battery: 10-cell NiMH, 3.0 Ah, nominal 12 V
 *   max (off charger): ~14.0 V   →  100 %
 *   min (cutoff):      ~10.0 V   →    0 %
 *   below 10.0 V is damaging and the robot begins to shut down
 *
 * percentage is estimated from a voltage lookup table derived from a
 * constant-current (10 A) discharge test.
 *
 * topic: /battery_state  (sensor_msgs/BatteryState)
 */

#include "studica_control/power_component.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace studica_control
{

    // ---------------------------------------------------------------------------
    // NiMH voltage → percentage lookup table
    // Points extracted from the 10 A discharge curve.
    // Each entry is { voltage_V, fraction_0_to_1 }.
    // Table must be sorted descending by voltage.
    // ---------------------------------------------------------------------------
    static constexpr std::array<std::pair<float, float>, 11> k_nimh_curve{{
        {14.06f, 1.00f}, // fresh off charger (surface charge)
        {12.90f, 0.94f}, // surface charge dissipated
        {12.50f, 0.86f},
        {12.20f, 0.72f},
        {12.00f, 0.58f},
        {11.85f, 0.44f},
        {11.65f, 0.30f},
        {11.20f, 0.16f},
        {10.90f, 0.10f},
        {10.30f, 0.04f},
        {10.00f, 0.00f}, // cutoff
    }};

    // linearly interpolates the lookup table to estimate state-of-charge
    static float voltage_to_percentage(float v)
    {
        if (v >= k_nimh_curve.front().first)
            return 1.0f;
        if (v <= k_nimh_curve.back().first)
            return 0.0f;

        for (std::size_t i = 1; i < k_nimh_curve.size(); ++i)
        {
            const auto [v_hi, p_hi] = k_nimh_curve[i - 1];
            const auto [v_lo, p_lo] = k_nimh_curve[i];
            if (v <= v_hi && v >= v_lo)
            {
                float t = (v - v_lo) / (v_hi - v_lo); // 0..1 within segment
                return p_lo + t * (p_hi - p_lo);
            }
        }
        return 0.0f; // unreachable
    }

    // ---------------------------------------------------------------------------
    // component lifecycle
    // ---------------------------------------------------------------------------

    std::shared_ptr<rclcpp::Node> Power::initialize(rclcpp::Node* control, std::shared_ptr<VMXPi> vmx)
    {
        control->declare_parameter<int>("power.battery_count", 1);
        int battery_count = control->get_parameter("power.battery_count").as_int();

        if (battery_count < 1 || battery_count > 2)
        {
            RCLCPP_WARN(control->get_logger(), "power.battery_count=%d is invalid — must be 1 or 2. Defaulting to 1.",
                        battery_count);
            battery_count = 1;
        }

        return std::make_shared<Power>(vmx, battery_count);
    }

    Power::Power(const rclcpp::NodeOptions& options)
        : rclcpp::Node("power", options)
        , design_capacity_ah_(3.0f)
    {
    }

    Power::Power(std::shared_ptr<VMXPi> vmx, int battery_count)
        : rclcpp::Node("power")
        , vmx_(vmx)
        , design_capacity_ah_(3.0f * static_cast<float>(battery_count))
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), // 2 hz — voltage changes slowly
                                         std::bind(&Power::publish_data, this));

        RCLCPP_INFO(this->get_logger(),
                    "power component ready — %d × 3.0 Ah NiMH (%.1f Ah total), "
                    "publishing on /battery_state at 2 Hz",
                    battery_count, design_capacity_ah_);
    }

    Power::~Power()
    {
    }

    // ---------------------------------------------------------------------------
    // publish
    // ---------------------------------------------------------------------------

    void Power::publish_data()
    {
        using BatteryState = sensor_msgs::msg::BatteryState;
        constexpr float unknown = std::numeric_limits<float>::quiet_NaN();

        BatteryState msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = ""; // system-level reading — no tf frame

        // 10-cell NiMH pack
        msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
        msg.design_capacity = design_capacity_ah_;
        msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        msg.present = true;

        // fields not available without a coulomb counter
        msg.current = unknown;
        msg.charge = unknown;
        msg.capacity = unknown; // actual remaining capacity (would need integration)

        // read voltage
        VMXErrorCode vmxerr;
        float voltage = 0.0f;
        if (!vmx_->power.GetSystemVoltage(voltage, &vmxerr))
        {
            msg.voltage = unknown;
            msg.percentage = unknown;
            msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "failed to read battery voltage (VMXError %d)", vmxerr);
            publisher_->publish(msg);
            return;
        }

        msg.voltage = voltage;
        msg.percentage = voltage_to_percentage(voltage);

        // health — flag when the battery is approaching the damage threshold
        if (voltage >= 11.0f)
        {
            msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        }
        else if (voltage >= 10.0f)
        {
            msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_DEAD;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "LOW BATTERY: %.2f V (%.0f %%) — land/stop the robot", voltage,
                                 msg.percentage * 100.0f);
        }
        else
        {
            msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_DEAD;
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "CRITICAL BATTERY: %.2f V — below safe minimum (10.0 V), "
                                  "robot shutting down",
                                  voltage);
        }

        publisher_->publish(msg);
    }

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Power)
