#ifndef COBRA_SENSOR_NODE_HPP_
#define COBRA_SENSOR_NODE_HPP_

#define DELAY                       0.001
#define POINTER_CONVERT             0X0000
#define POINTER_CONFIG              0x0001
#define POINTER_LOWTHRESH           0x0002
#define POINTER_HITHRESH            0x0003
#define CONFIG_OS_NO                0x8000
#define CONFIG_OS_SINGLE            0x8000
#define CONFIG_OS_READY             0x8000
#define CONFIG_OS_NOTREADY          0x8000
#define CONFIG_MODE_CONT            0x0000
#define CONFIG_MODE_SINGLE          0x0100
#define CONFIG_MUX_SINGLE_0         0x4000
#define CONFIG_MUX_SINGLE_1         0x5000
#define CONFIG_MUX_SINGLE_2         0x6000
#define CONFIG_MUX_SINGLE_3         0x7000
#define CONFIG_MUX_DIFF_PO_N1       0x0000
#define CONFIG_MUX_DIFF_PO_N3       0x1000
#define CONFIG_MUX_DIFF_P1_N3       0x2000
#define CONFIG_MUX_DIFF_P2_N3       0x3000
#define CONFIG_RATE_128HZ           0x0000
#define CONFIG_RATE_250HZ           0x0020
#define CONFIG_RATE_490HZ           0x0040
#define CONFIG_RATE_920HZ           0x0060
#define CONFIG_RATE_1600HZ          0x0080
#define CONFIG_RATE_2400HZ          0x00A0
#define CONFIG_RATE_3300HZ          0x00C0
#define CONFIG_PGA_MASK             0x0E00
#define CONFIG_PGA_TWOTHIRDS        0x0000 // +/- 6.144V
#define CONFIG_PGA_1                0x0200 // +/- 4.096V
#define CONFIG_PGA_2                0x0400 // +/- 2.048V
#define CONFIG_PGA_4                0x0600 // +/- 1.024V
#define CONFIG_PGA_8                0x0800 // +/- 0.512V
#define CONFIG_PGA_16               0x0A00 // +/- 0.256V
#define CONFIG_CMODE_TRAD           0x0000
#define CONFIG_CMODE_WINDOW         0x0010
#define CONFIG_CPOL_ACTVLOW         0x0000
#define CONFIG_CPOL_ACTVHI          0x0008
#define CONFIG_CLAT_NOLAT           0x0000
#define CONFIG_CLAT_LATCH           0x0004
#define CONFIG_CQUE_1CONV           0x0000
#define CONFIG_CQUE_2CONV           0x0001
#define CONFIG_CQUE_4CONV           0x0002
#define CONFIG_CQUE_NONE            0x0003

#include "studica_control/device.h"
#include "studica_control/i2c.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "VMXPi.h"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <studica_control/srv/set_data.hpp>


class CobraSensor : public Device {
public:
    CobraSensor(std::shared_ptr<VMXPi> vmx, std::shared_ptr<I2CHandler> i2c_handler, const std::string &name, const float &vRef, const int &mux_ch);

    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) override;

    void start_publishing();
    void stop_publishing();

    float GetVoltage(uint8_t channel);
    int GetRawValue(uint8_t channel);
    VMXResourceHandle cobra_res_handle;
private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<I2CHandler> i2c_handler_;
    float vRef_;
    int mux_ch_;
    bool is_publishing_;
    uint8_t deviceAddress;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cobra_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int GetSingle(uint8_t channel);
    void Delay(double seconds);
    void Spin();  // Called in the timer callback
    bool IsConnected();
    void publish_cobra_data();
    int port;
    
    // ADC config
    int mode;
    int gain;
    int sampleRate;
    float multiplierVolts;
};

#endif // COBRA_SENSOR_NODE_HPP_
