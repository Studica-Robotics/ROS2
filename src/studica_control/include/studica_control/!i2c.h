#ifndef I2C_HPP_
#define I2C_HPP_

#include <rclcpp/rclcpp.hpp>
#include "VMXPi.h"

class I2CHandler {
public:
    I2CHandler(std::shared_ptr<VMXPi> vmx);

    bool activateI2CResource(VMXResourceHandle& resource_handle);
    uint8_t scanI2CBus();
    bool i2cTransaction(uint8_t device_address, uint8_t* tx_data, size_t tx_size, uint8_t* rx_data, size_t rx_size);

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXResourceHandle i2c_res_handle;
    I2CConfig i2c_cfg;
};

#endif // I2C_HPP_
