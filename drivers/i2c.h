#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <memory>
#include <iostream>
#include <iomanip>
#include "VMXPi.h"

namespace studica_driver
{

class I2C {
public:
    I2C(std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~I2C();

    uint8_t scanI2CBus();
    bool i2cTransaction(uint8_t device_address, uint8_t* tx_data, size_t tx_size, uint8_t* rx_data, size_t rx_size);

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXResourceHandle i2c_res_handle_;
    I2CConfig i2c_cfg;
    
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_driver

#endif // I2C_H