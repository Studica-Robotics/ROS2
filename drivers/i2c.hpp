#pragma once

#include "VMXPi.h"
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdio.h>

namespace studica_driver
{

    class I2C
    {
        public:
            I2C(std::shared_ptr<VMXPi> vmx);
            ~I2C();

            uint8_t scanI2CBus();
            bool i2cTransaction(uint8_t device_address, uint8_t* tx_data, size_t tx_size, uint8_t* rx_data,
                                size_t rx_size);

            bool WriteI2C(uint8_t deviceAddress, int registerAddress, uint8_t* data, size_t data_size);
            bool ReadI2C(uint8_t deviceAddress, int registerAddress, uint8_t* data, size_t count);

            bool isOpen() const
            {
                return vmx_->IsOpen();
            }

        private:
            std::shared_ptr<VMXPi> vmx_;
            VMXResourceHandle i2c_res_handle_;
            I2CConfig i2c_cfg;

            void DisplayVMXError(VMXErrorCode vmxerr);
    };

} // namespace studica_driver
