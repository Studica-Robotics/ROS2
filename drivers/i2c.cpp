#include "i2c.h"
using namespace studica_driver;

I2C::I2C(std::shared_ptr<VMXPi> vmx) : vmx_(vmx), i2c_res_handle_(VMXResourceHandle()) {
    if (!vmx_->IsOpen()) {
        std::cout << "Unable to open VMX." << std::endl;
    } else {
        VMXErrorCode vmxerr;
        VMXChannelInfo i2c_channels[2] = {
            {VMXChannelInfo(vmx_->getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SDA), VMXChannelCapability::I2C_SDA)},
            {VMXChannelInfo(vmx_->getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SCL), VMXChannelCapability::I2C_SCL)}
        };

        if (!vmx_->io.ActivateDualchannelResource(i2c_channels[0], i2c_channels[1], &i2c_cfg, i2c_res_handle_, &vmxerr)) {
            std::cout << "Error Activating DualChannel Resource for Channel index " << i2c_channels[0].index << " and " << i2c_channels[1].index << std::endl;
        } else {
            std::cout << "Successfully Activated I2C Resource." << std::endl;
        }
    }
}
I2C::~I2C() {}

uint8_t I2C::scanI2CBus() {
    for (uint8_t address = 1; address < 127; ++address) {
        uint8_t partID = 0;
        if (i2cTransaction(address, nullptr, 0, &partID, 1)) {
            std::cout << "Found I2C device at address 0x" << std::hex << std::uppercase << address << std::dec << std::endl;
            return address;  // return first found device address
        }
    }
    return 0;  // no i2c device found
}

bool I2C::i2cTransaction(uint8_t device_address, uint8_t* tx_data, size_t tx_size, uint8_t* rx_data, size_t rx_size) {
    VMXErrorCode vmxerr;
    return vmx_->io.I2C_Transaction(i2c_res_handle_, device_address, tx_data, tx_size, rx_data, rx_size, &vmxerr);
}