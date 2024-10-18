#include "studica_control/i2c.h"
#include <cstdint>

I2CHandler::I2CHandler(std::shared_ptr<VMXPi> vmx) : vmx_(vmx) {}

bool I2CHandler::activateI2CResource(VMXResourceHandle& resource_handle) {
    VMXErrorCode vmxerr;

    VMXChannelInfo i2c_channels[2] = {
        {VMXChannelInfo(vmx_->getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SDA), VMXChannelCapability::I2C_SDA)},
        {VMXChannelInfo(vmx_->getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SCL), VMXChannelCapability::I2C_SCL)}
    };

    if (!vmx_->io.ActivateDualchannelResource(i2c_channels[0], i2c_channels[1], &i2c_cfg, resource_handle, &vmxerr)) {
        RCLCPP_ERROR(rclcpp::get_logger("I2CHandler"), "Error Activating DualChannel Resource for Channel index %d and %d.\n", i2c_channels[0].index, i2c_channels[1].index);
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("I2CHandler"), "Successfully Activated I2CHandler Resource.");
        i2c_res_handle = resource_handle;
        return true;
    }
}

uint8_t I2CHandler::scanI2CBus() {
    for (uint8_t address = 1; address < 127; ++address) {
        uint8_t partID = 0;
        if (i2cTransaction(address, nullptr, 0, &partID, 1)) {
            RCLCPP_INFO(rclcpp::get_logger("I2CHandler"), "Found I2CHandler device at address 0x%02X", address);
            return address;  // Return the first found device address
        }
    }
    return 0;  // No device found
}

bool I2CHandler::i2cTransaction(uint8_t device_address, uint8_t* tx_data, size_t tx_size, uint8_t* rx_data, size_t rx_size) {
    VMXErrorCode vmxerr;
    return vmx_->io.I2C_Transaction(i2c_res_handle, device_address, tx_data, tx_size, rx_data, rx_size, &vmxerr);
}
