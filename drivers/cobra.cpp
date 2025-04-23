#include "cobra.h"
using namespace studica_driver;

void Cobra::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError: %s\n", p_err_description);
}

Cobra::Cobra(std::shared_ptr<VMXPi> vmx, int vRef) : vmx_(vmx), vRef_(vRef) {
    port = 1;
    deviceAddress = 0x48;
    mode = CONFIG_MODE_SINGLE;
    gain = CONFIG_PGA_2;
    sampleRate = CONFIG_RATE_1600HZ;
    multiplierVolts = 1.0F;

    i2c_ = std::make_shared<I2C>(vmx_);

    try {
        if (!i2c_->isOpen()) {
            printf("Error:  Unable to open VMX via I2C.\n");
            printf("\n");
            printf("        - Is pigpio (or the system resources it requires) in use by another process?\n");
            printf("        - Does this application have root privileges?\n");
        }
        IsConnected();
    }
    catch (const std::exception& ex) {
        printf("Caught exception: %s", ex.what());
    }
}

int Cobra::GetRawValue(uint8_t channel) {
    return GetSingle(channel);
}

float Cobra::GetVoltage(uint8_t channel) {
    float raw = GetSingle(channel);
    float mV = vRef_/0x800;
    return raw * mV / 16;
}

void Cobra::Delay(double seconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(seconds * 1000)));
}

int Cobra::ReadRegister(uint8_t reg) {
    uint8_t tx_buf[1] = { reg };
    uint8_t rx_buf[2] = { 0 };

    if (!i2c_->i2cTransaction(deviceAddress, tx_buf, 1, rx_buf, 2)) {
        return -1;
    }

    return (rx_buf[0] << 8) | rx_buf[1];
}


bool Cobra::IsConnected() {
    uint8_t partID = 0;
    
    bool error = i2c_->ReadI2C(deviceAddress, 0, &partID, 1);

    if (error) {
        printf("Cobra could not be found at 0x%02X!\n", deviceAddress);
        return false;
    } else {
        printf("Cobra is connected at 0x%02X!\n", deviceAddress);
        return true;
    }
}

int Cobra::GetSingle(uint8_t channel) {
    if (channel > 3) {
        std::cout << "Invalid channel: " << (int)channel << std::endl;
        return -1;
    }

    int mux;
    switch (channel) {
        case 0: mux = CONFIG_MUX_SINGLE_0; break;
        case 1: mux = CONFIG_MUX_SINGLE_1; break;
        case 2: mux = CONFIG_MUX_SINGLE_2; break;
        case 3: mux = CONFIG_MUX_SINGLE_3; break;
        default: mux = CONFIG_MUX_SINGLE_0; break;  
    }

    uint16_t config = CONFIG_OS_SINGLE     |
                      mux                  | 
                      CONFIG_PGA_2         |
                      CONFIG_MODE_SINGLE   | 
                      CONFIG_RATE_1600HZ   | 
                      CONFIG_CQUE_NONE;      

    uint8_t write_buf[3];
    write_buf[0] = POINTER_CONFIG;
    write_buf[1] = (config >> 8) & 0xFF;
    write_buf[2] = config & 0xFF;

    if (!i2c_->i2cTransaction(deviceAddress, write_buf, 3, nullptr, 0)) {
        std::cout << "[GetSingle] Failed to write config!" << std::endl;
        return -1;
    }

    Delay(DELAY);  

    uint8_t pointer_buf[1] = { POINTER_CONVERT };
    if (!i2c_->i2cTransaction(deviceAddress, pointer_buf, 1, nullptr, 0)) {
        return -1;
    }

    uint8_t result_buf[2] = {0};
    if (!i2c_->i2cTransaction(deviceAddress, nullptr, 0, result_buf, 2)) {
        return -1;
    }

    return (result_buf[0] << 8) | result_buf[1];
}
