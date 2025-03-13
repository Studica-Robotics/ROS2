#include "cobra.h"
using namespace studica_driver;

void Cobra::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError &d: %s\n", p_err_description);
}

Cobra::Cobra(std::shared_ptr<VMXPi> vmx, int vRef) : vmx_(vmx), vRef_(vRef){
    port = 1;
    deviceAddress = 0x48;
    mode = CONFIG_MODE_CONT;
    gain = CONFIG_PGA_2;
    sampleRate = CONFIG_RATE_1600HZ;
    multiplierVolts = 1.0F;

    i2c_ = std::make_shared<I2C>(vmx_);

    try {
        if(!i2c_->isOpen()) {
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
    return raw * mV;
}


void Cobra::Delay(double seconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(seconds * 1000)));
}

int Cobra::ReadRegister(uint8_t location) {
    uint8_t buffer[2];
    uint8_t data[1];
    data[0] = POINTER_CONVERT;
    i2c_->WriteI2C(deviceAddress, location, data, 1);
    Delay(DELAY);
    i2c_->ReadI2C(deviceAddress, location, buffer, 2);
    return (int)((buffer[0]<<8) & 0xFF00) | (buffer[1] & 0xFF);
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
        return 0;
    }
    
    int config = CONFIG_OS_SINGLE | mode | sampleRate;
    config |= gain;
    
    switch (channel) {
        case(0):
            config |= CONFIG_MUX_SINGLE_0;
            break;
        case(1):
            config |= CONFIG_MUX_SINGLE_1;
            break;
        case(2):
            config |= CONFIG_MUX_SINGLE_2;
            break;
        case(3):
            config |= CONFIG_MUX_SINGLE_3;
            break;
    }

    uint8_t raw[2];
    raw[0] = (uint8_t) (config>>8);
    raw[1] = (uint8_t) (config & 0xFF);
    i2c_->WriteI2C(deviceAddress, POINTER_CONVERT, raw, 2);
    Delay(DELAY);
    return ReadRegister(POINTER_CONVERT)>>4;
}