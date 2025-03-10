#include "cobra.h"
using namespace studica_driver;

void Cobra::DisplayVMXError(VMXErrorCode vmxerr)
{
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError &d: %s\n", p_err_description);
}

Cobra::Cobra(int _vRef)
{
    port = 1;
    deviceAddress = 0x48;
    vRef = _vRef;
    mode = CONFIG_MODE_CONT;
    gain = CONFIG_PGA_2;
    sampleRate = CONFIG_RATE_1600HZ;
    multiplierVolts = 1.0F;
    try
    {
        if(vmx.IsOpen())
        {
            VMXErrorCode vmxerr;
            VMXChannelInfo i2c_channels[2] = 
            {
                { 
                    VMXChannelInfo(vmx.getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SDA), VMXChannelCapability::I2C_SDA) 
                },
                {    
                    VMXChannelInfo(vmx.getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SCL), VMXChannelCapability::I2C_SCL) 
                }
            };

			I2CConfig i2c_cfg;

			if (!vmx.io.ActivateDualchannelResource(i2c_channels[0], i2c_channels[1], &i2c_cfg, i2c_res_handle, &vmxerr)) 
            {
				printf("Failed to Activate I2C Resource.\n");
				DisplayVMXError(vmxerr);
			} 
            else 
            {
				printf("Successfully Activated I2C Resource with VMXChannels %d and %d\n",
						i2c_channels[0].index, i2c_channels[1].index);
			}
        }
        else
        {
            printf("Error:  Unable to open VMX Client.\n");
			printf("\n");
			printf("        - Is pigpio (or the system resources it requires) in use by another process?\n");
			printf("        - Does this application have root privileges?\n");
        }
        IsConnected();
    }
    catch (const std::exception& ex)
    {
        printf("Caught exception: %s", ex.what());
    }
}
// Cobra::~Cobra() {}

int Cobra::GetRawValue(uint8_t channel)
{
    return GetSingle(channel);
}

float Cobra::GetVoltage(uint8_t channel)
{
    float raw = GetSingle(channel);
    float mV = vRef/0x800;
    return raw * mV;
}

bool Cobra::WriteI2C(int registerAddress, uint8_t* data)
{
    VMXErrorCode vmxerr;
    if(!vmx.io.I2C_Write(i2c_res_handle, deviceAddress, registerAddress, data, sizeof(data), &vmxerr))
    {
        printf("Error Writing to I2C bus!");
        return true;
    }
    else
    {
        return false;
    }
}

bool Cobra::ReadI2C(int count, int registerAddress, uint8_t* data)
{
    VMXErrorCode vmxerr;
    if (count < 1)
    {
        printf("I2C Read count out of range");
        return true;
    }
    if (data == nullptr)
    {
        printf("I2C Read data is a null pointer");
        return true;
    }
    if(!vmx.io.I2C_Read(i2c_res_handle, deviceAddress, registerAddress, data, count, &vmxerr))
    {
        printf("Error reading I2C bus!");
        return true;
    }
    else
    {
        return false;
    }
}

void Cobra::Delay(double seconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(seconds * 1000)));
}

int Cobra::ReadRegister(uint8_t location)
{
    uint8_t buffer[2];
    uint8_t data[1];
    data[0] = POINTER_CONVERT;
    WriteI2C(location, data);
    Delay(DELAY);
    ReadI2C(2, location, buffer);
    return (int)((buffer[0]<<8) & 0xFF00) | (buffer[1] & 0xFF);
}

bool Cobra::IsConnected()
{
    uint8_t partID = 0;
    
    if(ReadI2C(1, 0, &partID))
    {
        printf("Cobra could not be found!");
        return false;
    }
    else
    {
        printf("Cobra is Connected!");
        return true;
    }
}

int Cobra::GetSingle(uint8_t channel)
{
    if (channel > 3)
    {
        return 0;
    }
    
    int config = CONFIG_OS_SINGLE | mode | sampleRate;
    config |= gain;
    
    switch (channel)
    {
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
    WriteI2C(POINTER_CONVERT, raw);
    Delay(DELAY);
    return ReadRegister(POINTER_CONVERT)>>4;
}