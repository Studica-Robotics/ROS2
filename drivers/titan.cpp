#include "titan.h"

using namespace studica_driver;

 
Titan::Titan(const std::string &name, const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx), canID_(canID), motorFreq_(motorFreq), distPerTick_(distPerTick)
{
    try
    {
        if (vmx_->IsOpen())
        {
            if(!vmx_->can.OpenReceiveStream(canrxhandle, 0x0, 0x0, 100, &vmxerr))
            {
                printf("Error opening CAN RX Stream 0.\n");
            }
            else
            {
                printf("Opened CAN Receive Stream 0, handle:  %d\n", canrxhandle);
                if (vmx_->can.EnableReceiveStreamBlackboard(canrxhandle, true, &vmxerr))
                {
                    printf("Enabled Blackboard on Stream 0.\n");
                }
                else
                {
                    printf("Error Enabling Blackboard on Stream 0.\n");
                }
            }
            if (!vmx_->can.FlushRxFIFO(&vmxerr))
            {
                printf("Error Flushing CAN RX FIFO.\n");
            }
            else
            {
                printf("Flushed CAN RX FIFO\n");
            }
 
            if (!vmx_->can.FlushTxFIFO(&vmxerr))
            {
                printf("Error Flushing CAN TX FIFO.\n");
            }
            else
            {
                printf("Flushed CAN TX FIFO\n");
            }
 
            if (!vmx_->can.SetMode(VMXCAN::VMXCAN_NORMAL, &vmxerr))
            {
                printf("Error setting CAN Mode to Normal\n");
            }
            else
            {
                printf("Set CAN Mode to Normal.\n");
            }
            vmx_->time.DelayMilliseconds(20);
        }
        else
        {
            printf("Error:  Unable to open VMX Client.\n");
            printf("\n");
            printf("        - Is pigpio (or the system resources it requires) in use by another process?\n");
            printf("        - Does this application have root privileges?\n");
        }
    }
    catch (const std::exception& ex)
    {
        printf("Caught exception: %s", ex.what());
    }
 
    if (canID > 0 && canID < 64)
    {
        if (motorFreq <= 20000)
        {
            //ID = canID;
            uint8_t data[8] = {0, static_cast<uint8_t>((motorFreq & 0xFF)), static_cast<uint8_t>((motorFreq >> 8)), 0, 0, 0, 0, 0};
            for (int i = 0; i < 4; i++)
            {
                data[0] = i; // motor #
                Write(CONFIG_MOTOR, data, 0);
            }
            printf("Titan Driver Started!\n");
        }
        else
        {
            printf("Titan Motor Frequency %i is out of range. (0 - 20k)", motorFreq);
        }
    }
    else
    {
        printf("Titan CAN ID %i is out of range. (1 - 63)", canID);
    }
}
Titan::~Titan() {}

void Titan::SetupEncoder(uint8_t encoder) {
    ConfigureEncoder(encoder, distPerTick_);
    ResetEncoder(encoder);
}

bool Titan::Write(uint32_t address, const uint8_t* data, int32_t periodMS)
{
    VMXCANMessage msg;
    msg.dataSize = 8;
    msg.setData(data, 8);
    msg.messageID = address;
    if (!vmx_->can.SendMessage(msg, periodMS, &vmxerr))
    {
        return false;
    }
    return true;
}
 
bool Titan::Read(uint32_t address, uint8_t* data)
{
    VMXCANTimestampedMessage blackboard_msg;
    bool already_retrieved;
    uint64_t sys_timestamp; // We could allow the user to read the timestamp to in future
    if (!vmx_->can.GetBlackboardEntry(canrxhandle, address, blackboard_msg, sys_timestamp, already_retrieved, &vmxerr))
    {
        return false;
    }
    else
    {
        std::memcpy(data, blackboard_msg.data, 8);
        return true;
    }
    return true;
}
 
void Titan::Enable(bool enable) // Not really a fan of this should be tied to the watchdog somehow aka checking if the watchdog is fed.
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (enable)
    {
        Titan::Write(ENABLED_FLAG, data, 10); // repeat every 10ms to keep the titan enabled
    }
    else
    {
        Titan::Write(DISABLED_FLAG, data, 10); // repeat every 10ms to keep the titan disabled
    }
}
 
uint8_t Titan::GetID()
{
    uint8_t data[8];
    Titan::Read(RETURN_TITAN_INFO, data);
    return data[0];
}
 
uint16_t Titan::GetFrequency()
{
    uint8_t data[8];
    Titan::Read(RETURN_MOTOR_FREQUENCY, data);
    return data[0] + (data[1] << 8);
}
 
std::string Titan::GetFirmwareVersion()
{
    std::string result;
    uint8_t data[8];
    Titan::Read(RETURN_TITAN_INFO, data);
    result += "Firmware Version: [";
    result += std::to_string(data[1]);
    result += ".";
    result += std::to_string(data[2]);
    result += ".";
    result += std::to_string(data[3]);
    result += "]";
    return (result);  
}
 
std::string Titan::GetHardwareVersion()
{
    uint8_t data[8];
    Titan::Read(RETURN_TITAN_INFO, data);
    if ((int)data[4] == 1)
    {
        return "Hardware: Titan Quad, Version: " + std::to_string(data[5]);
    }
    else if ((int)data[4] == 2)
    {
        return "Hardware: Titan Small, Version: " + std::to_string(data[5]);
    }
    else
    {
        return ("No Hardware Found!");
    }
}
 
float Titan::GetControllerTemp()
{
    uint8_t data[8];
    Titan::Read(MCU_TEMP, data);
    return data[0] + (data[1] / 100.0);
}
 
bool Titan::GetLimitSwitch(uint8_t motor, uint8_t direction)
{
    uint8_t data[8];
    Titan::Read(LIMIT_SWITCH, data);
    uint8_t index = 0;
    if (direction == 1)
    {
        index = (motor * 2) + 1;
    }
    else
    {
        index = (motor * 2);
    }
    if (data[index] == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}
 
int16_t Titan::GetRPM(uint8_t motor)
{
    uint8_t data[8];
    if (motor == 0)
    {
        Read(RPM_0, data);
    }
    if (motor == 1)
    {
        Read(RPM_1, data);
    }
    if (motor == 2)
    {
        Read(RPM_2, data);
    }
    if (motor == 3)
    {
        Read(RPM_3, data);
    }
    return static_cast<int16_t>(data[0] | (data[1] << 8));
}
 
std::string Titan::GetSerialNumber()
{
    uint8_t data1[8];
    uint8_t data2[8];
    uint8_t data3[8];
    std::stringstream stream1;
    std::stringstream stream2;
    std::stringstream stream3;
    Titan::Read(RETURN_WORD_1, data1);
    Titan::Read(RETURN_WORD_2, data2);
    Titan::Read(RETURN_WORD_3, data3);
    int word1 = data1[0] + (data1[1] << 8) + (data1[2] << 16) + (data1[3] << 24);
    int word2 = data2[0] + (data2[1] << 8) + (data2[2] << 16) + (data2[3] << 24);
    int word3 = data3[0] + (data3[1] << 8) + (data3[2] << 16) + (data3[3] << 24);
    stream1 << std::setfill('0') << std::setw(8) << std::hex << std::uppercase << word1;
    stream2 << std::setfill('0') << std::setw(8) << std::hex << std::uppercase << word2;
    stream3 << std::setfill('0') << std::setw(8) << std::hex << std::uppercase << word3;
    std::string w1 = stream1.str();
    std::string w2 = stream2.str();
    std::string w3 = stream3.str();
    return (w1 + "-" + w2 + "-" + w3);
}
 
double Titan::GetEncoderDistance(uint8_t motor)
{
    uint8_t data[8];
    int32_t ticks = 0;
    if (motor == 0)
    {
        Titan::Read(ENCODER_0_COUNT, data);
        ticks = static_cast<int32_t>(data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
        if (invertEncoder0)
        {
            ticks *= -1;
        }
        return ticks * distPerTick_0;
    }
    if (motor == 1)
    {
        Titan::Read(ENCODER_1_COUNT, data);
        ticks = static_cast<int32_t>(data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
        if (invertEncoder1)
        {
            ticks *= -1;
        }
        return ticks * distPerTick_1;
    }
    if (motor == 2)
    {
        Titan::Read(ENCODER_2_COUNT, data);
        ticks = static_cast<int32_t>(data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
        if (invertEncoder2)
        {
            ticks *= -1;
        }
        return ticks * distPerTick_2;
    }
    if (motor == 3)
    {
        Titan::Read(ENCODER_3_COUNT, data);
        ticks = static_cast<int32_t>(data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
        if (invertEncoder3)
        {
            ticks *= -1;
        }
        return ticks * distPerTick_3;
    }
    return -1;
}
 
int32_t Titan::GetEncoderCount(uint8_t motor)
{
    uint8_t data[8];
    int32_t ticks = 0;
    if (motor == 0)
    {
        Titan::Read(ENCODER_0_COUNT, data);
 
        ticks = static_cast<int32_t>(data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
        if (invertEncoder0)
        {
            ticks *= -1;
        }
    }
    if (motor == 1)
    {
        Titan::Read(ENCODER_1_COUNT, data);
        ticks = static_cast<int32_t>(data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
        if (invertEncoder1)
        {
            ticks *= -1;
        }
    }
    if (motor == 2)
    {
        Titan::Read(ENCODER_2_COUNT, data);
        ticks = static_cast<int32_t>(data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
        if (invertEncoder2)
        {
            ticks *= -1;
        }
    }
    if (motor == 3)
    {
        Titan::Read(ENCODER_3_COUNT, data);
        ticks = static_cast<int32_t>(data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24));
        if (invertEncoder3)
        {
            ticks *= -1;
        }
    }
    return ticks;
}

void Titan::ConfigureEncoder(uint8_t motor, double cfg)
{
    if (motor == 0)
    {
        distPerTick_0 = cfg;
    }
    if (motor == 1)
    {
        distPerTick_1 = cfg;
    }
    if (motor == 2)
    {
        distPerTick_2 = cfg;
    }
    if (motor == 3)
    {
        distPerTick_3 = cfg;
    }
}
 
void Titan::ResetEncoder(uint8_t motor)
{
    uint8_t data[8] = {motor, 0, 0, 0, 0, 0, 0, 0};
    Titan::Write(RESET_ENCODER, data, 0);
}
 
double Titan::GetCypherAngle(uint8_t port)
{
    uint8_t data[8];
    int index = port * 2;
    Titan::Read(CYPHER_OUTPUT, data);
    return ((static_cast<double>(data[index]) + (static_cast<double>(data[index+1] << 8))) / 100.0);
}
 
void Titan::SetSpeed(uint8_t motor, double speedCfg)
{
    uint8_t inA = 0;
    uint8_t inB = 0;
    if (motor == 0 && invertMotor0)
    {
        speedCfg *= -1;
    }
    else if (motor == 1 && invertMotor1)
    {
        speedCfg *= -1;
    }
    else if (motor == 2 && invertMotor2)
    {
        speedCfg *= -1;
    }
    else if (motor == 3 && invertMotor3)
    {
        speedCfg *= -1;
    }
    if (speedCfg <= 1.0 && speedCfg >= -1.0)
    {
        speedCfg = speedCfg * 100;
        if (speedCfg == 0)
        {
            inA = 1;
            inB = 1;
        }
        else if (speedCfg > 0)
        {
            inA = 1;
            inB = 0;
        }
        else if (speedCfg < 0)
        {
            inA = 0;
            inB = 1;
        }
        else
        {
            inA = 0;
            inB = 0;
        }
    }
    uint8_t data[8] = {motor, static_cast<uint8_t>(abs(speedCfg)), inA, inB, 0, 0, 0, 0};
    Titan::Write(SET_MOTOR_SPEED, data, 0);
}
 
void Titan::InvertMotorDirection(uint8_t motor)
{
    if (motor == 0)
    {
        invertMotor0 = true;
    }
    if (motor == 1)
    {
        invertMotor1 = true;
    }
    if (motor == 2)
    {
        invertMotor2 = true;
    }
    if (motor == 3)
    {
        invertMotor3 = true;
    }
}
 
void Titan::InvertMotorRPM(uint8_t motor)
{
    if (motor == 0)
    {
        invertRPM0 = true;
    }
    if (motor == 1)
    {
        invertRPM1 = true;
    }
    if (motor == 2)
    {
        invertRPM2 = true;
    }
    if (motor == 3)
    {
        invertRPM3 = true;
    }
}
 
void Titan::InvertEncoderDirection(uint8_t motor)
{
    if (motor == 0)
    {
        invertEncoder0 = true;
    }
    if (motor == 1)
    {
        invertEncoder1 = true;
    }
    if (motor == 2)
    {
        invertEncoder2 = true;
    }
    if (motor == 3)
    {
        invertEncoder3 = true;
    }
}

void Titan::InvertMotor(uint8_t motor)
{
    InvertMotorDirection(motor);
    InvertMotorRPM(motor);
    InvertEncoderDirection(motor);
}
