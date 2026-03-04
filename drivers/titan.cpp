#include "titan.h"
#include <cmath>

using namespace studica_driver;

uint32_t Titan::GetSendID(uint8_t cmd_offset) const
{
    return (CAN_DEVICE_TYPE + CAN_MANUFACTURER_ID + canID_) + (cmd_offset * CAN_OFFSET);
}

uint32_t Titan::GetRecvID(uint8_t rsp_offset) const
{
    return (CAN_DEVICE_TYPE + CAN_MANUFACTURER_ID + canID_) + (rsp_offset * CAN_OFFSET);
}

Titan::Titan(const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, std::shared_ptr<VMXPi> vmx)
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
                Write(GetSendID(CMD_CONFIG_MOTOR), data, 0);
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
 
void Titan::Enable(bool enable)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (enable)
        Titan::Write(GetSendID(CMD_ENABLED_FLAG), data, 10);
    else
        Titan::Write(GetSendID(CMD_DISABLED_FLAG), data, 10);
}
 
uint8_t Titan::GetID()
{
    uint8_t data[8];
    Titan::Read(GetRecvID(RSP_TITAN_INFO), data);
    return data[0];
}
 
uint16_t Titan::GetFrequency()
{
    uint8_t data[8];
    Titan::Read(GetRecvID(RSP_MOTOR_FREQUENCY), data);
    return data[0] + (data[1] << 8);
}
 
std::string Titan::GetFirmwareVersion()
{
    std::string result;
    uint8_t data[8];
    Titan::Read(GetRecvID(RSP_TITAN_INFO), data);
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
    Titan::Read(GetRecvID(RSP_TITAN_INFO), data);
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
    Titan::Read(GetRecvID(RSP_MCU_TEMP), data);
    return data[0] + (data[1] / 100.0);
}
 
bool Titan::GetLimitSwitch(uint8_t motor, uint8_t direction)
{
    uint8_t data[8];
    Titan::Read(GetRecvID(RSP_LIMIT_SWITCH), data);
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
    if (motor < 4)
        Read(GetRecvID(static_cast<uint8_t>(RSP_RPM_0 + motor)), data);
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
    Titan::Read(GetRecvID(RSP_UID_WORD_1), data1);
    Titan::Read(GetRecvID(RSP_UID_WORD_2), data2);
    Titan::Read(GetRecvID(RSP_UID_WORD_3), data3);
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
    if (motor >= 4) return -1;
    uint8_t data[8];
    Titan::Read(GetRecvID(static_cast<uint8_t>(RSP_ENCODER_0 + motor)), data);
    int32_t ticks = static_cast<int32_t>(
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 24)
    );
    if ((motor == 0 && invertEncoder0) || (motor == 1 && invertEncoder1) ||
        (motor == 2 && invertEncoder2) || (motor == 3 && invertEncoder3))
        ticks *= -1;
    double dpt = (motor == 0) ? distPerTick_0 : (motor == 1) ? distPerTick_1 : (motor == 2) ? distPerTick_2 : distPerTick_3;
    return ticks * dpt;
}
 
int32_t Titan::GetEncoderCount(uint8_t motor)
{
    if (motor >= 4) return 0;
    uint8_t data[8];
    Titan::Read(GetRecvID(static_cast<uint8_t>(RSP_ENCODER_0 + motor)), data);
    int32_t ticks = static_cast<int32_t>(
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 24)
    );
    if ((motor == 0 && invertEncoder0) || (motor == 1 && invertEncoder1) ||
        (motor == 2 && invertEncoder2) || (motor == 3 && invertEncoder3))
        ticks *= -1;
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
    Titan::Write(GetSendID(CMD_RESET_ENCODER), data, 0);
}
 
double Titan::GetCypherAngle(uint8_t port)
{
    uint8_t data[8];
    Titan::Read(GetRecvID(RSP_CYPHER_OUTPUT), data);
    int index = port * 2;
    if (index + 1 >= 8) return 0.0;
    uint16_t angle_x100 = static_cast<uint16_t>(data[index] | (data[index + 1] << 8));
    return static_cast<double>(angle_x100) / 100.0;
}
 
void Titan::SetSpeed(uint8_t motor, double speedCfg)
{
    if (motor >= 4) return;
    if (motor == 0 && invertMotor0) speedCfg *= -1;
    else if (motor == 1 && invertMotor1) speedCfg *= -1;
    else if (motor == 2 && invertMotor2) speedCfg *= -1;
    else if (motor == 3 && invertMotor3) speedCfg *= -1;
    if (speedCfg > 1.0) speedCfg = 1.0;
    if (speedCfg < -1.0) speedCfg = -1.0;
    int duty = static_cast<int>(std::abs(speedCfg) * 100.0);
    if (duty > 100) duty = 100;
    last_duty_[motor] = static_cast<uint8_t>(duty);
    uint8_t data[8] = { last_duty_[0], last_duty_[1], last_duty_[2], last_duty_[3], 0, 0, 0, 0 };
    Titan::Write(GetSendID(CMD_SET_MOTOR_SPEED), data, 0);
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
