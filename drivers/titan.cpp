#include "titan.hpp"

using namespace studica_driver;

static constexpr float kRpmScale = 100.0f; /* matches firmware RPM_SCALE (CAN_RPM_* and SET_TARGET_VELOCITY) */

static bool UnpackRpmX100FromCanData(const uint8_t* data, float* out_rpm)
{
    if (out_rpm == nullptr)
        return false;
    *out_rpm = 0.0f;
    int32_t raw = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                       (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
    *out_rpm = static_cast<float>(raw) / kRpmScale;
    return true;
}

Titan::Titan(const uint8_t& canID, const uint16_t& motorFreq, const float& distPerTick, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx)
    , canID_(canID)
    , motorFreq_(motorFreq)
    , distPerTick_(distPerTick)
{
    try
    {
        if (vmx_->IsOpen())
        {
            if (!vmx_->can.OpenReceiveStream(canrxhandle, 0x0, 0x0, 100, &vmxerr))
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
            // ID = canID;
            uint8_t data[8] = {
                0, static_cast<uint8_t>((motorFreq & 0xFF)), static_cast<uint8_t>((motorFreq >> 8)), 0, 0, 0, 0, 0};
            for (int i = 0; i < 4; i++)
            {
                data[0] = i; // motor #
                Write(GetAddress(CONFIG_MOTOR), data, 0);
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

Titan::~Titan()
{
}

uint32_t Titan::GetAddress(uint32_t addressBase) const
{
    return addressBase + canID_;
}

void Titan::SetupEncoder(uint8_t encoder)
{
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

bool Titan::ReadWithFreshFlag(uint32_t address, uint8_t* data, bool& is_fresh, uint64_t* out_timestamp_us)
{
    VMXCANTimestampedMessage blackboard_msg;
    uint64_t sys_timestamp;
    bool already_retrieved = false;
    if (!vmx_->can.GetBlackboardEntry(canrxhandle, address, blackboard_msg, sys_timestamp, already_retrieved, &vmxerr))
    {
        return false;
    }
    else
    {
        std::memcpy(data, blackboard_msg.data, 8);
        if (out_timestamp_us != nullptr)
            *out_timestamp_us = sys_timestamp;
        is_fresh = !already_retrieved;
        return true;
    }
    return true;
}

bool Titan::GetEncoderDistanceFresh(uint8_t motor, double& distance, bool& is_fresh, uint64_t* out_timestamp_us)
{
    uint8_t data[8] = {0};
    uint32_t addr;
    if      (motor == 0) addr = GetAddress(ENCODER_0);
    else if (motor == 1) addr = GetAddress(ENCODER_1);
    else if (motor == 2) addr = GetAddress(ENCODER_2);
    else if (motor == 3) addr = GetAddress(ENCODER_3);
    else                 addr = GetAddress(ENCODER_0);
    if (!ReadWithFreshFlag(addr, data, is_fresh, out_timestamp_us))
        return false;
    int32_t count = static_cast<int32_t>(
        data[0] | (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
    if ((motor == 0 && invertEncoder0) || (motor == 1 && invertEncoder1) ||
        (motor == 2 && invertEncoder2) || (motor == 3 && invertEncoder3))
        count *= -1;
    double dpt = 0.0;
    if      (motor == 0) dpt = distPerTick_0;
    else if (motor == 1) dpt = distPerTick_1;
    else if (motor == 2) dpt = distPerTick_2;
    else if (motor == 3) dpt = distPerTick_3;
    distance = count * dpt;
    return true;
}

void Titan::Enable(bool enable)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (enable)
    {
        Titan::Write(GetAddress(ENABLED_FLAG), data, 100);
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            Titan::Write(GetAddress(DISABLED_FLAG), data, 10);
            vmx_->time.DelayMilliseconds(50);
        }
        for (int i = 0; i < 4; i++)
            lastDuty_[i] = 0;
    }
}

bool Titan::EnsureTitanInfoCached()
{
    if (cached_titan_info_valid_)
        return true;
    uint8_t req[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    const int max_attempts = 4;
    const int delay_ms = 40;
    for (int attempt = 0; attempt < max_attempts; attempt++)
    {
        Write(GetAddress(GET_TITAN_INFO), req, 0);
        vmx_->time.DelayMilliseconds(delay_ms);
        if (Read(GetAddress(RETURN_TITAN_INFO), cached_titan_info_))
        {
            cached_titan_info_valid_ = true;
            return true;
        }
    }
    return false;
}

uint8_t Titan::GetID()
{
    EnsureTitanInfoCached();
    return cached_titan_info_valid_ ? cached_titan_info_[0] : 0;
}

uint16_t Titan::GetFrequency()
{
    uint8_t data[8];
    Titan::Read(GetAddress(RETURN_MOTOR_FREQUENCY), data);
    return data[0] + (data[1] << 8);
}

std::string Titan::GetFirmwareVersion()
{
    if (!EnsureTitanInfoCached())
        return "Firmware Version: (read failed)";
    std::string result;
    result += "Firmware Version: [";
    result += std::to_string(cached_titan_info_[1]);
    result += ".";
    result += std::to_string(cached_titan_info_[2]);
    result += ".";
    result += std::to_string(cached_titan_info_[3]);
    result += "]";
    return result;
}

std::string Titan::GetHardwareVersion()
{
    if (!EnsureTitanInfoCached())
        return "Hardware: (read failed)";
    if ((int)cached_titan_info_[4] == 1)
    {
        return "Hardware: Titan Quad, Version: " + std::to_string(cached_titan_info_[5]);
    }
    else if ((int)cached_titan_info_[4] == 2)
    {
        return "Hardware: Titan Small, Version: " + std::to_string(cached_titan_info_[5]);
    }
    else
    {
        return "Hardware: Type " + std::to_string((int)cached_titan_info_[4]) + ", Rev " +
               std::to_string((int)cached_titan_info_[5]) + " (unknown type)";
    }
}

float Titan::GetControllerTemp()
{
    uint8_t data[8];
    Titan::Read(GetAddress(MCU_TEMP), data);
    return data[0] + (data[1] / 100.0);
}

bool Titan::GetLimitSwitch(uint8_t motor, uint8_t direction)
{
    uint8_t data[8];
    Titan::Read(GetAddress(LIMIT_SWITCH), data);
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

bool Titan::GetLimitSwitchesFresh(bool fwd[4], bool rev[4], bool& is_fresh, uint64_t* out_timestamp_us)
{
    uint8_t data[8] = {0};
    if (!ReadWithFreshFlag(GetAddress(LIMIT_SWITCH), data, is_fresh, out_timestamp_us))
        return false;
    for (int i = 0; i < 4; i++) {
        fwd[i] = (data[i * 2]     == 1);
        rev[i] = (data[i * 2 + 1] == 1);
    }
    return true;
}

float Titan::GetRPM(uint8_t motor)
{
    float v = 0.0f;
    TryGetRPM(motor, &v);
    return v;
}

bool Titan::TryGetRPM(uint8_t motor, float* out_rpm)
{
    if (out_rpm == nullptr)
        return false;
    *out_rpm = 0.0f;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t addr;
    if (motor == 0)
        addr = GetAddress(CAN_RPM_0);
    else if (motor == 1)
        addr = GetAddress(CAN_RPM_1);
    else if (motor == 2)
        addr = GetAddress(CAN_RPM_2);
    else if (motor == 3)
        addr = GetAddress(CAN_RPM_3);
    else
        addr = GetAddress(CAN_RPM_0);
    if (!Read(addr, data))
        return false;
    UnpackRpmX100FromCanData(data, out_rpm);
    if ((motor == 0 && invertRPM0) ||
        (motor == 1 && invertRPM1) ||
        (motor == 2 && invertRPM2) ||
        (motor == 3 && invertRPM3))
        *out_rpm = -*out_rpm;
    return true;
}

bool Titan::GetRPMFresh(uint8_t motor, float& rpm, bool& is_fresh, uint64_t* out_timestamp_us)
{
    rpm = 0.0f;
    uint8_t data[8] = {0};
    uint32_t addr;
    if (motor == 0)
        addr = GetAddress(CAN_RPM_0);
    else if (motor == 1)
        addr = GetAddress(CAN_RPM_1);
    else if (motor == 2)
        addr = GetAddress(CAN_RPM_2);
    else if (motor == 3)
        addr = GetAddress(CAN_RPM_3);
    else
        addr = GetAddress(CAN_RPM_0);
    if (!ReadWithFreshFlag(addr, data, is_fresh, out_timestamp_us))
        return false;
    UnpackRpmX100FromCanData(data, &rpm);
    if ((motor == 0 && invertRPM0) ||
        (motor == 1 && invertRPM1) ||
        (motor == 2 && invertRPM2) ||
        (motor == 3 && invertRPM3))
        rpm = -rpm;
    return true;
}

std::string Titan::GetSerialNumber()
{
    uint8_t data1[8];
    uint8_t data2[8];
    uint8_t data3[8];
    std::stringstream stream1;
    std::stringstream stream2;
    std::stringstream stream3;
    Titan::Read(GetAddress(RETURN_WORD_1), data1);
    Titan::Read(GetAddress(RETURN_WORD_2), data2);
    Titan::Read(GetAddress(RETURN_WORD_3), data3);
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
        Titan::Read(GetAddress(ENCODER_0), data);
        ticks = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
        if (invertEncoder0)
        {
            ticks *= -1;
        }
        return ticks * distPerTick_0;
    }
    if (motor == 1)
    {
        Titan::Read(GetAddress(ENCODER_1), data);
        ticks = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
        if (invertEncoder1)
        {
            ticks *= -1;
        }
        return ticks * distPerTick_1;
    }
    if (motor == 2)
    {
        Titan::Read(GetAddress(ENCODER_2), data);
        ticks = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
        if (invertEncoder2)
        {
            ticks *= -1;
        }
        return ticks * distPerTick_2;
    }
    if (motor == 3)
    {
        Titan::Read(GetAddress(ENCODER_3), data);
        ticks = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
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
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int32_t ticks = 0;
    if (motor == 0)
    {
        if (!Titan::Read(GetAddress(ENCODER_0), data))
            return 0;
        ticks = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
        if (invertEncoder0)
        {
            ticks *= -1;
        }
    }
    else if (motor == 1)
    {
        if (!Titan::Read(GetAddress(ENCODER_1), data))
            return 0;
        ticks = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
        if (invertEncoder1)
        {
            ticks *= -1;
        }
    }
    else if (motor == 2)
    {
        if (!Titan::Read(GetAddress(ENCODER_2), data))
            return 0;
        ticks = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
        if (invertEncoder2)
        {
            ticks *= -1;
        }
    }
    else if (motor == 3)
    {
        if (!Titan::Read(GetAddress(ENCODER_3), data))
            return 0;
        ticks = static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
                                     (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
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
    Titan::Write(GetAddress(RESET_ENCODER), data, 0);
}

double Titan::GetCypherAngle(uint8_t port)
{
    uint8_t data[8];
    int index = port * 2;
    Titan::Read(GetAddress(CYPHER_OUTPUT), data);
    return ((static_cast<double>(data[index]) + (static_cast<double>(data[index + 1] << 8))) / 100.0);
}

bool Titan::GetCypherAnglesFresh(double angles[4], bool& is_fresh, uint64_t* out_timestamp_us)
{
    uint8_t data[8] = {0};
    if (!ReadWithFreshFlag(GetAddress(CYPHER_OUTPUT), data, is_fresh, out_timestamp_us))
        return false;
    for (int i = 0; i < 4; i++)
        angles[i] = (static_cast<double>(data[i * 2]) + static_cast<double>(data[i * 2 + 1] << 8)) / 100.0;
    return true;
}

void Titan::SetSpeed(uint8_t motor, double speedCfg)
{
    if (motor > 3)
        return;
    /* Apply invert (direction) for this channel. */
    if (motor == 0 && invertMotor0)
        speedCfg *= -1;
    else if (motor == 1 && invertMotor1)
        speedCfg *= -1;
    else if (motor == 2 && invertMotor2)
        speedCfg *= -1;
    else if (motor == 3 && invertMotor3)
        speedCfg *= -1;
    if (speedCfg > 1.0)
        speedCfg = 1.0;
    if (speedCfg < -1.0)
        speedCfg = -1.0;
    int duty = static_cast<int>(speedCfg >= 0 ? speedCfg * 100.0 : -speedCfg * 100.0);
    if (duty > 100)
        duty = 100;
    if (duty < 0)
        duty = 0;
    lastDuty_[motor] = static_cast<uint8_t>(duty);
    uint8_t inA, inB;
    if (speedCfg == 0.0) {
        inA = 1; inB = 1;                        // brake: both high = H-bridge short
    } else if (speedCfg > 0.0) {
        inA = 1; inB = 0;                        // forward
        lastDirection_ |= (1u << motor);
    } else {
        inA = 0; inB = 1;                        // reverse
        lastDirection_ &= ~(1u << motor);
    }
    /* Titan format: one frame per motor [motor, duty, inA, inB] */
    uint8_t data[8] = {motor, static_cast<uint8_t>(duty), inA, inB, 0, 0, 0, 0};
    Titan::Write(GetAddress(SET_MOTOR_SPEED), data, 0);
}

void Titan::SetSpeedAll(double duty)
{
    if (duty > 1.0)
        duty = 1.0;
    if (duty < 0.0)
        duty = 0.0;
    int d = static_cast<int>(duty * 100.0);
    if (d > 100)
        d = 100;
    if (d < 0)
        d = 0;
    uint8_t u = static_cast<uint8_t>(d);
    lastDuty_[0] = lastDuty_[1] = lastDuty_[2] = lastDuty_[3] = u;
    lastDirection_ = 0x0F; /* all forward */
    /* Titan format: one frame per motor [motor, duty, inA, inB]; send 4 frames. */
    for (uint8_t m = 0; m < 4; m++)
    {
        uint8_t data[8] = {m, u, 1, 0, 0, 0, 0, 0};
        Write(GetAddress(SET_MOTOR_SPEED), data, 0);
    }
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

float Titan::GetTargetRPM(uint8_t motor)
{
    float v = 0.f;
    TryGetTargetRPM(motor, &v);
    return v;
}

bool Titan::TryGetTargetRPM(uint8_t motor, float* out_rpm)
{
    if (out_rpm == nullptr || motor >= 4)
        return false;
    *out_rpm = 0.0f;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t addr;
    if (motor == 0)
        addr = GetAddress(TARGET_RPM_0);
    else if (motor == 1)
        addr = GetAddress(TARGET_RPM_1);
    else if (motor == 2)
        addr = GetAddress(TARGET_RPM_2);
    else
        addr = GetAddress(TARGET_RPM_3);
    if (!Read(addr, data))
        return false;
    UnpackRpmX100FromCanData(data, out_rpm);
    return true;
}

bool Titan::TryGetTargetRPMFromAll(float targetRpm[4])
{
    if (targetRpm == nullptr)
        return false;
    bool ok = true;
    for (int m = 0; m < 4; m++)
    {
        if (!TryGetTargetRPM(m, &targetRpm[m]))
            ok = false;
    }
    return ok;
}

void Titan::SetTargetVelocity(uint8_t motor, float velocityRpm)
{
    if (motor >= 4)
        return;
    float rpmScaled = velocityRpm * kRpmScale;
    int32_t rpm32 =
        (rpmScaled >= 0.0f) ? static_cast<int32_t>(rpmScaled + 0.5f) : static_cast<int32_t>(rpmScaled - 0.5f);
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = motor;
    data[1] = static_cast<uint8_t>(rpm32 & 0xFF);
    data[2] = static_cast<uint8_t>((rpm32 >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>((rpm32 >> 16) & 0xFF);
    data[4] = static_cast<uint8_t>((rpm32 >> 24) & 0xFF);
    Write(GetAddress(SET_TARGET_VELOCITY), data, 0);
}

void Titan::SetTargetDistance(uint8_t motor, int32_t distanceCounts)
{
    if (motor >= 4)
        return;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = motor;
    data[1] = static_cast<uint8_t>(distanceCounts & 0xFF);
    data[2] = static_cast<uint8_t>((distanceCounts >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>((distanceCounts >> 16) & 0xFF);
    Write(GetAddress(SET_TARGET_DISTANCE), data, 0);
}

void Titan::SetTargetAngle(uint8_t motor, double angleDeg)
{
    if (motor >= 4)
        return;
    if (angleDeg < 0.0)
        angleDeg = 0.0;
    if (angleDeg > 360.0)
        angleDeg = 360.0;
    uint16_t angle_x100 = static_cast<uint16_t>(angleDeg * 100.0);
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = motor;
    data[1] = static_cast<uint8_t>(angle_x100 & 0xFF);
    data[2] = static_cast<uint8_t>((angle_x100 >> 8) & 0xFF);
    Write(GetAddress(SET_TARGET_ANGLE), data, 0);
}

void Titan::SetPositionHold(uint8_t motor, bool hold)
{
    if (motor >= 4)
        return;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = motor;
    data[1] = hold ? 1 : 0;
    Write(GetAddress(SET_POSITION_HOLD), data, 0);
}

void Titan::SetEncoderResolution(uint8_t channel, uint16_t cpr)
{
    if (channel >= 4)
        return;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = channel;
    data[1] = static_cast<uint8_t>(cpr & 0xFF);
    data[2] = static_cast<uint8_t>((cpr >> 8) & 0xFF);
    Write(GetAddress(SET_ENCODER_RESOLUTION), data, 0);
}

void Titan::SetCurrentLimit(uint8_t channel, float limitAmps)
{
    if (channel >= 4)
        return;
    if (limitAmps < 0.0f)
        limitAmps = 0.0f;
    /* Titan: send limit in 0.01A (e.g. 150 = 1.5A); firmware may need (float)limit/100.0f */
    uint16_t limit_x100 = static_cast<uint16_t>(limitAmps * 100.0f);
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = channel;
    data[1] = static_cast<uint8_t>(limit_x100 & 0xFF);
    data[2] = static_cast<uint8_t>((limit_x100 >> 8) & 0xFF);
    Write(GetAddress(SET_CURRENT_LIMIT), data, 0);
}

void Titan::SetCurrentLimitMode(uint8_t channel, uint8_t mode)
{
    if (channel >= 4)
        return;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = channel;
    data[1] = (mode != 0) ? 1 : 0;
    Write(GetAddress(SET_CURRENT_LIMIT_MODE), data, 0);
}

void Titan::SetMotorStopMode(uint8_t mode)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = mode;
    Write(GetAddress(SET_MOTOR_STOP_MODE), data, 0);
}

void Titan::SetPIDType(uint8_t type)
{
    if (type > TITAN_PID_TYPE_MAX)
        return;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = STUDICA_CAN_PIDTYPE_BROADCAST;
    data[1] = type;
    data[2] = STUDICA_CAN_PIDTYPE_PAYLOAD_SIG;
    Write(GetAddress(SET_PID_TYPE), data, 0);
}

void Titan::SetMotorPIDType(uint8_t motor, uint8_t type)
{
    if (motor >= 4 || type > TITAN_PID_TYPE_MAX)
        return;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = motor;
    data[1] = type;
    data[2] = STUDICA_CAN_PIDTYPE_PAYLOAD_SIG;
    Write(GetAddress(SET_PID_TYPE), data, 0);
}

void Titan::AutotuneAll()
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    Write(GetAddress(AUTOTUNE_ALL), data, 0);
}

void Titan::AutotuneMotor(uint8_t motor)
{
    if (motor >= 4)
        return;
    uint8_t data[8] = {motor, 0, 0, 0, 0, 0, 0, 0};
    Write(GetAddress(AUTOTUNE_MOTOR), data, 0);
}

void Titan::SetSensitivity(uint8_t motor, uint8_t sensitivity)
{
    if (motor >= 4 || sensitivity > 10)
        return;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = motor;
    data[1] = sensitivity;
    Write(GetAddress(SET_SENSITIVITY), data, 0);
}

void Titan::SetCANSensorOsDelay(uint16_t periodMs)
{
    if (periodMs < 5u)
        periodMs = 5u;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = static_cast<uint8_t>(periodMs & 0xFFu);
    data[1] = static_cast<uint8_t>((periodMs >> 8) & 0xFFu);
    Write(GetAddress(SET_CAN_SENSOR_OS_DELAY), data, 0);
}

void Titan::DisableMotor(uint8_t motor)
{
    if (motor >= 4)
        return;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = motor;
    Write(GetAddress(DISABLE_MOTOR), data, 0);
}