#ifndef TITAN_H
#define TITAN_H

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <sstream>
#include <iomanip>
#include <memory>
#include "VMXPi.h"

namespace studica_driver
{

/* CAN Protocol - aligned with Titan2 Core/Inc/can_protocol.h */
#define CAN_DEVICE_TYPE        0x02000000
#define CAN_MANUFACTURER_ID    0x000C0000
#define CAN_OFFSET             64

/* Command offsets (Host -> Titan) */
#define CMD_DISABLED_FLAG            0x00
#define CMD_ENABLED_FLAG             0x01
#define CMD_SET_MOTOR_SPEED          0x02
#define CMD_DISABLE_MOTOR            0x03
#define CMD_GET_TITAN_INFO           0x04
#define CMD_GET_UNIQUE_ID            0x06
#define CMD_CONFIG_MOTOR             0x0A
#define CMD_GET_MOTOR_FREQUENCY      0x0B
#define CMD_RESET_ENCODER            0x0D
#define CMD_SET_CURRENT_LIMIT        0x0E
#define CMD_SET_MOTOR_STOP_MODE      0x0F
#define CMD_SET_TARGET_VELOCITY      0x10
#define CMD_SET_TARGET_DISTANCE      0x11
#define CMD_SET_ENCODER_RESOLUTION   0x12
#define CMD_SET_CURRENT_LIMIT_MODE   0x13
#define CMD_SET_PID_TYPE             0x14
#define CMD_AUTOTUNE_ALL             0x15
#define CMD_SET_SENSITIVITY           0x16
#define CMD_SET_TARGET_ANGLE         0x17
#define CMD_SET_POSITION_HOLD        0x18

/* Response offsets (Titan -> Host) */
#define RSP_TITAN_INFO                0x05
#define RSP_UID_WORD_1                0x07
#define RSP_UID_WORD_2                0x08
#define RSP_UID_WORD_3                0x09
#define RSP_MOTOR_FREQUENCY           0x0C
#define RSP_CYPHER_OUTPUT             0x24
#define RSP_ENCODER_0                 0x25
#define RSP_ENCODER_1                 0x26
#define RSP_ENCODER_2                 0x27
#define RSP_ENCODER_3                 0x28
#define RSP_RPM_0                     0x29
#define RSP_RPM_1                     0x2A
#define RSP_RPM_2                     0x2B
#define RSP_RPM_3                     0x2C
#define RSP_LIMIT_SWITCH              0x2D
#define RSP_MCU_TEMP                  0x2F

class Titan
{
public:
    Titan(const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~Titan();
    void Enable(bool enable);
    void SetupEncoder(uint8_t encoder);
    uint8_t GetID();
    uint16_t GetFrequency();
    std::string GetFirmwareVersion();
    std::string GetHardwareVersion();
    float GetControllerTemp();
    bool GetLimitSwitch(uint8_t motor, uint8_t direction);
    int16_t GetRPM(uint8_t motor);
    std::string GetSerialNumber();
    double GetEncoderDistance(uint8_t motor);
    int32_t GetEncoderCount(uint8_t motor);
    void ConfigureEncoder(uint8_t motor, double cfg);
    void ResetEncoder(uint8_t motor);
    double GetCypherAngle(uint8_t port);
    void SetSpeed(uint8_t motor, double speedCfg);
    void InvertMotorDirection(uint8_t motor);
    void InvertMotorRPM(uint8_t motor);
    void InvertEncoderDirection(uint8_t motor);
    void InvertMotor(uint8_t motor);

private:
    std::shared_ptr<VMXPi> vmx_;
    uint8_t canID_;
    uint16_t motorFreq_;
    uint8_t nEncoder_;
    float distPerTick_;

    VMXCANReceiveStreamHandle canrxhandle = 0;
    VMXErrorCode vmxerr;

    /** CAN ID = (DEVICE_TYPE + MANUFACTURER_ID + canID) + (offset * CAN_OFFSET) */
    uint32_t GetSendID(uint8_t cmd_offset) const;
    uint32_t GetRecvID(uint8_t rsp_offset) const;

    bool Write(uint32_t address, const uint8_t* data, int32_t periodMS);
    bool Read(uint32_t address, uint8_t* data);

    /* Last duty 0-100 for each channel (SET_MOTOR_SPEED sends all 4) */
    uint8_t last_duty_[4] = {0, 0, 0, 0};

    bool invertMotor0 = false;
    bool invertRPM0 = false;
    bool invertEncoder0 = false;
    bool invertMotor1 = false;
    bool invertRPM1 = false;
    bool invertEncoder1 = false;
    bool invertMotor2 = false;
    bool invertRPM2 = false;
    bool invertEncoder2 = false;
    bool invertMotor3 = false;
    bool invertRPM3 = false;
    bool invertEncoder3 = false;

    double distPerTick_0 = 0;
    double distPerTick_1 = 0;
    double distPerTick_2 = 0;
    double distPerTick_3 = 0;
};

} // namespace studica_driver

#endif // TITAN_H
